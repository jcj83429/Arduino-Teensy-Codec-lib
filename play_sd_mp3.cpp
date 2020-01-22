/*
	Arduino Audiocodecs

	Copyright (c) 2014 Frank Bösing

	This library is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this library.  If not, see <http://www.gnu.org/licenses/>.

	The helix decoder itself as a different license, look at the subdirectories for more info.

	Diese Bibliothek ist freie Software: Sie können es unter den Bedingungen
	der GNU General Public License, wie von der Free Software Foundation,
	Version 3 der Lizenz oder (nach Ihrer Wahl) jeder neueren
	veröffentlichten Version, weiterverbreiten und/oder modifizieren.

	Diese Bibliothek wird in der Hoffnung, dass es nützlich sein wird, aber
	OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite
	Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK.
	Siehe die GNU General Public License für weitere Details.

	Sie sollten eine Kopie der GNU General Public License zusammen mit diesem
	Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.

	Der Helixdecoder selbst hat eine eigene Lizenz, bitte für mehr Informationen
	in den Unterverzeichnissen nachsehen.

 */

 /* The Helix-Library is modified for Teensy 3.1 */

// Total RAM Usage: 35 KB


#include "play_sd_mp3.h"

#define MP3_SD_BUF_SIZE	4096 								//Enough space for 2 complete stereo frames. If not using seeking, then only half of this is needed
#define MP3_BUF_SIZE	(MAX_NCHAN * MAX_NGRAN * MAX_NSAMP) //MP3 output buffer
#define DECODE_NUM_STATES 2									//How many steps in decode() ?

#define MIN_FREE_RAM (35+1)*1024 // 1KB Reserve

//#define CODEC_DEBUG

static AudioPlaySdMp3 	*mp3objptr;
void decodeMp3(void);

void AudioPlaySdMp3::stop(void)
{
	NVIC_DISABLE_IRQ(IRQ_AUDIOCODEC);
	playing = codec_stopped;
	if (buf[1]) {free(buf[1]);buf[1] = NULL;}
	if (buf[0]) {free(buf[0]);buf[0] = NULL;}
	freeBuffer();
	if (hMP3Decoder) {MP3FreeDecoder(hMP3Decoder);hMP3Decoder=NULL;};
}
/*
float AudioPlaySdMp3::processorUsageMaxDecoder(void){
	//this is somewhat incorrect, it does not take the interruptions of update() into account -
	//therefore the returned number is too high.
	//Todo: better solution
	return (decode_cycles_max / (0.026*F_CPU)) * 100;
};

float AudioPlaySdMp3::processorUsageMaxSD(void){
	//this is somewhat incorrect, it does not take the interruptions of update() into account -
	//therefore the returned number is too high.
	//Todo: better solution
	return (decode_cycles_max_sd / (0.026*F_CPU)) * 100;
};
*/

int AudioPlaySdMp3::play(void)
{
	lastError = ERR_CODEC_NONE;
	initVars();

	sd_buf = allocBuffer(MP3_SD_BUF_SIZE);
	if (!sd_buf) return ERR_CODEC_OUT_OF_MEMORY;

	mp3objptr = this;

	buf[0] = (short *) malloc(MP3_BUF_SIZE * sizeof(int16_t));
	buf[1] = (short *) malloc(MP3_BUF_SIZE * sizeof(int16_t));

	hMP3Decoder = MP3InitDecoder();

	if (!buf[0] || !buf[1] || !hMP3Decoder)
	{
		lastError = ERR_CODEC_OUT_OF_MEMORY;
		stop();
		return lastError;
	}

	//Read-ahead 10 Bytes to detect ID3
	sd_left =  fread(sd_buf, 10);

	//Skip ID3, if existent
	int skip = skipID3(sd_buf);
	if (skip) {
		size_id3 = skip;
		int b = skip & 0xfffffe00;
		fseek(b);
		sd_left = 0;
//		Serial.print("skip");
//		Serial.print(fposition());
	} else size_id3 = 0;

	//Fill buffer from the beginning with fresh data
	sd_left = fillReadBuffer(sd_buf, sd_buf, sd_left, MP3_SD_BUF_SIZE);

	if (!sd_left) {
		lastError = ERR_CODEC_FILE_NOT_FOUND;
		stop();
		return lastError;
	}
	
	
	// check and parse Xing header for VBR file duration
	vbr_total_frames = 0;
	int syncOffset = MP3FindSyncWord(sd_buf, sd_left);
	if (syncOffset <= sd_left - 160) { // xing header is less than 160 bytes
		XHEADDATA xingHeaderData;
		xingHeaderData.toc = NULL; // not reading TOC right now
		if(GetXingHeader(&xingHeaderData, sd_buf + syncOffset)){
			Serial.println("found xing VBR header");
			vbr_total_frames = xingHeaderData.frames;
		}
	}

	_VectorsRam[IRQ_AUDIOCODEC + 16] = &decodeMp3;
	initSwi();

	decoded_length[0] = 0;
	decoded_length[1] = 0;
	decoding_block = 0;
	decoding_state = 0;

	play_pos = 0;

	sd_p = sd_buf;

	for (size_t i=0; i< DECODE_NUM_STATES; i++) decodeMp3();

	if((mp3FrameInfo.samprate != AUDIOCODECS_SAMPLE_RATE ) || (mp3FrameInfo.bitsPerSample != 16) || (mp3FrameInfo.nChans > 2)) {
		//Serial.println("incompatible MP3 file.");
		lastError = ERR_CODEC_FORMAT;
		stop();
		return lastError;
	}
	decoding_block = 1;

	playing = codec_playing;

#ifdef CODEC_DEBUG
//	Serial.printf("RAM: %d\r\n",ram-freeRam());

#endif
    return lastError;
}

uint32_t AudioPlaySdMp3::timeMsToOffset(uint32_t timeMs, uint8_t *toc)
{
	uint64_t sizeWithoutID3 = fsize() - size_id3;
	if (vbr_total_frames && toc) {
		// use Xing DXHEAD for VBR
		return size_id3 + SeekPoint(toc, sizeWithoutID3, 100.0 * timeMs / lengthMillis());
	} else {
		// offset calculation for CBR
		return size_id3 + sizeWithoutID3 * timeMs / lengthMillis();
	}
}

uint32_t AudioPlaySdMp3::offsetToTimeMs(uint32_t offset, uint8_t *toc)
{
	uint32_t sizeWithoutID3 = fsize() - size_id3;
	if (vbr_total_frames && toc) {
		// use Xing DXHEAD for VBR
		float tocValue = 256.0 * (offset - size_id3) / sizeWithoutID3;
		int l = 0, h = 99, loffset = 0, hoffset = 255;
		if (tocValue > toc[99]) {
			// special case near end of file
			l = 99;
			loffset = toc[99];
			h = 100;
			hoffset = 256;
		} else {
			// do a binary search to find the toc entry that's just before the offset
			while (h - l >= 2){
				int mid = (h + l) / 2;
				if (toc[mid] < tocValue) {
					l = mid;
					loffset = toc[mid];
				} else if (toc[mid] > tocValue) {
					h = mid;
					hoffset = toc[mid];
				} else {
					// highly unlikely
					return lengthMillis() * mid / 100;
				}
			}
		}
		Serial.print("l="); Serial.print(l); Serial.print(", h="); Serial.println(h);
		return lengthMillis() * (l + (tocValue - loffset) / (hoffset - loffset) * (h - l)) / 100;
	} else {
		// calculation for CBR
		return (uint64_t)(offset - size_id3) * lengthMillis() / sizeWithoutID3;
	}
}

bool AudioPlaySdMp3::seek(uint32_t timesec)
{
	if (!isPlaying()) {
		return false;
	}

	pause(true);
	
	uint8_t toc[100];
	bool tocValid = false;
	if (vbr_total_frames) {
		// use TOC in VBR header for offset
		fseek(size_id3);
		fillReadBuffer(sd_buf, sd_buf, 0, MP3_SD_BUF_SIZE);
		int syncOffset = MP3FindSyncWord(sd_buf, MP3_SD_BUF_SIZE);
		if (syncOffset <= MP3_SD_BUF_SIZE - 160) {
			XHEADDATA xingHeaderData;
			xingHeaderData.toc = toc;
			if(GetXingHeader(&xingHeaderData, sd_buf + syncOffset)){
				Serial.println("using xing VBR header");
				tocValid = true;
			}
		}
	}
	
	uint32_t targetOffset = timeMsToOffset(timesec * 1000, tocValid ? toc : NULL);
	Serial.print("MP3 seeking to offset ");
	Serial.println(targetOffset);
	fseek(targetOffset);
	
	// clear and refill MP3 stream buffer
	sd_p = sd_buf;
	sd_left = fillReadBuffer(sd_buf, sd_buf, 0, MP3_SD_BUF_SIZE);
	if (!sd_left) {
		Serial.println("fillReadBuffer failed EOF?");
		stop();
		return true;
	}
	int validFrameFound = 0;
	// Sometimes after one successful decode after seek, the next decode can still fail.
	// I don't know if it's possible for a decode to fail after 2 successful decodes, but I will require 3 consecutive successful decodes to be safe.
	// The better way to handle it is to change decodeMp3 to not declare EOF as soon as it hits an error. For now this loop will do.
	while(validFrameFound < 3) {
		// MP3_SD_BUF_SIZE is not enough to fit 2 320kbps frames.
		// So advance to next MP3 frame and fill buffer again to avoid buffer underflow in decoding.
		int offset = MP3FindSyncWord(sd_p, sd_left);
		if (offset < 0) {
			Serial.println("No sync"); //no error at end of file
			stop();
			return true;
		} else {
			Serial.print("sync at buffer offset ");
			Serial.println(offset);
			sd_p += offset;
			sd_left -= offset;
			
			// The only way to fully know if we found a valid frame is by trying to decode it
			int decode_res = MP3Decode(hMP3Decoder, &sd_p, (int*)&sd_left, buf[decoding_block], 0);
			if (decode_res) {
				Serial.print("decode error ");
				Serial.println(decode_res);
				if (decode_res == ERR_MP3_INVALID_FRAMEHEADER) {
					// keep going to find next frameheader
					sd_p += 2;
					sd_left -= 2;
				}
				validFrameFound = 0;
			} else {
				// don't throw away the decoding result
				MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);
				decoded_length[decoding_block] = mp3FrameInfo.outputSamps;
				decoding_block = 1 - decoding_block;
				validFrameFound++;
			}
		}

		sd_left = fillReadBuffer(sd_buf, sd_p, sd_left, MP3_SD_BUF_SIZE);
		if (!sd_left) {
			Serial.println("fillReadBuffer failed EOF?");
			stop();
			return true;
		}
		sd_p = sd_buf;
	}

	decoding_state = 1; // we refilled the buffer
	
	decoded_length[decoding_block] = 0;
	for (int i=0; i< DECODE_NUM_STATES; i++) {
		if (isPlaying()) {
			decodeMp3();
			if (lastError) {
				Serial.print("lastError ");
				Serial.println(lastError);
			}
		}
	}
	
	// we don't know where we are after all the retries, so calculate time from file offset
	// hardcode 44100 for now
	samples_played = (uint64_t)offsetToTimeMs(fposition() - MP3_SD_BUF_SIZE, tocValid ? toc : NULL) * AUDIOCODECS_SAMPLE_RATE / 1000;
	
	if (isPlaying()) {
		pause(false);
		return true;
	} else {
		Serial.print("lastError ");
		Serial.println(lastError);
		return false;
	}
}

uint32_t AudioPlaySdMp3::lengthMillis()
{
	if (vbr_total_frames) {
		// for VBR
		return (uint64_t)vbr_total_frames * mp3FrameInfo.outputSamps / mp3FrameInfo.nChans * 1000 / mp3FrameInfo.samprate;
	} else {
		// for CBR
		uint64_t sizeWithoutID3 = fsize() - size_id3;
		return sizeWithoutID3 * 1000 / (mp3FrameInfo.bitrate / 8);
	}
}

//runs in ISR
__attribute__ ((optimize("O2")))
void AudioPlaySdMp3::update(void)
{
	audio_block_t	*block_left;
	audio_block_t	*block_right;

	//paused or stopped ?
	if (playing != codec_playing) return;

	//chain decoder-interrupt.
	//to give the user-sketch some cpu-time, only chain
	//if the swi is not active currently.
	//In addition, check before if there waits work for it.
	int db = decoding_block;
	if (!NVIC_IS_ACTIVE(IRQ_AUDIOCODEC))
		if (decoded_length[db]==0)
			NVIC_TRIGGER_INTERRUPT(IRQ_AUDIOCODEC);

	//determine the block we're playing from
	int playing_block = 1 - db;
	if (decoded_length[playing_block] <= 0) return;

	// allocate the audio blocks to transmit
	block_left = allocate();
	if (block_left == NULL) return;

	uintptr_t pl = play_pos;

	if (mp3FrameInfo.nChans == 2) {
		// if we're playing stereo, allocate another
		// block for the right channel output
		block_right = allocate();
		if (block_right == NULL) {
			release(block_left);
			return;
		}

		memcpy_frominterleaved(&block_left->data[0], &block_right->data[0], buf[playing_block] + pl);

		pl += AUDIO_BLOCK_SAMPLES * 2;
		transmit(block_left, 0);
		transmit(block_right, 1);
		release(block_right);
		decoded_length[playing_block] -= AUDIO_BLOCK_SAMPLES * 2;

	} else
	{
		// if we're playing mono, no right-side block
		// let's do a (hopefully good optimized) simple memcpy
		memcpy(block_left->data, buf[playing_block] + pl, AUDIO_BLOCK_SAMPLES * sizeof(short));

		pl += AUDIO_BLOCK_SAMPLES;
		transmit(block_left, 0);
		transmit(block_left, 1);
		decoded_length[playing_block] -= AUDIO_BLOCK_SAMPLES;

	}

	samples_played += AUDIO_BLOCK_SAMPLES;

	release(block_left);

	//Switch to the next block if we have no data to play anymore:
	if (decoded_length[playing_block] == 0)
	{
		decoding_block = playing_block;
		play_pos = 0;
	} else
	play_pos = pl;

}

//decoding-interrupt
//__attribute__ ((optimize("O2"))) <- does not work here, bug in g++
void decodeMp3(void)
{
	AudioPlaySdMp3 *o = mp3objptr;
	int db = o->decoding_block;

	if ( o->decoded_length[db] > 0 ) return; //this block is playing, do NOT fill it

	uint32_t cycles = ARM_DWT_CYCCNT;
	int eof = false;

	switch (o->decoding_state) {

	case 0:
		{

			o->sd_left = o->fillReadBuffer(o->sd_buf, o->sd_p, o->sd_left, MP3_SD_BUF_SIZE);
			if (!o->sd_left) { eof = true; goto mp3end; }
			o->sd_p = o->sd_buf;

			uint32_t cycles_rd = (ARM_DWT_CYCCNT - cycles);
			if (cycles_rd > o->decode_cycles_max_read )o-> decode_cycles_max_read = cycles_rd;
			break;
		}

	case 1:
		{
			// find start of next MP3 frame - assume EOF if no sync found
			int offset = MP3FindSyncWord(o->sd_p, o->sd_left);

			if (offset < 0) {
				//Serial.println("No sync"); //no error at end of file
				eof = true;
				goto mp3end;
			}

			o->sd_p += offset;
			o->sd_left -= offset;

			int decode_res = MP3Decode(o->hMP3Decoder, &o->sd_p, (int*)&o->sd_left,o->buf[db], 0);

			AudioPlaySdMp3::lastError = decode_res;
			switch(decode_res)
			{
				case ERR_MP3_NONE:
				{
					MP3GetLastFrameInfo(o->hMP3Decoder, &o->mp3FrameInfo);
					o->decoded_length[db] = o->mp3FrameInfo.outputSamps;
					break;
				}

				case ERR_MP3_MAINDATA_UNDERFLOW:
				{
					break;
				}

				default :
				{
					eof = true;
					break;
				}
			}

			cycles = (ARM_DWT_CYCCNT - cycles);
			if (cycles > o->decode_cycles_max ) o->decode_cycles_max = cycles;
			break;
		}
	}//switch

mp3end:

	o->decoding_state++;
	if (o->decoding_state >= DECODE_NUM_STATES) o->decoding_state = 0;

	if (eof) o->stop();

}
