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

 // Total RAM Usage: 31 KB

#include "play_sd_aac.h"
#include "common/assembly.h"

#define AAC_SD_BUF_SIZE	3072 								//Enough space for a complete stereo frame
//#define AAC_SD_BUF_SIZE	2560 								//Enough space for a complete stereo frame
//#define AAC_SD_BUF_SIZE	(1536 + 768)
#define AAC_BUF_SIZE	(AAC_MAX_NCHANS * AAC_MAX_NSAMPS)	//AAC output buffer

#define DECODE_NUM_STATES 2									//How many steps in decode() ?
#define CODEC_DEBUG

static AudioPlaySdAac 	*aacobjptr;
void decodeAac(void);

void AudioPlaySdAac::stop(void)
{
	NVIC_DISABLE_IRQ(IRQ_AUDIOCODEC);

	playing = codec_stopped;
	if (buf[1]) {free(buf[1]);buf[1] = NULL;}
	if (buf[0]) {free(buf[0]);buf[0] = NULL;}
	freeBuffer();
	if (hAACDecoder) {AACFreeDecoder(hAACDecoder);hAACDecoder=NULL;};
}

uint32_t AudioPlaySdAac::lengthMillis(void)
{
	if (duration)
		return duration;
	else
		return AudioCodec::lengthMillis();
}

//read big endian 16-Bit from fileposition(position)
uint16_t AudioPlaySdAac::fread16(size_t position)
{
	uint16_t tmp16;
	fseek(position);
	fread((uint8_t *) &tmp16, sizeof(tmp16));
	return REV16(tmp16);
}

//read big endian 32-Bit from fileposition(position)
uint32_t AudioPlaySdAac::fread32(size_t position)
{
	uint32_t tmp32;
	fseek(position);
	fread((uint8_t *) &tmp32, sizeof(tmp32));
	return REV32(tmp32);
}

_ATOM AudioPlaySdAac::findMp4Atom(const char *atom, const uint32_t posi, const bool loop = true)
{

	bool r;
	_ATOM ret;
	_ATOMINFO atomInfo;

	ret.position = posi;
	do
	{
		r = fseek(ret.position);
		fread((uint8_t *) &atomInfo, sizeof(atomInfo));
		ret.size = REV32(atomInfo.size);
		if (ret.size == 1) {
			uint32_t tmp;
			// read top 32 bits
			fread((uint8_t *)&tmp, sizeof(tmp));
			if(tmp != 0){
				// sizes > 4GB not supported
				break;
			}
			fread((uint8_t *)&tmp, sizeof(tmp));
			ret.size = REV32(tmp);
		}
		//ret.size = atomInfo.size;
		if (strncmp(atom, atomInfo.name, 4)==0){
			return ret;
		}
		ret.position += ret.size ;
	} while (loop && r);
	ret.position = 0;
	ret.size = 0;
	return ret;

}

// Currently, only replay gain tags are parsed
void AudioPlaySdAac::parseMp4Tags(uint32_t moovPosition)
{
	_ATOM udta = findMp4Atom("udta", moovPosition + 8);
	if(!udta.size){
		Serial.println("no udta");
		return;
	}
	_ATOM meta = findMp4Atom("meta", udta.position + 8);
	if(!meta.size){
		Serial.println("no meta");
		return;
	}
	_ATOM ilst = findMp4Atom("ilst", meta.position + 12);
	if(!ilst.size){
		Serial.println("no ilst");
		return;
	}
	uint32_t ilstEnd = ilst.position + ilst.size;
	Serial.println("found ilst atom");
	uint32_t nextTagPos = ilst.position + 8;
	while(nextTagPos < ilstEnd){
		if(!fseek(nextTagPos)){
			return;
		}
		_ATOMINFO atomInfo;
		fread((uint8_t *) &atomInfo, sizeof(atomInfo));
		atomInfo.size = REV32(atomInfo.size);
		
		if(strncmp(atomInfo.name, "----", 4) == 0){
			_ATOM name = findMp4Atom("name", nextTagPos + 8);
			if(!name.size){
				goto nexttag;
			}
			char strbuf[25];
			if(!fseek(name.position + 12)){
				return;
			}
			fread((uint8_t*)&strbuf, 24);
			strbuf[24] = 0;
			bool isPeak, isAlbum;
			if(strcasecmp("REPLAYGAIN_TRACK_GAIN", strbuf) == 0){
				isPeak = false; isAlbum = false;
			}else if(strcasecmp("REPLAYGAIN_ALBUM_GAIN", strbuf) == 0){
				isPeak = false; isAlbum = false;
			}else if(strcasecmp("REPLAYGAIN_TRACK_PEAK", strbuf) == 0){
				isPeak = true; isAlbum = false;
			}else if(strcasecmp("REPLAYGAIN_ALBUM_PEAK", strbuf) == 0){
				isPeak = true; isAlbum = true;
			}else{
				goto nexttag;
			}
			
			_ATOM data = findMp4Atom("data", nextTagPos + 8);
			float value;
			if(data.size && data.size < 40 && data.position < ilstEnd){
				if(!fseek(data.position + 16)){
					return;
				}
				fread((uint8_t*)&strbuf, data.size - 16);
				strbuf[data.size - 16] = 0;
				value = atof(strbuf);
			}else{
				Serial.print(strbuf);
				Serial.println(" data too big");
				goto nexttag;
			}
			
			if(isPeak){
				value = min(1, max(0, value));
				if(isAlbum){
					replaygain_album_peak = value;
				}else{
					replaygain_track_peak = value;
				}
			}else{
				if(isAlbum){
					replaygain_album_gain_db = value;
				}else{
					replaygain_track_gain_db = value;
				}
			}
		}
nexttag:
		nextTagPos += atomInfo.size;
	}
}

int AudioPlaySdAac::setupMp4(void)
{
	_ATOM ftyp = findMp4Atom("ftyp",0, false);
	if (!ftyp.size)
		return -1; //no mp4/m4a file

	//go through the boxes to find the interesting atoms:
	_ATOM moov = findMp4Atom("moov", 0);
	if (!moov.size) {
		Serial.println("no moov");
		return 1;
	}

	//read mvhd and return false if there is more than 1 track (video file)
	_ATOM mvhd = findMp4Atom("mvhd", moov.position + 8);
	if (!mvhd.size) {
		Serial.println("no mvhd");
		return 1;
	}

	uint8_t mvhdVersion;
	fseek(mvhd.position);
	fread(&mvhdVersion, 1);
	uint32_t next_track_id = fread32(mvhd.position + (mvhdVersion == 0 ? 104 : 116));
	if (next_track_id > 2) {
		return 1;
	}

	uint32_t trak = findMp4Atom("trak", moov.position + 8).position;
	uint32_t mdia = findMp4Atom("mdia", trak + 8).position;

	//determine duration:
	uint32_t mdhd = findMp4Atom("mdhd", mdia + 8).position;
	uint32_t timescale = fread32(mdhd + 8 + 0x0c);
	duration = 1000.0 * ((float)fread32(mdhd + 8 + 0x10) / (float)timescale);

	//MP4-data has no aac-frames, so we have to set the parameters by hand.
	uint32_t minf = findMp4Atom("minf", mdia + 8).position;
	uint32_t stbl = findMp4Atom("stbl", minf + 8).position;
	//stsd sample description box: - infos to parametrize the decoder
	_ATOM stsd = findMp4Atom("stsd", stbl + 8);
	if (!stsd.size)
		return 1; //something is not ok

	_ATOM mp4a = findMp4Atom("mp4a", stsd.position + 16);
	if (!mp4a.size) {
		// not audio file?
		Serial.println("no mp4a");
		return 1;
	}

	uint16_t channels = fread16(mp4a.position + 0x18);
	//uint16_t channels = 1;
	//uint16_t bits		= fread16(stsd.position + 8 + 0x22); //not used
	samplerate = fread32(mp4a.position + 0x1e);
	if (!samplerate) {
		Serial.println("no samplerate");
		return 1;
	}

	setupDecoder(channels, samplerate, AAC_PROFILE_LC);

	// parse stsc for chunk sizes seeking
	stscPosition = findMp4Atom("stsc", stbl + 8).position;
	uint32_t stscEntries = fread32(stscPosition + 12);
	Serial.print("stsc has ");
	Serial.print(stscEntries);
	Serial.println(" entries");
	for(uint32_t i=0; i<stscEntries; i++) {
		uint32_t first_chunk = fread32(stscPosition + 16 + i * 12);
		uint32_t samples_per_chunk = fread32(stscPosition + 16 + i * 12 + 4);
		// don't read the sample_description_index
		Serial.print("  first_chunk=");
		Serial.print(first_chunk);
		Serial.print("  samples_per_chunk=");
		Serial.println(samples_per_chunk);
	}

	// parse stts for "sample" (actually AAC block) size for seeking
	sttsPosition = findMp4Atom("stts", stbl + 8).position;
	uint32_t sttsEntries = fread32(sttsPosition + 12);
	for(uint32_t i=0; i<sttsEntries; i++) {
		uint32_t sample_count = fread32(sttsPosition + 16 + i * 8);
		uint32_t sample_delta = fread32(sttsPosition + 16 + i * 8 + 4);
		// don't read the sample_description_index
		Serial.print("  sample_count=");
		Serial.print(sample_count);
		Serial.print("  sample_delta=");
		Serial.println(sample_delta);
	}

	//stco - chunk offset atom:
	stcoPosition = findMp4Atom("stco", stbl + 8).position;
	//stsz - sample (aac block) size atom
	stszPosition = findMp4Atom("stsz", stbl + 8).position;

	//number of chunks:
	uint32_t nChunks = fread32(stcoPosition + 8 + 0x04);
	//first entry from chunk table:
	firstChunk = fread32(stcoPosition + 8 + 0x08);
	//last entry from chunk table:
	lastChunk = fread32(stcoPosition + 8 + 0x04 + nChunks * 4);

	if (nChunks == 1) {
		_ATOM mdat =  findMp4Atom("mdat", 0);
		lastChunk = mdat.size;
	}
	
	parseMp4Tags(moov.position);

#if 0
	for(uint32_t i=0; i<nChunks; i++) {
		uint32_t chunk_offset = fread32(stcoPosition + 16 + i * 4);
		Serial.print("chunk ");
		Serial.print(i);
		Serial.print(" offset ");
		Serial.println(chunk_offset);
	}

	Serial.print("mdhd duration=");
	Serial.print(duration);
	Serial.print(" ms, stsd: chan=");
	Serial.print(channels);
	Serial.print(" samplerate=");
	Serial.print(samplerate);
	Serial.print(" nChunks=");
	Serial.print(nChunks);
	Serial.print(" firstChunk=");
	Serial.println(firstChunk, HEX);
	Serial.print(" lastChunk=");
	Serial.println(lastChunk, HEX);
#endif

	return 0;
}

void AudioPlaySdAac::setupDecoder(int channels, int samplerate, int profile)
{
	memset(&aacFrameInfo, 0, sizeof(AACFrameInfo));
	aacFrameInfo.nChans = channels;
	//aacFrameInfo.bitsPerSample = bits; not used
	aacFrameInfo.sampRateCore = samplerate;
	aacFrameInfo.profile = AAC_PROFILE_LC;
	AACSetRawBlockParams(hAACDecoder, 0, &aacFrameInfo);
}

int AudioPlaySdAac::play(void){
	lastError = ERR_CODEC_NONE;
	initVars();
	sd_buf = allocBuffer(AAC_SD_BUF_SIZE);
	if (!sd_buf) return ERR_CODEC_OUT_OF_MEMORY;

	aacobjptr = this;

	buf[0] = (short *) malloc(AAC_BUF_SIZE * sizeof(int16_t));
	buf[1] = (short *) malloc(AAC_BUF_SIZE * sizeof(int16_t));

	hAACDecoder = AACInitDecoder();

	if (!buf[0] || !buf[1] || !hAACDecoder)
	{
		lastError = ERR_CODEC_OUT_OF_MEMORY;
		stop();
		return lastError;
	}

	isRAW = true;
	duration = 0;
	sd_left = 0;

	sd_p = sd_buf;

	int mp4Error = setupMp4();
	if (mp4Error == 0) {
		fseek(firstChunk);
		sd_left = 0;
		isRAW = false;
		//Serial.print("mp4");
	}
	else if (mp4Error < 0) { //NO MP4. Do we have an ID3TAG ?

		fseek(0);
		//Read-ahead 10 Bytes to detect ID3
		sd_left = fread(sd_buf, 10);
		//Skip ID3, if existent
		uint32_t skip = skipID3(sd_buf);
		if (skip) {
			size_id3 = skip;
			fseek(skip);
			sd_left = 0;
			//Serial.print("ID3");
		} else size_id3 = 0;
	}
	else { //corrupted MP4
		stop();
		return ERR_CODEC_FORMAT;
	}

	//Fill buffer from the beginning with fresh data
	sd_left = fillReadBuffer(sd_buf, sd_buf, sd_left, AAC_SD_BUF_SIZE);

	if (!sd_left) {
		lastError = ERR_CODEC_FILE_NOT_FOUND;
		stop();
		return lastError;
	}

	_VectorsRam[IRQ_AUDIOCODEC + 16] = &decodeAac;
	initSwi();

	decoded_length[0] = 0;
	decoded_length[1] = 0;
	decoding_state = 0;
	decoding_block = 0;

	for (int i=0; i< DECODE_NUM_STATES; i++) decodeAac();

	if(lastError){
		stop();
		return lastError;
	}

	samplerate = aacFrameInfo.sampRateOut;

	if(aacFrameInfo.nChans > 2) {
		//Serial.println("incompatible AAC file.");
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

bool AudioPlaySdAac::seek(uint32_t timesec) {
	if (isRAW) {
		return false; // ADTS seeking not implemented
	}

	pause(true);

	// parse stts for "sample" (actually AAC block) size for seeking
	uint32_t sttsEntries = fread32(sttsPosition + 12);
	if (sttsEntries < 1) {
		Serial.print("seek fail: stts has no entries");
		return false;
	} else if (sttsEntries > 1) {
		// files produced by ffmpeg have an extra stts entry for the last frame with less than 1024 samples
		Serial.print("seek: stts has too many entries (");
		Serial.print(sttsEntries);
		Serial.println("). ignoring extra entries");
	}
	uint32_t sample_count = fread32(sttsPosition + 16);
	uint32_t sample_delta = fread32(sttsPosition + 16 + 4);
	uint32_t aacBlockNum = timesec * samplerate / sample_delta;
	
	if (aacBlockNum >= sample_count) {
		Serial.println("seek fail: trying to seek past end of file");
		return false;
	}
	
	Serial.print("going to block ");
	Serial.println(aacBlockNum);
	
	// parse stsc for chunk sizes seeking
	uint32_t stscEntries = fread32(stscPosition + 12);
	Serial.print("stsc has ");
	Serial.print(stscEntries);
	Serial.println(" entries");
	if (stscEntries < 1) {
		Serial.println("seek fail: stsc has no entries");
	}

	uint32_t cumulativeBlocks = 0;
	uint32_t last_first_chunk = 1, last_samples_per_chunk = 1; // initialize to avoid compiler warning
	for(uint32_t i=0; i<stscEntries; i++) {
		uint32_t first_chunk = fread32(stscPosition + 16 + i * 12);
		uint32_t samples_per_chunk = fread32(stscPosition + 16 + i * 12 + 4);
		// don't read the sample_description_index
		Serial.print("  first_chunk=");
		Serial.print(first_chunk);
		Serial.print("  samples_per_chunk=");
		Serial.println(samples_per_chunk);
		if (i > 0) {
			uint32_t blocksInLastChunk = (first_chunk - last_first_chunk) * last_samples_per_chunk;
			if (cumulativeBlocks + blocksInLastChunk > aacBlockNum) {
				// last chunk is the chunk
				break;
			}
			cumulativeBlocks += blocksInLastChunk;
		}
		last_first_chunk = first_chunk;
		last_samples_per_chunk = samples_per_chunk;
	}
	Serial.print("cumulativeBlocks ");Serial.println(cumulativeBlocks);
	// chunks in mp4 are 1-indexed.
	uint32_t theChunk = (aacBlockNum - cumulativeBlocks) / last_samples_per_chunk + last_first_chunk;
	uint32_t chunkFirstBlock = aacBlockNum - (aacBlockNum - cumulativeBlocks) % last_samples_per_chunk;
	uint32_t chunk_offset = fread32(stcoPosition + 16 + (theChunk-1) * 4);

	uint32_t block_offset = chunk_offset;
	uint32_t sample_size = fread32(stszPosition + 12);
	if (sample_size) {
		block_offset += sample_size * (aacBlockNum - chunkFirstBlock);
	} else  {
		uint32_t stsz_sample_count = fread32(stszPosition + 16);
		if (stsz_sample_count < aacBlockNum) {
			Serial.print("seek fail: stsz sample_count is ");
			Serial.print(stsz_sample_count);
			Serial.print(", need ");
			Serial.println(aacBlockNum);
			return false;
		}
		for (uint32_t i = chunkFirstBlock; i < aacBlockNum; i++) {
			uint32_t blockSize = fread32(stszPosition + 20 + i * 4);
			block_offset += blockSize;
		}
	}
	samples_played = sample_delta * aacBlockNum;
	
	Serial.print("go to chunk ");Serial.println(theChunk);
	Serial.print("chunk_offset ");Serial.println(chunk_offset);
	Serial.print("skip ");Serial.print(aacBlockNum - chunkFirstBlock); Serial.println(" blocks");
	Serial.print("block_offset ");Serial.println(block_offset);
	
	if (!fseek(block_offset)) {
		Serial.println("seek fail: fseek failed");
		return false;
	}
	// clear AAC stream buffer and decoding_block data
	sd_p = sd_buf;
	sd_left = fillReadBuffer(sd_buf, sd_buf, 0, AAC_SD_BUF_SIZE);
	if (!sd_left) {
		Serial.println("fillReadBuffer failed");
		return false;
	}
	decoding_state = 1; // we refilled the buffer
	
	decoded_length[decoding_block] = 0;
	for (int i=0; i< DECODE_NUM_STATES; i++) decodeAac();
	decoding_block = 1 - decoding_block;
	decoded_length[decoding_block] = 0;
	for (int i=0; i< DECODE_NUM_STATES; i++) decodeAac();
	
	pause(false);
	return true;
}

//runs in ISR
void AudioPlaySdAac::update(void)
{
	audio_block_t	*block_left;
	audio_block_t	*block_right;

	//paused or stopped ?
	if (playing  != codec_playing) return;

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
	if (decoded_length[playing_block] <= 0)
		{
			stop();
			return;
		}

	// allocate the audio blocks to transmit
	block_left = allocate();
	if (block_left == NULL) return;

	int pl = play_pos;

	if (aacFrameInfo.nChans == 2) {
		// if we're playing stereo, allocate another
		// block for the right channel output
		block_right = allocate();
		if (block_right == NULL) {
			release(block_left);
			return;
		}

		memcpy_frominterleaved(block_left->data, block_right->data, buf[playing_block] + pl);

		pl += AUDIO_BLOCK_SAMPLES * 2 ;
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
	if ((decoded_length[playing_block] == 0) )
	{
		decoding_block = playing_block;
		play_pos = 0;
	} else
	play_pos = pl;

}

//decoding-interrupt
void decodeAac(void)
{
	AudioPlaySdAac *o = aacobjptr;
	int db = o->decoding_block;
	if (o->decoded_length[db]) return; //this block is playing, do NOT fill it

	uint32_t cycles = ARM_DWT_CYCCNT;
	int eof = false;

	switch (o->decoding_state) {

	case 0:
		{

			o->sd_left = o->fillReadBuffer(o->sd_buf, o->sd_p, o->sd_left, AAC_SD_BUF_SIZE);
			if (!o->sd_left) { eof = true; goto aacend; }
			o->sd_p = o->sd_buf;

			uint32_t cycles_rd = ARM_DWT_CYCCNT - cycles;
			if (cycles_rd > o->decode_cycles_max_read ) o->decode_cycles_max_read = cycles_rd;
			break;
		}

	case 1:
		{
			if (o->isRAW) {

				// find start of next AAC frame - assume EOF if no sync found
				int offset = AACFindSyncWord(o->sd_p, o->sd_left);

				if (offset < 0) {
					eof = true;
					goto aacend;
				}

				o->sd_p += offset;
				o->sd_left -= offset;

			}

			int decode_res = AACDecode(o->hAACDecoder, &o->sd_p, (int*)&o->sd_left, o->buf[db]);

			if (!decode_res) {
				AACGetLastFrameInfo(o->hAACDecoder, &o->aacFrameInfo);
				o->decoded_length[db] = o->aacFrameInfo.outputSamps;
			} else {
				AudioPlaySdAac::lastError = decode_res;
				eof = true;
				//goto aacend;
			}

			//if (!o->isRAW && (o->fposition() > o->lastChunk)) eof = true;

			cycles = ARM_DWT_CYCCNT - cycles;
			if (cycles > o->decode_cycles_max ) o->decode_cycles_max = cycles;
		}
	} //switch

aacend:

	o->decoding_state++;
	if (o->decoding_state >= DECODE_NUM_STATES) o->decoding_state = 0;

	if (eof) o->stop();

}
