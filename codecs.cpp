/*
	Helix library Arduino Audio Library MP3/AAC objects

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


#include "codecs.h"

#include "common/assembly.h"

//Skip ID3-Tags at the beginning of the file.
//http://id3.org/id3v2.4.0-structure
size_t skipID3(uint8_t *sd_buf)
{
	if (sd_buf[0]=='I' && sd_buf[1]=='D' && sd_buf[2]=='3' &&
		sd_buf[3]<0xff && sd_buf[4]<0xff &&
		sd_buf[6]<0x80 && sd_buf[7]<0x80 &&
		sd_buf[8]<0x80 && sd_buf[9]<0x80)
	{
		// bytes 6-9:offset of maindata, with bit.7=0:
		int ofs =	((sd_buf[6] & 0x7f) << 21) |
				((sd_buf[7] & 0x7f) << 14) |
				((sd_buf[8] & 0x7f) <<  7) |
				 (sd_buf[9] & 0x7f);
	    return ofs;

	}
	else return 0;
}

bool isReplayGainKey(const char *key){
	if(strncasecmp("REPLAYGAIN_TRACK_GAIN", key, 21) == 0 ||
	   strncasecmp("REPLAYGAIN_ALBUM_GAIN", key, 21) == 0 ||
	   strncasecmp("REPLAYGAIN_TRACK_PEAK", key, 21) == 0 ||
	   strncasecmp("REPLAYGAIN_ALBUM_PEAK", key, 21) == 0){
		return true;
	}
	return false;
}

size_t AudioCodec::fillReadBuffer(uint8_t *sd_buf, uint8_t *data, size_t dataLeft, size_t sd_bufsize)
{//TODO: Sync to 512-Byte blocks, if possible

	memmove(sd_buf, data, dataLeft);

	size_t spaceLeft = sd_bufsize - dataLeft;
	size_t read = dataLeft;
	size_t n;

	if (spaceLeft>0)
	{

		n = fread(sd_buf + dataLeft, spaceLeft);
		dataLeft += n;
		read +=n;

		if(n < spaceLeft)
		{ //Rest mit 0 füllen (EOF)
			memset(sd_buf + dataLeft, 0, sd_bufsize - dataLeft);
		}

	}

	return read;
}
/*
size_t AudioCodec::fillReadBuffer(uint8_t *data, size_t dataLeft)
{//TODO: Sync to 512-Byte blocks, if possible

	memmove(bufptr, data, dataLeft);

	size_t spaceLeft = rdbufsize - dataLeft;
	size_t read = dataLeft;
	size_t n;

	if (spaceLeft>0)
	{

		n = fread(bufptr + dataLeft, spaceLeft);
		dataLeft += n;
		read +=n;

		if(n < spaceLeft)
		{ //Rest mit 0 füllen (EOF)
			memset(bufptr + dataLeft, 0, rdbufsize - dataLeft);
		}

	}

	return read;
}
*/

bool AudioCodec::pause(const bool paused)
{
	if (paused) playing = codec_paused;
	else
	playing = codec_playing;
	return (playing == codec_paused);
}

/** Return the number of bytes currently free in RAM. */
/** from Fat16util.h 2008 by William Greiman**/
int AudioCodec::freeRam(void) {
  extern int  __bss_end;
  extern int* __brkval;
  int free_memory;
  if (reinterpret_cast<int>(__brkval) == 0) {
    // if no heap use from end of bss section
    free_memory = reinterpret_cast<int>(&free_memory)
                  - reinterpret_cast<int>(&__bss_end);
  } else {
    // use from top of stack to heap
    free_memory = reinterpret_cast<int>(&free_memory)
                  - reinterpret_cast<int>(__brkval);
  }
  return free_memory;
}

uint32_t parseID3size(uint8_t *buf){
	uint32_t size = 0;
	for(int i = 0; i < 4; i++){
		size <<= 7;
		size |= *buf & 0x7f;
		buf++;
	}
	return size;
}

uint32_t AudioCodec::parseID3(void)
{
	uint8_t buf[50] = {0};
	// read header
	fseek(0);
	fread(buf, 14);
	if(strncmp("ID3", (char*)buf, 3)){
		return 0;
	}

	uint32_t id3size = parseID3size(&buf[6]) + 10;
	uint32_t frameEnd = id3size;
	if(buf[5] & 0x10){
		// footer present
		id3size += 10;
	}

	// flags
	if(buf[5] & 0xa0){
		// unsynchronization or experimental
		return id3size;
	}

	uint32_t framePos = 10;
	if(buf[5] & 0x40){
		// skip extended header
		Serial.println("skipping extended header");
		framePos += parseID3size(&buf[10]);
	}
	
	while(framePos < frameEnd){
		fseek(framePos);
		fread(buf, 10);
		uint32_t frameSize = parseID3size(&buf[4]);
		framePos += frameSize + 10;
		if(frameSize > 49){
			// too big
			continue;
		}
		if(strncmp("TXXX", (char*)buf, 4) == 0){
			fread(buf, frameSize);
			if(buf[0] != 0){
				continue;
			}
			if(isReplayGainKey((char*)buf + 1)){
				float value = atof((char*)buf + 23);
				setReplayGainValue((char*)buf + 1, value);
			}
		}
	}
	
	return id3size;
}

void AudioCodec::setReplayGainValue(const char *key, float value){
	if(strncasecmp("REPLAYGAIN_TRACK_GAIN", key, 21) == 0){
		replaygain_track_gain_db = value;
	}else if(strncasecmp("REPLAYGAIN_ALBUM_GAIN", key, 21) == 0){
		replaygain_album_gain_db = value;
	}else if(strncasecmp("REPLAYGAIN_TRACK_PEAK", key, 21) == 0){
		replaygain_track_peak = min(1, max(0, value));
	}else if(strncasecmp("REPLAYGAIN_ALBUM_PEAK", key, 21) == 0){
		replaygain_album_peak = min(1, max(0, value));
	}else{
		Serial.print(__FUNCTION__);
		Serial.print(" ignoring key ");
		Serial.println(key);
	}
}
