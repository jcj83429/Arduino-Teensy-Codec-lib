README for Xing's Variable Bitrate MP3 Playback SDK
December 1998
------------------------------------------------------------------
(c) Copyright Xing Technology Corporation, 1998 - 1999

---------------------------------------
Playback of Variable Bit Rate MP3 Files
---------------------------------------
Most MP3 players are able to successfully playback
Variable Bit Rate (VBR) MP3 files (such as those
created by Xing's AudioCatalyst 1.5 product), but
due to the nature of VBR MP3 files the players
encounter the following issues:
- they are unable to determine the duration of the file
- they are unable to accurately seek within the file

Xing has provided the following solution to these issues:
- Xing's MP3 Encoder creates at the start of a VBR MP3 file
  a null audio frame in which a "Xing Header" is stored.
  This header contains:
  - indication of the total number of audio frames
    in the bitstream
  - indication of the total number of bytes in the bitstream
  - a 100-element Table of Contents indicating seek points
    in the bitstream.

Application developers can eliminate the VBR playback
issues by including in their MP3 playback products support
for this Xing Header.

-------------------------------------
Using the DXHEAD.C and DXHEAD.H Files
-------------------------------------
The dxhead.c and dxhead.h files provide C-based functions
for extracting and using the Xing Header.

To use these functions, include dxhead.h in the file(s)
from which you want to call its functions and compile
dxhead.c into your executable(s).

For further information, please read the comments in the
dxhead.c and dxhead.h files.

The SampleVBR.mp3 file included in this ZIP archive is
a sample VBR MP3 file containing the Xing Header.

-----------------
Technical Support
-----------------
You can obtain Technical Support for the Variable Bitrate
MP3 Playback SDK by accessing the Support area of our Web
site at http://support.xingtech.com/. 

From the Support page, you can search the FAQ knowledge 
base, submit problem reports and feature requests, download 
updates to software, and read additional product documentation.