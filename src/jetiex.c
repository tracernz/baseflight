/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 *
 * Driver for Jeti EX Bus receiver using USART2
 * Written by: tracernz
 *
 * Based on Jeti EX Bus protocol v1.21
 * http://www.jetimodel.com/en/show-file/642/
 */

#include "board.h"
#include "mw.h"

#define JETI_BAUD_SLOW 125000
#define JETI_BAUD_FAST 250000
#define JETI_RCFRAME_BEGIN 0x3E
#define JETI_PACKET_RC 0x31
#define JETI_FRAME_MINLEN 7
#define JETI_FRAME_MAXLEN 70
#define JETI_NUM_CHANNELS 16


// external vars (ugh)
extern int16_t failsafeCnt;

static USART_TypeDef *jetiUart = USART2;
static bool jetiFrameBegun = false;
static uint8_t jetiFramePos = 0;
static uint8_t jetiFrame[JETI_FRAME_MAXLEN];
static bool jetiRcFrameComplete = false;
static bool jetiBaudValid = false;
static uint16_t jetiJunkChars = 0;


static void jetiDataReceive(uint16_t c);
static uint16_t jetiReadRawRC(uint8_t chan);


// checksum code provided by Jeti EX-Bus Protocol v1.21 docs
static uint16_t crc_ccitt_update(uint16_t crc, uint8_t data)
{
	uint16_t ret_val;

	data ^= (uint8_t)crc & (uint8_t)0xFF;
	data ^= data << 4;
	ret_val = ((((uint16_t)data << 8) | ((crc & 0xFF00) >> 8))
			^ (uint8_t)(data >> 4) ^ ((uint16_t)data << 3));

	return ret_val;
}

static uint16_t getcrc16z(uint8_t *p, uint16_t len)
{
	uint16_t crc16_data = 0;

	while(len--) {
		crc16_data = crc_ccitt_update(crc16_data, p[0]);
		p++;
	}

	return crc16_data;
}


void jetiUartOpen(uint32_t baud, portMode_t mode) {
	core.rcvrport = uartOpen(jetiUart, jetiDataReceive, baud, mode);

	uartSetHalfDuplex(core.rcvrport, true);
}

void jetiInit(rcReadRawDataPtr *callback)
{
	jetiUartOpen(JETI_BAUD_SLOW, MODE_RX);

    core.numRCChannels = JETI_NUM_CHANNELS;

    if (callback)
        *callback = jetiReadRawRC;
}

bool jetiFrameComplete(void) {
	if(jetiRcFrameComplete) {
		failsafeCnt = 0;
		jetiRcFrameComplete = false;
		return true;
	}
	else if(!jetiBaudValid && (jetiJunkChars > 1000)) {
		// If no valid channel data is received try switching baud
		jetiJunkChars = 0;
		if(core.rcvrport->baudRate == JETI_BAUD_SLOW)
			jetiUartOpen(JETI_BAUD_FAST, MODE_RX);
		else
			jetiUartOpen(JETI_BAUD_SLOW, MODE_RX);
	}
	return false;
}

// UART receive ISR callback
static void jetiDataReceive(uint16_t c)
{
	static uint8_t jetiFrameLen;

	if(!jetiFrameBegun) {
		// check for frame begin byte
		if(c == JETI_RCFRAME_BEGIN) {
			jetiFrameBegun = true;
			jetiRcFrameComplete = false;
			jetiFramePos = 0;
			// actual length isn't known until byte 3 arrives
			jetiFrameLen = JETI_FRAME_MINLEN;
		} else {
			jetiJunkChars++;
			return;
		}

	} else if(jetiFramePos == 2) {
		// capture byte 3, aka the frame length
		jetiFrameLen = (uint8_t)c;
		if(jetiFrameLen > JETI_FRAME_MAXLEN) // make sure frame isn't too long for buffer
			jetiFrameBegun = false;
	} else if((jetiFramePos == 4) && ((uint8_t)c != JETI_PACKET_RC)) {
		// ignore telemetry and jetibox packets
		jetiFrameBegun = false;
	}

	jetiFrame[jetiFramePos++] = (uint8_t)c;

	if(jetiFramePos == jetiFrameLen) {
		// end of frame, check crc16-ccit
		if(getcrc16z(jetiFrame, jetiFrameLen) == 0) {
			jetiBaudValid = true;
			jetiRcFrameComplete = true;
		} else
			jetiJunkChars += jetiFrameLen;

		// allow to look for a new frame since this one is done
		jetiFrameBegun = false;
	}
}

static uint16_t jetiReadRawRC(uint8_t chan)
{
	uint16_t data;
	uint8_t b;

	chan = mcfg.rcmap[chan];

	if(chan < JETI_NUM_CHANNELS) {
		// rc channel data starts at byte 7
		// each channel is 2 bytes in LSB, MSB order
		b = 6 + (chan * 2);
		data = (((uint16_t)jetiFrame[b+1]) << 8) | ((uint16_t)jetiFrame[b]);
		// 1 = 1/8 us
		data = (data + 4) / 8;
	} else {
		data = mcfg.midrc;
	}

	return data;
}
