#include <string.h>

#include "conspi.h"
#include "global.h"

extern "C" void outDebugString(const char *format, ...);

#define SWAP_ENDIAN false

CConSpi::CConSpi()
{
    zeroAllVars();
}

CConSpi::~CConSpi()
{

}

void CConSpi::zeroAllVars(void)
{
    prevAtnWord.got = false;
    remainingPacketLength    = -1;
}

void CConSpi::applyNoTxRxLimis(void)
{
    setRemainingTxRxLen(NO_REMAINING_LENGTH, NO_REMAINING_LENGTH);  // set no remaining length
}

void CConSpi::receiveAndApplyTxRxLimits(void)
{
    BYTE inBuff[4], outBuff[4];
    memset(outBuff, 0, 4);
    memset(inBuff, 0, 4);

    WORD *pwIn = (WORD *) inBuff;

    // get TX LEN and RX LEN
    txRx(4, outBuff, inBuff);

    WORD txLen = swapWord(pwIn[0]);
    WORD rxLen = swapWord(pwIn[1]);

    outDebugString("TX/RX limits: TX %d WORDs, RX %d WORDs", txLen, rxLen);

    if(txLen > 1024 || rxLen > 1024) {
        outDebugString("TX/RX limits above are probably wrong! Fix this!");

        if(txLen > 1024) {
            txLen = 1024;
        }

        if(rxLen > 1024) {
            rxLen = 1024;
        }
    }

    setRemainingTxRxLen(txLen, rxLen);
}

WORD CConSpi::swapWord(WORD val)
{
    WORD tmp = 0;

    tmp  = val << 8;
    tmp |= val >> 8;

    return tmp;
}

void CConSpi::setRemainingTxRxLen(WORD txLen, WORD rxLen)
{
//    outDebugString("CConSpi::setRemainingTxRxLen - TX %d, RX %d", txLen, rxLen);

    if(txLen == NO_REMAINING_LENGTH && rxLen == NO_REMAINING_LENGTH) {    // if setting NO_REMAINING_LENGTH
        if(remainingPacketLength != 0) {
            outDebugString("CConSpi - didn't TX/RX enough data, padding with %d zeros! Fix this!", remainingPacketLength);
            memset(paddingBuffer, 0, PADDINGBUFFER_SIZE);
            txRx(remainingPacketLength, paddingBuffer, paddingBuffer, true);
        }
    } else {                    // if setting real limit
        txLen *= 2;             // convert WORD count to BYTE count
        rxLen *= 2;

        if(txLen >= 8) {        // if we should TX more than 8 bytes, subtract 8 (header length)
            txLen -= 8;
        } else {                // shouldn't TX 8 or more bytes? Then don't TX anymore.
            txLen = 0;
        }

        if(rxLen >= 8) {        // if we should RX more than 8 bytes, subtract 8 (header length)
            rxLen -= 8;
        } else {                // shouldn't RX 8 or more bytes? Then don't RX anymore.
            rxLen = 0;
        }
    }

    // The SPI bus is TX and RX at the same time, so we will TX/RX until both are used up.
    // So use the greater count as the limit.
    if(txLen >= rxLen) {
        remainingPacketLength = txLen;
    } else {
        remainingPacketLength = rxLen;
    }
}

WORD CConSpi::getRemainingLength(void)
{
    return remainingPacketLength;
}

void CConSpi::txRx(int count, BYTE *sendBuffer, BYTE *receiveBufer, bool addLastToAtn)
{
    if(SWAP_ENDIAN) {       // swap endian on sending if required
        BYTE tmp;

        for(int i=0; i<count; i += 2) {
            tmp             = sendBuffer[i+1];
            sendBuffer[i+1] = sendBuffer[i];
            sendBuffer[i]   = tmp;
        }
    }

    if(count == TXRX_COUNT_REST) {          // if should TX/RX the rest, use the remaining length
        count = remainingPacketLength;
    }

    if(remainingPacketLength != NO_REMAINING_LENGTH) {
        if(count > remainingPacketLength) {
            outDebugString("CConSpi::txRx - trying to TX/RX %d more bytes then allowed! Fix this!", (count - remainingPacketLength));

            count = remainingPacketLength;
        }
    }

//    outDebugString("CConSpi::txRx - count: %d", count);

    write   (count, sendBuffer);
    read    (count, receiveBufer);

    if(remainingPacketLength != NO_REMAINING_LENGTH) {
        remainingPacketLength -= count;             // mark that we've send this much data
    }

    // add the last WORD as possible to check for the new ATN
    if(addLastToAtn) {
        setAtnWord(&receiveBufer[count - 2]);
    }
}

void CConSpi::write(int count, BYTE *buffer)
{
    DWORD bytesWrote;
    int remaining = count;
    int wroteTotal = 0;

	// PORT TODO -- replace this
//    DWORD start = GetTickCount();
    DWORD start = 0;

    while(remaining > 0) {
		// PORT TODO -- replace this
		/*
        if((GetTickCount() - start) > 3000) {         // timeout?
            outDebugString("Timeout on USB write!");
            outDebugString("remaining: %d", remaining);
            return;
        }
		*/

//        (*pFT_Write)(ftHandle, &buffer[wroteTotal], remaining, &bytesWrote);

        remaining  -= bytesWrote;
        wroteTotal += bytesWrote;
    }
}

void CConSpi::read (int count, BYTE *buffer)
{
    DWORD bytesRead;
    int remaining = count;
    int readTotal = 0;

	// PORT TODO -- replace this
//    DWORD start = GetTickCount();
	DWORD start = 0;

    while(remaining > 0) {
	// PORT TODO -- replace this
/*	
        if((GetTickCount() - start) > 3000) {         // timeout?
            outDebugString("Timeout on USB read!");
            return;
        }
*/

//        (*pFT_Read)(ftHandle, &buffer[readTotal], remaining, &bytesRead);

        remaining -= bytesRead;
        readTotal += bytesRead;
    }
}

void CConSpi::getAtnWord(BYTE *bfr)
{
    if(prevAtnWord.got) {                   // got some previous ATN word? use it
        bfr[0] = prevAtnWord.bytes[0];
        bfr[1] = prevAtnWord.bytes[1];
        prevAtnWord.got = false;

        return;
    }

    // no previous ATN word? read it!
    BYTE outBuff[2];
    memset(outBuff, 0, 2);
    memset(bfr, 0, 2);

    txRx(2, outBuff, bfr, false);
}

void CConSpi::setAtnWord(BYTE *bfr)
{
    prevAtnWord.bytes[0] = bfr[0];
    prevAtnWord.bytes[1] = bfr[1];
    prevAtnWord.got = true;
}

