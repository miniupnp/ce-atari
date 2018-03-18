// vim: tabstop=4 softtabstop=4 shiftwidth=4 expandtab
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <stdio.h>
#include <errno.h>

#include "gpio.h"
#include "conspi.h"
#include "global.h"
#include "debug.h"
#include "acsidatatrans.h"
#include "utils.h"

#define SWAP_ENDIAN false
//#define DEBUG_SPI_COMMUNICATION

#define GPIOSYSFSPATH "/sys/class/gpio"

CConSpi::CConSpi():
fdAtnHans(-1), fdAtnFranz(-1), firstTimeReadingAtn(true)
{
    char tmppath[256];
	paddingBuffer = new BYTE[PADDINGBUFFER_SIZE];
    // https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
    FILE * f = fopen(GPIOSYSFSPATH "/export", "w");
    if (f != NULL) {
        fprintf(f, "%d\n", SPI_ATN_HANS);
        fclose(f);
        f = fopen(GPIOSYSFSPATH "/export", "w");
        fprintf(f, "%d\n", SPI_ATN_FRANZ);
        fclose(f);
        snprintf(tmppath, sizeof(tmppath), "%s/gpio%d/edge", GPIOSYSFSPATH, SPI_ATN_HANS);
        f = fopen(tmppath, "w");
        fprintf(f, "rising\n");
        fclose(f);
        snprintf(tmppath, sizeof(tmppath), "%s/gpio%d/edge", GPIOSYSFSPATH, SPI_ATN_FRANZ);
        f = fopen(tmppath, "w");
        fprintf(f, "rising\n");
        fclose(f);
        snprintf(tmppath, sizeof(tmppath), "%s/gpio%d/value", GPIOSYSFSPATH, SPI_ATN_HANS);
        fdAtnHans = open(tmppath, O_RDONLY);
        snprintf(tmppath, sizeof(tmppath), "%s/gpio%d/value", GPIOSYSFSPATH, SPI_ATN_FRANZ);
        fdAtnFranz = open(tmppath, O_RDONLY);
    } else {
        Debug::out(LOG_INFO, "cannot open %s", GPIOSYSFSPATH "/export");
    }
}

CConSpi::~CConSpi()
{
	delete [] paddingBuffer;
    if (fdAtnHans > 0)
        close(fdAtnHans);
    if (fdAtnFranz > 0)
        close(fdAtnFranz);
}

void CConSpi::applyNoTxRxLimis(int whichSpiCs)
{
    setRemainingTxRxLen(whichSpiCs, NO_REMAINING_LENGTH, NO_REMAINING_LENGTH);  // set no remaining length
}

bool CConSpi::waitForATNs(int timeoutMs, BYTE & hansAtn, BYTE & franzAtn)
{
    struct pollfd pfd[2];
    char buffer[8];
    BYTE outBuf[8];
    BYTE inBuf[8];
    ssize_t len;

    memset(inBuf, 0, 8);

    hansAtn = ATN_NONE;
    franzAtn = ATN_NONE;

    if (firstTimeReadingAtn || fdAtnHans < 0) {
        if(waitForATN(SPI_CS_HANS, (BYTE) ATN_ANY, 0, inBuf))
            hansAtn = inBuf[3];
        memset(inBuf, 0, 8);
        if(waitForATN(SPI_CS_FRANZ, (BYTE) ATN_ANY, 0, inBuf))
            franzAtn = inBuf[3];
        firstTimeReadingAtn = false;
        return true;
    }

    pfd[0].fd = fdAtnHans;
    pfd[0].events = POLLPRI|POLLERR;
    pfd[1].fd = fdAtnFranz;
    pfd[1].events = POLLPRI|POLLERR;

    int n = poll(pfd, 2, timeoutMs);
    if (n < 0) {
        Debug::out(LOG_ERROR, "CConSpi::waitForATNs() poll: %s", strerror(errno));
        return false;
    }
    if(n > 0) {
        if(pfd[0].revents & POLLPRI) {
            lseek(fdAtnHans, 0, SEEK_SET);
            len = read(fdAtnHans, buffer, sizeof(buffer));
            if(len < 0) {
                Debug::out(LOG_ERROR, "CConSpi::waitForATNs() read(HANS): %s", strerror(errno));
            } else {
#ifdef DEBUG_SPI_COMMUNICATION
                Debug::out(LOG_DEBUG, "CConSpi::waitForATNs() read(HANS): %d %.*s", (int)len, (int)len, buffer);
#endif
                if (buffer[0] == '1') {
                    memset(outBuf, 0, 8);
                    if(readHeader(SPI_CS_HANS, outBuf, inBuf)) {    // receive: 0xcafe, ATN code, txLen, rxLen
                        hansAtn = inBuf[3];
                    }
                }
            }
        }
        if(pfd[1].revents & POLLPRI) {
            lseek(fdAtnFranz, 0, SEEK_SET);
            len = read(fdAtnFranz, buffer, sizeof(buffer));
            if (len < 0) {
                Debug::out(LOG_ERROR, "CConSpi::waitForATNs() read(FRANZ): %s", strerror(errno));
            } else {
#ifdef DEBUG_SPI_COMMUNICATION
                Debug::out(LOG_DEBUG, "CConSpi::waitForATNs() read(FRANZ): %d %.*s", (int)len, (int)len, buffer);
#endif
                if (buffer[0] == '1') {
                    memset(outBuf, 0, 8);
                    if(readHeader(SPI_CS_FRANZ, outBuf, inBuf)) {    // receive: 0xcafe, ATN code, txLen, rxLen
                        franzAtn = inBuf[3];
                    }
                }
            }
        }
        return true;
    }
    return false;
}

bool CConSpi::waitForATN(int whichSpiCs, BYTE atnCode, DWORD timeoutMs, BYTE *inBuf)
{
    BYTE outBuf[8];
	
	memset(outBuf, 0, 8);
	memset(inBuf, 0, 8);
	
	// first translate CS signal to ATN signal
	int whichAtnSignal;
	if(whichSpiCs == SPI_CS_HANS) {							// for CS Hans
		whichAtnSignal = SPI_ATN_HANS;						// look for ATN Hans
	} else {												// for CS Franz
		whichAtnSignal = SPI_ATN_FRANZ;						// look for ANT Franz
	}

	applyNoTxRxLimis(whichSpiCs);							// set that we're not limiting the TX/RX count for now
	
	// single check for any ATN code?
	if(atnCode == ATN_ANY) {								// special case - check if any ATN is pending, no wait
		if(!spi_atn(whichAtnSignal)) {						// if that ATN is not up, failed				
			return false;
		}
	
#ifdef DEBUG_SPI_COMMUNICATION
		Debug::out(LOG_DEBUG, "\nwaitForATN(%d, %d) single good!", whichSpiCs, whichAtnSignal);
#endif	
		if(!readHeader(whichSpiCs, outBuf, inBuf)) {		// receive: 0xcafe, ATN code, txLen, rxLen
			return false;
		}
		
		return true;
	}

    // wait for specific ATN code?
    DWORD timeOut = Utils::getEndTime(timeoutMs);

    while(1) {
		if(Utils::getCurrentMs() >= timeOut) {				// if it takes more than allowed timeout, fail
			Debug::out(LOG_ERROR, "waitForATN %02x fail - timeout", atnCode);
			return false;
		}
		
		if( spi_atn(whichAtnSignal) ) {						// if ATN signal is up
			break;
		}
    }

#ifdef DEBUG_SPI_COMMUNICATION
	Debug::out(LOG_DEBUG, "\nwaitForATN starting...");
#endif

	if(!readHeader(whichSpiCs, outBuf, inBuf)) {			// receive: 0xcafe, ATN code, txLen, rxLen
		return false;
	}

    if(inBuf[3] == atnCode) {                      		// ATN code found?
#ifdef DEBUG_SPI_COMMUNICATION
		Debug::out(LOG_DEBUG, "waitForATN %02x good.", atnCode);
#endif		
        return true;
	} else {
		Debug::out(LOG_ERROR, "waitForATN %02x, but received %02x! Fail!", atnCode, inBuf[3]);
        return false;
	}
}

bool CConSpi::readHeader(int whichSpiCs, BYTE *outBuf, BYTE *inBuf)
{
	WORD *inWord = (WORD *) inBuf;
	WORD marker;
	DWORD loops = 0;

    remainingPacketLength[whichSpiCs] = NO_REMAINING_LENGTH;
	// read the first WORD, if it's not 0xcafe, then read again to synchronize
	while(sigintReceived == 0) {
		txRx(whichSpiCs, 2, outBuf, inBuf);					// receive: 0, ATN code, txLen, rxLen
		marker = *inWord;
	
		if(marker == 0xfeca) {								// 0xcafe with reversed bytes
			break;
		}
		
		loops++;
		
		if(loops >= 10000) {								// if this doesn't synchronize in 10k loops, something is very wrong
			Debug::out(LOG_ERROR, "readHeader(%d) couldn't synchronize!", whichSpiCs);
			return false;
		}
	}
	
	if(loops != 0) {
		Debug::out(LOG_DEBUG, "readHeader took %d loops to synchronize!", loops);
	}

	txRx(whichSpiCs, 6, outBuf+2, inBuf+2);					// receive: 0, ATN code, txLen, rxLen
	applyTxRxLimits(whichSpiCs, inBuf);						// now apply txLen and rxLen
	
	return true;
}

void CConSpi::applyTxRxLimits(int whichSpiCs, BYTE *inBuff)
{
    WORD *pwIn = (WORD *) inBuff;

	// words 0 and 1 are 0 and ATN code, words 2 and 3 are txLen, rxLen);
    WORD txLen = swapWord(pwIn[2]);
    WORD rxLen = swapWord(pwIn[3]);

#ifdef DEBUG_SPI_COMMUNICATION
	Debug::out(LOG_DEBUG, "TX/RX limits: TX %d WORDs, RX %d WORDs", txLen, rxLen);
#endif	

	// manually limit the TX and RX len
    if(	whichSpiCs == SPI_CS_HANS  && (txLen > ONE_KB || rxLen > ONE_KB) ) {
        Debug::out(LOG_ERROR, "applyTxRxLimits - TX/RX limits for HANS are probably wrong! Fix this!");
		txLen = MIN(txLen, ONE_KB);
		rxLen = MIN(rxLen, ONE_KB);
	}

    if(	whichSpiCs == SPI_CS_FRANZ  && (txLen > TWENTY_KB || rxLen > TWENTY_KB) ) {
        Debug::out(LOG_ERROR, "applyTxRxLimits - TX/RX limits for FRANZ are probably wrong! Fix this!");
		txLen = MIN(txLen, TWENTY_KB);
		rxLen = MIN(rxLen, TWENTY_KB);
	}

    setRemainingTxRxLen(whichSpiCs, txLen, rxLen);
}

WORD CConSpi::swapWord(WORD val)
{
    WORD tmp = 0;

    tmp  = val << 8;
    tmp |= val >> 8;

    return tmp;
}

void CConSpi::setRemainingTxRxLen(int whichSpiCs, WORD txLen, WORD rxLen)
{
#ifdef DEBUG_SPI_COMMUNICATION
    if(txLen != NO_REMAINING_LENGTH || rxLen != NO_REMAINING_LENGTH || remainingPacketLength[whichSpiCs] != NO_REMAINING_LENGTH) {
		Debug::out(LOG_DEBUG, "CConSpi::setRemainingTxRxLen - TX %d, RX %d, while the remainingPacketLength is %d", txLen, rxLen, remainingPacketLength[whichSpiCs]);
	}
#endif	
		
    if(txLen == NO_REMAINING_LENGTH && rxLen == NO_REMAINING_LENGTH) {    // if setting NO_REMAINING_LENGTH
        if(remainingPacketLength[whichSpiCs] != 0 && remainingPacketLength[whichSpiCs] != NO_REMAINING_LENGTH) {
            Debug::out(LOG_ERROR, "CConSpi(%d) - didn't TX/RX enough data, padding with %d zeros! Fix this!", whichSpiCs, remainingPacketLength[whichSpiCs]);
            memset(paddingBuffer, 0, PADDINGBUFFER_SIZE);
            txRx(whichSpiCs, remainingPacketLength[whichSpiCs], paddingBuffer, paddingBuffer);
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
    remainingPacketLength[whichSpiCs] = (txLen > rxLen) ? txLen : rxLen;
}

WORD CConSpi::getRemainingLength(int whichSpiCs)
{
    return remainingPacketLength[whichSpiCs];
}

void CConSpi::txRx(int whichSpiCs, int count, BYTE *sendBuffer, BYTE *receiveBufer)
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
        count = remainingPacketLength[whichSpiCs];
    }

    if(remainingPacketLength[whichSpiCs] != NO_REMAINING_LENGTH) {
        if(count > remainingPacketLength[whichSpiCs]) {
            Debug::out(LOG_ERROR, "CConSpi::txRx(%d) - trying to TX/RX %d more bytes than allowed! Fix this! %d %d", whichSpiCs, (count - remainingPacketLength[whichSpiCs]), count, remainingPacketLength[whichSpiCs]);

            count = remainingPacketLength[whichSpiCs];
        }
    }

#ifdef DEBUG_SPI_COMMUNICATION
    Debug::out(LOG_DEBUG, "CConSpi::txRx(%d) - count: %d", whichSpiCs, count);
#endif	

	spi_tx_rx(whichSpiCs, count, sendBuffer, receiveBufer);

    if(remainingPacketLength[whichSpiCs] != NO_REMAINING_LENGTH) {
        remainingPacketLength[whichSpiCs] -= count;             // mark that we've send/received this much data
    }
}

