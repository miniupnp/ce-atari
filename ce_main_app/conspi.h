// vim: tabstop=4 softtabstop=4 shiftwidth=4 expandtab
#ifndef CONSPI_H
#define CONSPI_H

#include <map>
#include "datatypes.h"

#define NO_REMAINING_LENGTH     0xffff
#define TXRX_COUNT_REST         0xffff

#define ONE_KB					1024
#define TWENTY_KB				(20 * 1024)
#define PADDINGBUFFER_SIZE      TWENTY_KB

class CConSpi
{
public:
    CConSpi();
    ~CConSpi();

    bool waitForATNs(int timeoutMs, BYTE & hansAtn, BYTE & franzAtn);
	bool waitForATN(int whichSpiCs, BYTE atnCode, DWORD timeoutMs, BYTE *inBuf);
    void txRx(int whichSpiCs, int count, BYTE *sendBuffer, BYTE *receiveBufer);

	void applyTxRxLimits(int whichSpiCs, BYTE *inBuff);
	void applyNoTxRxLimis(int whichSpiCs);
		
    void setRemainingTxRxLen(int whichSpiCs, WORD txLen, WORD rxLen);
    WORD getRemainingLength(int whichSpiCs);

private:
    std::map<int, WORD> remainingPacketLength;
    BYTE *paddingBuffer;
    int fdAtnHans;
    int fdAtnFranz;
    bool firstTimeReadingAtn;

	bool readHeader(int whichSpiCs, BYTE *outBuf, BYTE *inBuf);	
    WORD swapWord(WORD val);
};

#endif // CONSPI_H
