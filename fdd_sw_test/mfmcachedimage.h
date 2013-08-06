#ifndef MFMCACHEDIMAGE_H
#define MFMCACHEDIMAGE_H

#include "ifloppyimage.h"

// maximum 2 sides, 85 tracks per side
#define MAX_TRACKS      (2 * 85)

typedef struct {
    int     track;
    int     side;

    BYTE    *mfmStream;
    int     bytesInStream;
} TCachedTrack;

class MfmCachedImage
{
public:
    MfmCachedImage();
    ~MfmCachedImage();

    void encodeAndCacheImage(IFloppyImage *img);
    void deleteCachedImage(void);
    BYTE *getEncodedTrack(int track, int side, int &bytesInBuffer);

private:
    bool gotImage;

    TCachedTrack tracks[MAX_TRACKS];
    WORD                CRC;

    void initTracks(void);
    void encodeSingleTrack(IFloppyImage *img, int side, int track, int sectorsPerTrack,  BYTE *buffer, int &bytesStored);

    void appendCurrentSectorCommand(int track, int side, int sector, BYTE *buffer, int &count);
    void appendRawByte(BYTE val, BYTE *bfr, int &cnt);
    void appendZeroIfNeededToMakeEven(BYTE *bfr, int &cnt);
    void appendA1MarkToStream(BYTE *bfr, int &cnt);
    void appendTime(BYTE time, BYTE *bfr, int &cnt);
    void appendChange(BYTE chg, BYTE *bfr, int &cnt);
    void appendByteToStream(BYTE val, BYTE *bfr, int &cnt);
    bool createMfmStream(IFloppyImage *img, int side, int track, int sector, BYTE *buffer, int &count);
    void fdc_add_to_crc(WORD &crc, BYTE data);

};

#endif // MFMCACHEDIMAGE_H
