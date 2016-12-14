//--------------------------------------------------
#include <mint/osbind.h> 
#include <mint/linea.h> 
#include <stdio.h>

#include "acsi.h"
#include "translated.h"
#include "gemdos.h"
#include "gemdos_errno.h"
#include "VT52.h"
#include "cookiejar.h"
#include "version.h"
#include "hdd_if.h"
#include "stdlib.h"

//--------------------------------------------------

void showConnectionErrorMessage(void);
BYTE findDevice(void);
int getConfig(void); 
int readHansTest (DWORD byteCount, WORD xorVal );
int writeHansTest(DWORD byteCount, WORD xorVal );
void sleep(int seconds);

void print_head(void);
void print_status(void);

void showHexByte(BYTE val);
void showHexDword(DWORD val);
void logMsg(char *logMsg);
void deleteErrorLines(void);
void speedTest(void);
void generateDataOnPartition(void);

BYTE ce_identify(BYTE ACSI_id);
//--------------------------------------------------
BYTE deviceID;

BYTE commandLong [CMD_LENGTH_LONG ] = {0x1f, 0xA0, 'C', 'E', HOSTMOD_TRANSLATED_DISK, 0, 0, 0, 0, 0, 0, 0, 0}; 

BYTE readBuffer [256 * 512];
BYTE writeBuffer[256 * 512];
BYTE *rBuffer, *wBuffer;

BYTE prevCommandFailed;

#define HOSTMOD_CONFIG              1
#define HOSTMOD_LINUX_TERMINAL      2
#define HOSTMOD_TRANSLATED_DISK     3
#define HOSTMOD_NETWORK_ADAPTER     4
#define HOSTMOD_FDD_SETUP           5

#define TRAN_CMD_IDENTIFY           0
#define TRAN_CMD_GETDATETIME        1

#define DATE_OK                     0
#define DATE_ERROR                  2
#define DATE_DATETIME_UNKNOWN       4

#define DEVTYPE_OFF                 0
#define DEVTYPE_SD                  1
#define DEVTYPE_RAW                 2
#define DEVTYPE_TRANSLATED          3

#define Clear_home()    (void) Cconws("\33E")

#define ERROR_LINE_START        10

BYTE ifUsed;

// Warning! Don't use VT52_Save_pos() and VT52_Load_pos(), because they don't work on Falcon! (They work fine on ST and TT.)

int errorLine = 0;
BYTE simpleNotDetailedErrReport = 1;

void hdIfCmdAsUser(BYTE readNotWrite, BYTE *cmd, BYTE cmdLength, BYTE *buffer, WORD sectorCount);

typedef struct {
    DWORD goodPlain;
    DWORD goodWithRetry;
    DWORD errorTimeout;
    DWORD errorCrc;
    DWORD errorOther;
} Tresults;

struct {
    DWORD run;
    DWORD singleOps;
    
    Tresults read;
    Tresults write;
} counts;

void updateCounts(BYTE doingWriteNotRead, int res);
void printOpResult(int res);

void testDataReliability(void);
void testContinousRead  (BYTE testReadNotSDcard);

WORD getTOSversion(void)
{
    // detect TOS version and try to automatically choose the interface
    BYTE  *pSysBase     = (BYTE *) 0x000004F2;
    BYTE  *ppSysBase    = (BYTE *)  ((DWORD )  *pSysBase);                      // get pointer to TOS address
    WORD  tosVersion    = (WORD  ) *(( WORD *) (ppSysBase + 2));                // TOS +2: TOS version
    return tosVersion;
}

//--------------------------------------------------
int main(void)
{
    BYTE key;
    DWORD toEven;

    //----------------------
    // read all the keys which are waiting, so we can ignore them
    while(1) {
        if(Cconis() != 0) {             // if some key is waiting, read it
            Cnecin();
        } else {                        // no key is waiting, quit the loop
            break;
        }
    }
    //----------------------
    
    lineaa();   // hide mouse
    
    prevCommandFailed = 0;
    
    // ---------------------- 
    // create buffer pointer to even address 
    toEven = (DWORD) &readBuffer[0];
  
    if(toEven & 0x0001)       // not even number? 
        toEven++;
  
    rBuffer = (BYTE *) toEven; 

    //----------
    toEven = (DWORD) &writeBuffer[0];
  
    if(toEven & 0x0001)       // not even number? 
        toEven++;
  
    wBuffer = (BYTE *) toEven; 

    Clear_home();

    // ---------------------- 
    print_head();
    (void) Cconws("\r\n");
    (void) Cconws(" Non-destructive ACSI read/write test.\r\n");
    (void) Cconws(" Helpful to detect possible DMA problems\r\n");      
    (void) Cconws(" your ST hardware might have. See:\r\n");        
    (void) Cconws(" http://joo.kie.sk/?page_id=250 and \r\n");
    (void) Cconws(" https://goo.gl/bKcbNV for infos+fixes.\r\n\r\n");        
    
    unsigned long *cescreencast_cookie=0;
    if(CookieJarReadAsUser(0x43455343,(unsigned long *) &cescreencast_cookie) != 0) // Cookie "CESC" 
    { 
        (void) Cconws("\r\n");
        (void) Cconws(" CosmosEx Screencast is active. Please\r\n");
        (void) Cconws(" deactivate. \r\n\r\n Press any key to quit.\r\n");
        Cnecin();
        (void) Cconws("Quit.");         
        
        linea9();   
        return 0;
    }
    
    //----------------------
    // detect TOS version and try to automatically choose the interface
    WORD  tosVersion    = Supexec(getTOSversion);
    BYTE  tosMajor      = tosVersion >> 8;
    
    if(tosMajor == 1 || tosMajor == 2) {                // TOS 1.xx or TOS 2.xx -- running on ST
        (void) Cconws("Running on ST, choosing ACSI interface.");
    
        hdd_if_select(IF_ACSI);
        ifUsed      = IF_ACSI;
    } else if(tosMajor == 4) {                          // TOS 4.xx -- running on Falcon
        (void) Cconws("Running on Falcon, choosing SCSI interface.");
    
        hdd_if_select(IF_SCSI_FALCON);
        ifUsed      = IF_SCSI_FALCON;
    } else {                                            // TOS 3.xx -- running on TT
        (void) Cconws("Running on TT, plase choose [A]CSI or [S]CSI:");
        
        while(1) {
            key = Cnecin();
            
            if(key == 'a' || key == 'A') {      // A pressed, choosing ACSI
                (void) Cconws("\n\rACSI selected.\n\r");
                hdd_if_select(IF_ACSI);
                ifUsed      = IF_ACSI;
                break;
            }
            
            if(key == 's' || key == 'S') {      // S pressed, choosing SCSI
                (void) Cconws("\n\rSCSI selected.\n\r");
                hdd_if_select(IF_SCSI_TT);
                ifUsed      = IF_SCSI_TT;
                break;
            }
        }
    }

    // ---------------------- 
    (void) Cconws("\r\n\r\n[S]imple or [D]etailed error report?\r\n");
    
    key = Cnecin();
    if(key == 'D' || key == 'd') {      // detailed?
        simpleNotDetailedErrReport = 0;
    } else {                            // simple!
        simpleNotDetailedErrReport = 1;
    }
    
    // ---------------------- 
    // search for device on the ACSI bus 
    deviceID = findDevice();

    if( deviceID == 0xff )
    {
        (void) Cconws("Quit.");         

        linea9();   
        return 0;
    }
  
    // ----------------- 
    // now set up the acsi command bytes so we don't have to deal with this one anymore 
    commandLong [0] = (deviceID << 5) | 0x1f;           // cmd[0] = ACSI_id + ICD command marker (0x1f) 

    // ----------------- 

    while(1) {
        VT52_Clear_home();
        
        (void) Cconws("Choose test type:\r\n");
        (void) Cconws("[E] - short speed test\r\n");
        (void) Cconws("[D] - data from RPi     validity check\r\n");
        (void) Cconws("[C] - data from RPI     continous read\r\n");
        (void) Cconws("[S] - data from SD card continous read\r\n");
        (void) Cconws("[G] - generated data on GEMDOS partition\r\n");
        (void) Cconws("[Q] - quit\r\n");

        key = Cnecin();
        
        if(key >= 'A' && key <= 'Z') {          // upper case letter? to lower case
            key += 32;
        }
            
        if(key == 'q') {                        // quit
            break;
        }

        switch(key) {
            case 'e':   speedTest();                break;  // short speed test
            case 'd':   testDataReliability();      break;  // data validity check from RPi
            case 'c':   testContinousRead(1);       break;  // stress test - continous read
            case 's':   testContinousRead(0);       break;  // SD card continous read
            case 'g':   generateDataOnPartition();  break;  // higher level data generation 
        }
    }
    
    // ----------------- 
    
    linea9();                                       // show mouse    
    return 0;
}

BYTE showQuestionGetBool(const char *question, BYTE trueKey, const char *trueWord, BYTE falseKey, const char *falseWord)
{
    // show question
    (void) Cconws(question);

    while(1) {
        BYTE key = Cnecin();

        if(key >= 'A' && key <= 'Z') {          // upper case letter? to lower case
            key += 32;
        }

        if(key == trueKey) {                    // yes
            (void) Cconws(" ");
            (void) Cconws(trueWord);
            (void) Cconws("\r\n");
            return 1;
        }

        if(key == falseKey) {                   // no
            (void) Cconws(" ");
            (void) Cconws(falseWord);
            (void) Cconws("\r\n");
            return 0;
        }
    }
}

BYTE getIntFromUser(BYTE allowZero)
{
    while(1) {
        BYTE key = Cnecin();

        if(key < '0' || key > '9') {            // out of char range? try again
            continue;
        }
        
        if(!allowZero && key == '0') {          // if zero is not allowed, and it's zero, try again
            continue;
        }

        Cconout(key);
        (void) Cconws("\r\n");
        
        return (key - '0');                     // return the number
    }
}

void generateDataOnPartition(void)
{
    VT52_Clear_home();
    (void) Cconws("Generated data on GEMDOS partition\r\n");

    BYTE writeNotVerify = showQuestionGetBool("Write data or verify data? W/V", 'w', "WRITE", 'v', "VERIFY");

    //----------
    // choose drive for testing
    WORD drives = Drvmap();
    (void) Cconws("Choose drive: ");
    int i;
    for(i=2; i<16; i++) {
        if(drives & (1 << i)) {     // drive exists? show letter
            Cconout('A' + i);
        }
    }
    Cconout(' ');

    BYTE testDrive;
    while(1) {
        BYTE key = Cnecin();

        if(key >= 'a' && key <= 'z') {          // lower case letter? to upper case
            key -= 32;
        }

        if(key < 'A' ||key > 'P') {             // out of char range? try again
            continue;
        }

        int driveNo = key - 'A';                // transform char into index
        if(drives & (1 << driveNo)) {           // drive with that index exists? good
            testDrive = key;
            break;
        }
    }

    Cconout(testDrive);
    (void) Cconws("\r\n");
    
    //----------
    (void) Cconws("Choose test file size in MB: ");
    int testFileSizeMb = getIntFromUser(0);

    (void) Cconws("Choose test files count: ");
    int testFileCount = getIntFromUser(0);

    BYTE *pGenerated = wBuffer;
    int j; 
    for(i=0; i<MAXSECTORS; i++) {       // for all the sectors
        for(j=0; j<512; j++) {          // for all bytes in sector
            if(j == 0) {                // index 0: sector #
                *pGenerated = i;
            } else {                    // other indices: index in sector
                *pGenerated = j;
            }
            pGenerated++;
        }
    }

    int fileSizeInBuffers = testFileSizeMb * 8;     // 8 write buffers per MB -> total buffers per whole file size
    for(i=0; i<testFileCount; i++) {                // for all files
        char testFilePath[32] = "X:\\TSTFILEY.BIN"; // pattern for filename
        
        testFilePath[ 0] = testDrive;               // store test drive letter
        testFilePath[10] = i + '0';                 // store index to filename

        int f;
        if(writeNotVerify) {                        // for write
            (void) Cconws("Writing ");
            (void) Cconws(testFilePath);
            (void) Cconws(": ");
            
            f = Fcreate(testFilePath, 0);           // create for writing
        } else {                                    // for read
            (void) Cconws("Verifying ");
            (void) Cconws(testFilePath);
            (void) Cconws(": ");

            f = Fopen(testFilePath, 0);             // open for reading
        }

        if(f < 0) {                                 // if failed
            (void) Cconws("fail\r\n");
            continue;
        }

        int res;
        int bufferSize = MAXSECTORS * 512;
        for(j=0; j<fileSizeInBuffers; j++) {
            if(writeNotVerify) {                    // for write
                res = Fwrite(f, bufferSize, wBuffer);

                if(res == bufferSize) {             // written everything? good
                    (void) Cconws("*");
                } else {                            // something not written? fail
                    (void) Cconws("-");
                }
            } else {                                // for read
                res = Fread(f, bufferSize, rBuffer);

                if(res == bufferSize) {             // read everything? check the content
                    res = memcomp(rBuffer, wBuffer, bufferSize);

                    if(res == 0) {                  // everything fine? 
                        (void) Cconws("*");
                    } else {                        // some mismatch?
                        (void) Cconws("c");
                    }
                } else {                            // something not read? fail
                    (void) Cconws("-");
                }
            }
        }

        Fclose(f);                                  // close file
        (void) Cconws("\r\n");
    }

    (void) Cconws("\r\nDone. Press any key to continue.\r\n");
    Cnecin();
}

void scsi_reset(void);

BYTE getSDcardId(void)
{
    BYTE cmd[CMD_LENGTH_SHORT] = {0, 'C', 'E', HOSTMOD_TRANSLATED_DISK, TEST_GET_ACSI_IDS, 0};

    cmd[0] = (deviceID << 5);                                       // cmd[0] = deviceID + TEST UNIT READY (0)   
    memset(rBuffer, 0, 512);                                        // clear the buffer 

    hdIfCmdAsUser(ACSI_READ, cmd, CMD_LENGTH_SHORT, rBuffer, 1);
    if(!hdIf.success || hdIf.statusByte != 0) {                     // if command failed, return -1 (0xff)
        return 0xff;
    }
    
    int i;
    for(i=0; i<8; i++) {                                            // go through ACSI IDs
        if(rBuffer[i] == DEVTYPE_SD) {                              // if found SD card, good!
            return i;
        }
    }
    
    return 0xff;                                                    // SD card ACSI ID not found
}

void testContinousRead(BYTE testReadNotSDcard)
{
    VT52_Clear_home();
    VT52_Wrap_on();
    (void) Cconws("Continous read without data checking\r\n");

    BYTE pauseUntilKeypressAfterFail    = showQuestionGetBool("On fail - pause until key pressed? Y/N", 'y', "YES", 'n', "NO");
    BYTE doScsiResetAfterFail           = showQuestionGetBool("On fail - do SCSI reset?           Y/N", 'y', "YES", 'n', "NO");
    BYTE showGoodResultAsterisk         = showQuestionGetBool("Show asterisk after good result?   Y/N", 'y', "YES", 'n', "NO");

    hdIf.maxRetriesCount = 0;                           // disable retries

    BYTE sdReadCmd[CMD_LENGTH_SHORT] = {0, 0, 0, 0, 0, 0};

    if(testReadNotSDcard) {                             // should do test on CE generated data?
        (void) Cconws("Data source: Raspberry Pi\r\n");

        DWORD byteCount = ((DWORD) MAXSECTORS) << 9;    // convert sector count to byte count ( sc * 512 )

        commandLong[4+1] = TEST_READ;

        // size to read
        commandLong[5+1] = (byteCount >> 16) & 0xFF;
        commandLong[6+1] = (byteCount >>  8) & 0xFF;
        commandLong[7+1] = (byteCount      ) & 0xFF;

        // Word to XOR with data on CE side
        commandLong[8+1] = 0;
        commandLong[9+1] = 0;
    } else {                                            // should do test on SD card?
        (void) Cconws("Data source: SD card\r\n");
        
        BYTE sdCardId = getSDcardId();
        
        if(sdCardId == 0xff) {                          // SD card not configured on ACSI bus?
            (void) Cconws("No ACSI/SCSI ID configured for SD card!\r\n");
            (void) Cconws("Set it and try again.\r\n");
            Cnecin();
            return;                                     // fail and quit
        }

        // generate READ(6) command
        sdReadCmd[0] = (sdCardId << 5) | 0x08;          // cmd[0] = ACSI id of SD card + SCSI READ(6)
        // sdReadCmd[1] - leave 0
        // sdReadCmd[2] - sector # msb
        // sdReadCmd[3] - sector # lsb
        sdReadCmd[4] = MAXSECTORS;
        // v[5] - leave 0
    }

    (void) Cconws("Press 'R' for SCSI bus RESET.\r\n");
    (void) Cconws("Press 'Q' to quit.\r\n");
    
    while(1) {
        if(Cconis() != 0) {                         // if some key is waiting
            BYTE key = Cnecin();

            if(key == 'q' || key == 'Q') {
                break;
            }

            if(key == 'r' || key == 'R') {
                scsi_reset();
                continue;
            }
        }

        if(testReadNotSDcard) {     // should do test on CE generated data?
            hdIfCmdAsUser(ACSI_READ, commandLong, CMD_LENGTH_LONG, rBuffer, MAXSECTORS);    // issue the command and check the result
        } else {                    // should do test on SD card?
            DWORD randomSectorNo = getTicks();

            sdReadCmd[2] = (BYTE) (randomSectorNo >> 8);    // sector # high
            sdReadCmd[3] = (BYTE) (randomSectorNo     );    // sector # low

            hdIfCmdAsUser(ACSI_READ, sdReadCmd, CMD_LENGTH_SHORT, rBuffer, MAXSECTORS);      // issue the command and check the result
        }

        if(!hdIf.success || hdIf.statusByte != 0) { // failed?
            if(doScsiResetAfterFail) {
                scsi_reset();
            }

            (void) Cconws("_");

            if(pauseUntilKeypressAfterFail) {
                (void) Cconws("\r\nCMD failed, press any key to continue.\r\n");
                Cnecin();
            }

            continue;
        } else {                                    // everything OK
            if(showGoodResultAsterisk) {
                (void) Cconws("*");
            }
        }
    }
}

void testDataReliability(void)
{
    BYTE key;
    WORD xorVal=0xC0DE;
    int charcnt=0;
    int linecnt=0;

    VT52_Clear_home();

    print_head();
    print_status();
   
    VT52_Goto_pos(0, 22);
    (void) Cconws("Press 'Q' to quit the test...\r\n");         
    (void) Cconws("Testing (*=OK,C=Crc,_=Timeout):\r\n");       

    int x = 0;                          // this variable will store current X position of carret, so we can return to the right position after some VT52_Goto_pos()
    VT52_Goto_pos(0, 24);

    (void) Cconws("R:");
    x += 2;
    
    BYTE doingWriteNotRead;
    
    while(1)
    {
        if(Cconis() != 0) {             // if some key is waiting
            key = Cnecin();
            
            if(key == 'q' || key == 'Q') {
                break;
            }
        }
    
        int res=0;
  
        errorLine = ERROR_LINE_START;
  
        doingWriteNotRead = linecnt & 1;
  
        if( doingWriteNotRead ){
            res = writeHansTest(MAXSECTORS * 512, xorVal);
        }else{
            res = readHansTest (MAXSECTORS * 512, xorVal);
        }
        
        xorVal++;  // change XOR value every time, so the data will be different every time (better for detecting errors)
        
        updateCounts(doingWriteNotRead, res);           // update the results count
        
        VT52_Goto_pos(x, 24);
        printOpResult(res);                             // show the result as a single characted ( * _ c ? )
        
        x++;
        charcnt++;
        
        print_status();
        print_head();

        //--------------
        if(!simpleNotDetailedErrReport) {                           // if detailed error report with wait
            if(errorLine != ERROR_LINE_START) {                     // if some error happened, wait for key
                BYTE quitIt = 0;
                
                logMsg("Shit happened, press 'C' to continue or 'Q' to quit.");    // show message
                
                while(1) {                                          // wait for 'c' key or 'q' key
                    key = Cnecin();
                    
                    if(key == 'C' || key == 'c') {
                        break;
                    }
                    
                    if(key == 'Q' || key == 'q') {
                        quitIt = 1;
                        break;
                    }
                }
                
                if(quitIt) {                                        // should quit app?
                    break;
                }
            }
            
            deleteErrorLines();
        }
        //--------------

        VT52_Goto_pos(x, 24);
        
        if( charcnt>=40-2 ){
            VT52_Goto_pos(0,4);
            VT52_Del_line();
            
            VT52_Goto_pos(0, 24);
            x = 0;
            
            charcnt=0;
            linecnt++;
            
            doingWriteNotRead = linecnt & 1;
            
            if( doingWriteNotRead ){
                (void) Cconws("W:");
            }else{
                (void) Cconws("R:");
                counts.run++;
            }
            x += 2;
        }
    }    
}

void updateCounts(BYTE doingWriteNotRead, int res)
{
    counts.singleOps++;

    // get the pointer to the right structure
    Tresults *pResults = doingWriteNotRead ? &counts.write : &counts.read;
    
    switch( res )
    {
        case 0:                                 // test succeeded
            if(hdIf.retriesDoneCount == 0) {    // success without retries
                pResults->goodPlain++;
            } else {                            // success with some retries
                pResults->goodWithRetry++;
            }
            break;
        //---------------
        case -1:                                // test failed with communication error
            pResults->errorTimeout++;
            break;
        //---------------
        case -2:                                // test failed with CRC error
            pResults->errorCrc++;
            break;
        //---------------
        default:                                // some other error?
            pResults->errorOther++;
            break;
    }    
}

void printOpResult(int res) 
{
    switch( res )
    {
        case 0:                                 // test succeeded
            if(hdIf.retriesDoneCount == 0) {    // success without retries
                Cconout('*');
            } else {                            // success with some retries
                char retryCountChar = (hdIf.retriesDoneCount < 9) ? ('0' + hdIf.retriesDoneCount) : '9';
                Cconout(retryCountChar);
            }
            break;
        //---------------
        case -1:                                // test failed with communication error
            Cconout('_');
            break;
        //---------------
        case -2:                                // test failed with CRC error
            Cconout('c');
            break;
        //---------------
        default:                                // some other error?
            Cconout('?');
            break;
    }    
}

void print_head()
{
    VT52_Goto_pos(0,0);

    (void) Cconws("\33p[ CosmosEx HDD IF test  ver "); 
    showAppVersion();
    (void) Cconws(" ]\33q\r\n");        
}

void print_status(void)
{
    char colorOrBw = 0;

    //-------------
    // 2nd row - general info
    VT52_Goto_pos(0,1);
    (void) Cconws("\33p[ Run:");
    showInt(counts.run, 3);
    
    (void) Cconws("  Ops: ");
    showInt(counts.singleOps, 5);
    
    (void) Cconws("  Data: ");

    // calculate and show transfered data capacity
    DWORD kilobytes     = 127 * counts.singleOps;       // 127 kB per operation * operations count
    int megsInteger     = kilobytes / 1024;             // get integer part of mega_bytes
    int megsFraction    = (kilobytes % 1024) / 100;     // get mega_bytes_after_decimal_point part
    
    showIntWithPrepadding(megsInteger, 4, ' ');
    Cconout('.');
    showInt(megsFraction, 1);
    (void) Cconws(" MB ]\33q\r\n");

    //-------------
    // 3rd row - read statistics
    if(colorOrBw) {
        (void) Cconws("\33p[ R \33c2Gp:");
    } else {
        (void) Cconws("\33p[ R Gp:");
    }
        
    showInt(counts.read.goodPlain,      4);
    
    (void) Cconws(" Gr:");
    showInt(counts.read.goodWithRetry,  3);
    
    if(colorOrBw) {
        (void) Cconws(" \33c1T/O:");
    } else {
        (void) Cconws(" T/O:");
    }
    
    showInt(counts.read.errorTimeout,   2);

    (void) Cconws(" CRC:");
    showInt(counts.read.errorCrc,       2);

    (void) Cconws(" Oth:");
    showInt(counts.read.errorOther,     2);

    if(colorOrBw) {
        (void) Cconws("\33c0]\33q\r\n");
    } else {
        (void) Cconws("]\33q\r\n");
    }
    
    //-------------
    // 4rd row - write statistics
    if(colorOrBw) {
        (void) Cconws("\33p[ W \33c2Gp:");
    } else {
        (void) Cconws("\33p[ W Gp:");
    }
    
    showInt(counts.write.goodPlain,     4);
    
    (void) Cconws(" Gr:");
    showInt(counts.write.goodWithRetry, 3);
    
    if(colorOrBw) {
        (void) Cconws(" \33c1T/O:");
    } else {
        (void) Cconws(" T/O:");
    }
    showInt(counts.write.errorTimeout,  2);

    (void) Cconws(" CRC:");
    showInt(counts.write.errorCrc,      2);

    (void) Cconws(" Oth:");
    showInt(counts.write.errorOther,    2);

    if(colorOrBw) {
        (void) Cconws("\33c0]\33q\r\n");
    } else {
        (void) Cconws("]\33q\r\n");
    }
}

//--------------------------------------------------
BYTE ce_identify(BYTE ACSI_id)
{
  BYTE cmd[CMD_LENGTH_SHORT] = {0, 'C', 'E', HOSTMOD_TRANSLATED_DISK, TRAN_CMD_IDENTIFY, 0};
  
  cmd[0] = (ACSI_id << 5);                  // cmd[0] = ACSI_id + TEST UNIT READY (0)   
  memset(rBuffer, 0, 512);                  // clear the buffer 

  hdIfCmdAsUser(ACSI_READ, cmd, CMD_LENGTH_SHORT, rBuffer, 1);   // issue the identify command and check the result 
    
  if(!hdIf.success || hdIf.statusByte != 0) {                   // if failed, return FALSE 
    return 0;
  }
    
  if(strncmp((char *) rBuffer, "CosmosEx translated disk", 24) != 0) {      // the identity string doesn't match? 
    return 0;
  }
    
  return 1;                             // success 
}
//--------------------------------------------------
void showConnectionErrorMessage(void)
{
//  Clear_home();
    (void) Cconws("Communication with CosmosEx failed.\nWill try to reconnect in a while.\n\nTo quit to desktop, press F10\n");
    
    prevCommandFailed = 1;
}
//--------------------------------------------------
BYTE findDevice(void)
{
    BYTE i;
    BYTE key, res;
    BYTE id = 0xff;
    char bfr[2];

    hdIf.maxRetriesCount = 0;           // disable retries - we are expecting that the devices won't answer on every ID
    
    bfr[1] = 0; 
    (void) Cconws("Looking for CosmosEx on ");
    
    switch(ifUsed) {
        case IF_ACSI:           (void) Cconws("ACSI: ");        break;
        case IF_SCSI_TT:        (void) Cconws("TT SCSI: ");     break;
        case IF_SCSI_FALCON:    (void) Cconws("Falcon SCSI: "); break;
    }

    while(1) {
        for(i=0; i<8; i++) {
            bfr[0] = i + '0';
            (void) Cconws(bfr); 
              
            res = ce_identify(i);                       // try to read the IDENTITY string 
      
            if(res == 1) {                              // if found the CosmosEx 
                id = i;                                 // store the ACSI ID of device 
                break;
            }
        }
  
        if(res == 1) {                                  // if found, break 
            break;
        }
      
        (void) Cconws(" - not found.\r\nPress any key to retry or 'Q' to quit.\r\n");
        key = Cnecin();        
    
        if(key == 'Q' || key=='q') {
            hdIf.maxRetriesCount = 16;                  // enable retries
            return 0xff;
        }
    }
  
    hdIf.maxRetriesCount = 16;                          // enable retries
    
    bfr[0] = id + '0';
    (void) Cconws("\r\nCosmosEx ID: ");
    (void) Cconws(bfr);
    (void) Cconws("\r\n\r\n");
    
    return id;
}

//--------------------------------------------------

int readHansTest(DWORD byteCount, WORD xorVal )
{
    commandLong[4+1] = TEST_READ;

    // size to read
    commandLong[5+1] = (byteCount >> 16) & 0xFF;
    commandLong[6+1] = (byteCount >>  8) & 0xFF;
    commandLong[7+1] = (byteCount      ) & 0xFF;

    // Word to XOR with data on CE side
    commandLong[8+1] = (xorVal >> 8) & 0xFF;
    commandLong[9+1] = (xorVal     ) & 0xFF;

    memset(rBuffer, 0, 8);                  // clear first few bytes so we can detect if the data was really read, or it was only retained from last write in the buffer    
        
    hdIfCmdAsUser(ACSI_READ, commandLong, CMD_LENGTH_LONG, rBuffer, (byteCount+511)>>9 );        // issue the command and check the result
    
    if(!hdIf.success) {                     // FAIL? quit...
        return -1;
    }
    
    // if we came here, then either there's no error, or there's error but we still want to compare the buffers
    BYTE retVal;
    if(hdIf.success && hdIf.statusByte == OK) { // no error - at the end just return 0
        retVal = 0;
    } else {            // some error - at the end return -1
        retVal = -1;
    }
    
    WORD counter = 0;
    WORD data = 0;
    DWORD i;
    for(i=0; i<byteCount; i += 2) {
        data = counter ^ xorVal;       // create word
        if( !(rBuffer[i]==(data>>8) && rBuffer[i+1]==(data&0xFF)) ){
            return -2;
        }  
        counter++;
    }

    if(byteCount & 1) {                                 // odd number of bytes? add last byte
        BYTE lastByte = (counter ^ xorVal) >> 8;
        if( rBuffer[byteCount-1]!=lastByte ){
            return -2;
        }  
    }
    
    if(retVal != 0) {       // if we came here, then either there's no xCSI error, or there was xCSI error but the received buffer is still OK
        logMsg("xCSI cmd failed, but the READ buffer seems to be OK, wtf?!");
    }
    
    return retVal;
}

//--------------------------------------------------

int writeHansTest(DWORD byteCount, WORD xorVal)
{
    static WORD prevXorVal = 0xffff;
    
    commandLong[4+1] = TEST_WRITE;

  //size to read
    commandLong[5+1] = (byteCount >> 16) & 0xFF;
    commandLong[6+1] = (byteCount >> 8 ) & 0xFF;
    commandLong[7+1] = (byteCount      ) & 0xFF;

  //Word to XOR with data on CE side
    commandLong[8+1] = (xorVal >> 8) & 0xFF;
    commandLong[9+1] = (xorVal     ) & 0xFF;

    if(prevXorVal != xorVal) {              // if xorVal changed since last call, generate buffer (otherwise skip that)
        prevXorVal = xorVal;
    
        WORD counter = 0;
        WORD data = 0;
        DWORD i;
        for(i=0; i<byteCount; i += 2) {
            data = counter ^ xorVal;       // create word
            wBuffer[i  ]    = (data >> 8);
            wBuffer[i+1]    = (data &  0xFF);
            counter++;
        }

        if(byteCount & 1) {                                 // odd number of bytes? add last byte
            BYTE lastByte           = (counter ^ xorVal) >> 8;
            wBuffer[byteCount-1]    = lastByte;
        }
    }

    hdIfCmdAsUser(ACSI_WRITE, commandLong, CMD_LENGTH_LONG, wBuffer, (byteCount+511)>>9 );       // issue the command and check the result
    
    if(!hdIf.success) {                 // fail?
logMsg("writeHansTest: not success");
        return -1;
    }
    
    if(hdIf.statusByte == E_CRC) {      // status byte: CRC error?
logMsg("writeHansTest: E_CRC");
        return -2;
    }
    
    if(hdIf.statusByte != 0) {          // some other error?
logMsg("writeHansTest: status byte is ");
showHexByte(hdIf.statusByte);
        return -3;
    }
    
    return 0;
}

void logMsg(char *logMsg)
{
    if(simpleNotDetailedErrReport) {    // if simple, don't show these SCSI log messages
        return;
    }

    VT52_Goto_pos(0, errorLine++);
    
    (void) Cconws(logMsg);
}

void deleteErrorLines(void)
{
    int line;
    
    for(line=ERROR_LINE_START; line<errorLine; line++) {
        VT52_Goto_pos(0, line);
        (void) Cconws("                                        ");
    }
}

void logMsgProgress(DWORD current, DWORD total)
{
    VT52_Goto_pos(0, errorLine++);
    
    (void) Cconws("Progress: ");
    showHexDword(current);
    (void) Cconws(" out of ");
    showHexDword(total);
    (void) Cconws("\n\r");
}

void showHexByte(BYTE val)
{
    int hi, lo;
    char tmp[3];
    char table[16] = {"0123456789ABCDEF"};
    
    hi = (val >> 4) & 0x0f;;
    lo = (val     ) & 0x0f;

    tmp[0] = table[hi];
    tmp[1] = table[lo];
    tmp[2] = 0;
    
    (void) Cconws(tmp);
}

void showHexDword(DWORD val)
{
    showHexByte((BYTE) (val >> 24));
    showHexByte((BYTE) (val >> 16));
    showHexByte((BYTE) (val >>  8));
    showHexByte((BYTE)  val);
}

void speedTest(void)
{
    DWORD byteCount = ((DWORD) MAXSECTORS) << 9;     // convert sector count to byte count ( sc * 512 )

    commandLong[4+1] = TEST_READ;

    // size to read
    commandLong[5+1] = (byteCount >> 16) & 0xFF;
    commandLong[6+1] = (byteCount >>  8) & 0xFF;
    commandLong[7+1] = (byteCount      ) & 0xFF;

    // Word to XOR with data on CE side
    commandLong[8+1] = 0;
    commandLong[9+1] = 0;

    VT52_Goto_pos(0, 23);
    (void) Cconws("Read speed: ");
    
    DWORD now, until, diff;
    now = getTicksAsUser();
    
    int i;
    
    for(i=0; i<20; i++) {
        hdIfCmdAsUser(ACSI_READ, commandLong, CMD_LENGTH_LONG, rBuffer, MAXSECTORS );  // issue the command and check the result

        if(!hdIf.success) {                     // ACSI ERROR?
            (void) Cconws("fail -- on ");
            showInt(i, 2);
            (void) Cconws("out of ");
            showInt(20, 2);
            (void) Cconws("\n\r");

            (void) Cnecin();
            return;
        }
    }
    
    until   = getTicksAsUser();
    diff    = until - now;

    int timeMs  = (diff * 1000) / 200;
    int kbps    = ((20 * MAXSECTORS) * 500) / timeMs;
    
    showInt(kbps, -1);
    (void) Cconws(" kB/s\n\r");
    
    (void) Cconws("Press any key to continue...\n\r");
    (void) Cnecin();
}

//--------------------------------------------------
// global variables, later used for calling hdIfCmdAsSuper
BYTE __readNotWrite, __cmdLength;
WORD __sectorCount;
BYTE *__cmd, *__buffer;

void hdIfCmdAsSuper(void)
{
    // this should be called through Supexec()
    (*hdIf.cmd)(__readNotWrite, __cmd, __cmdLength, __buffer, __sectorCount);
}

void hdIfCmdAsUser(BYTE readNotWrite, BYTE *cmd, BYTE cmdLength, BYTE *buffer, WORD sectorCount)
{
    // store params to global vars
    __readNotWrite  = readNotWrite;
    __cmd           = cmd;
    __cmdLength     = cmdLength;
    __buffer        = buffer;
    __sectorCount   = sectorCount;    
    
    // call the function which does the real work, and uses those global vars
    Supexec(hdIfCmdAsSuper);
}
