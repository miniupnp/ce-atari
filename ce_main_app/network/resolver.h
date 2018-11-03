// vim: tabstop=4 softtabstop=4 shiftwidth=4 expandtab
#ifndef _RESOLVE_H_
#define _RESOLVE_H_

#include <netdb.h>
#include <string>

#include "../datatypes.h"

#define RESOLV_COUNT    10
struct Tresolv {
             DWORD          startTime;
    volatile BYTE           getaddrinfoHasFinished;
    volatile BYTE           processed;
             int            error;

             char           hostName[256];
             char           canonName[256];

             int            count;
             BYTE           data[128];
             std::string    h_name;

#if defined(__linux__)
             gaicb          req;
#endif
             addrinfo       hints;
};

class ResolverRequest {
public:
    int  addRequest         (const char *hostName);
    bool checkAndhandleSlot (int index);
    void showSlot           (int index);
    void clearSlot          (int index);
    bool slotIndexValid     (int index);

    Tresolv requests[RESOLV_COUNT];

private:
    bool getaddrinfoHasFinished (int index);
    bool isProcessed            (int index);
    bool processSlot            (int index);
};

#endif
