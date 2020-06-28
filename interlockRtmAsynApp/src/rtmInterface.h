#ifndef  _RTM_INTERFACE_H
#define  _RTM_INTERFACE_H

#include <epicsTime.h>
#include <epicsTypes.h>

void registerRtm2llrfHlsAsyn(void *pRtm);
void callRtmProcessing(epicsTimeStamp time, epicsUInt32 timeslot, void *pRtm);

#endif /* _RTM_INTERFACE_H */
