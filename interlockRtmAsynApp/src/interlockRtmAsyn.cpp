#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include <string>
#include <sstream>
#include <fstream>

#include <sys/types.h>
#include <sys/stat.h>

#include <cantProceed.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsExit.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <epicsTimer.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsPrint.h>
#include <ellLib.h>
#include <iocsh.h>

#include <yaml-cpp/yaml.h>
#include <yamlLoader.h>

#include <drvSup.h>
#include <epicsExport.h>
#include <registryFunction.h>

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>

#include "interlockRtmAsyn.h"
#include "rtmInterface.h"


static const char *driverName = "interlockRtmAsynDriver";
const char * getDriverName(void) { return driverName; }

static ELLLIST  *pDrvEllList = NULL;

typedef struct {
    ELLNODE     node;
    char        *named_root;
    char        *port;
    char        *regPath;
    interlockRtmAsynDriver   *pInterlockRtmAsyn;
} pDrvList_t;

static void init_drvList(void)
{
    if(!pDrvEllList) {
        pDrvEllList = (ELLLIST *) mallocMustSucceed(sizeof(ELLLIST), "interlockRtmAsyn driver: init_drvList()");
        ellInit(pDrvEllList);
    }

    return;
}

static pDrvList_t *find_drvByPort(const char *port)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
      if(p->port && strlen(p->port) && !strcmp(p->port, port)) break;
      p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}

static pDrvList_t *find_drvByNamedRoot(const char *named_root)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);

    while(p) {
        if(p->named_root && strlen(p->named_root) && !strcmp(p->named_root, named_root)) break;
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return p;
}


void callRtmProcessing(epicsTimeStamp time, epicsUInt32 timeslot, void *pRtm)
{
    interlockRtmAsynDriver *p = ((pDrvList_t *) pRtm)->pInterlockRtmAsyn;
    if(p) p->interlockProcess(time, (unsigned) timeslot);
}


interlockRtmAsynDriver::interlockRtmAsynDriver(const char *portName, const char *pathString, const char *named_root)
    : asynPortDriver(portName,
                     1, /* number of elements of this device */

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
                     NUM_LLRFHLS_DET_PARAMS, /* number of asyn params of be cleared for each device */
#endif  /* asyn version check, under 4.32 */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynDrvUserMask | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask, /* Interface mask */
                     asynInt32Mask | asynFloat64Mask | asynOctetMask | asynEnumMask    | asynInt16ArrayMask | asynInt32ArrayMask | asynFloat64ArrayMask,  /* Interrupt mask */
                     1, /* asynFlags.  This driver does block and it is not multi-device, so flag is 1 */
                     1, /* Autoconnect */
                     0, /* Default priority */
                     0) /* Default stack size*/
{
    Path p_root;
    Path p_interlockRtm;
    port = epicsStrDup(portName);
    path = epicsStrDup(pathString);

    try {
        p_root = (named_root && strlen(named_root))? cpswGetNamedRoot(named_root): cpswGetRoot();
        p_interlockRtm = p_root->findByName(strlen(pathString)? pathString: ".");
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s, file %s, line %d\n", e.getInfo().c_str(), __FILE__, __LINE__);
        throw e;
    }

    fw = IinterlockRtmFw::create(p_interlockRtm);
    pRtmLock = new epicsMutex;
    pevent   = new epicsEvent;
    ready    = false;
    count    = 0;


    paramSetup();
    initRtmWaveforms();
    getRtmInfo();

    callParamCallbacks();

}

interlockRtmAsynDriver::~interlockRtmAsynDriver() {}


void interlockRtmAsynDriver::paramSetup(void)
{
    char param_name[64];

    // RTM Interlock related
    
    createParam(rtmStatusString,  asynParamInt32, &p_rtmStatus);
    createParam(rtmFaultOutStatusString, asynParamInt32, &p_rtmFaultOutStatus);
    createParam(rtmAdcLockedStatusString, asynParamInt32, &p_rtmAdcLockedStatus);
    createParam(rtmRFOffStatusString, asynParamInt32, &p_rtmRFOffStatus);
    
    createParam(rtmClearFaultString, asynParamInt32, &p_rtmClearFault);
    
    createParam(rtmDesiredSledString, asynParamInt32, &p_rtmDesiredSled);
    
    createParam(rtmThresholdAdcInFwdPowerString, asynParamFloat64, &p_rtmThresholdAdcInFwdPower);
    createParam(rtmThresholdAdcInRefPowerString, asynParamFloat64, &p_rtmThresholdAdcInRefPower);
    createParam(rtmThresholdAdcInBeamCurrentString, asynParamFloat64, &p_rtmThresholdAdcInBeamCurrent);
    createParam(rtmThresholdAdcInBeamVoltageString, asynParamFloat64, &p_rtmThresholdAdcInBeamVoltage);
    
    createParam(rtmModThresholdFwdPowerString, asynParamFloat64, &p_rtmModThresholdFwdPower);
    createParam(rtmModThresholdRefPowerString, asynParamFloat64, &p_rtmModThresholdRefPower);
    createParam(rtmKlyThresholdBeamCurrentString, asynParamFloat64, &p_rtmKlyThresholdBeamCurrent);
    createParam(rtmKlyThresholdBeamVoltageString, asynParamFloat64, &p_rtmKlyThresholdBeamVoltage);
    
    createParam(rtmBeamCurrentWFString, asynParamFloat64Array, &p_rtmBeamCurrentWF);
    createParam(rtmBeamVoltageWFString, asynParamFloat64Array, &p_rtmBeamVoltageWF);
    createParam(rtmFwdPowerWFString,    asynParamFloat64Array, &p_rtmFwdPowerWF);
    createParam(rtmRefPowerWFString,    asynParamFloat64Array, &p_rtmRefPowerWF);
    
    createParam(rtmSetFwdcalRatioString, asynParamFloat64, &p_rtmSetFwdcalRatio);
    createParam(rtmSetRefcalRatioString, asynParamFloat64, &p_rtmSetRefcalRatio);
    createParam(rtmSetDivrRatioString,   asynParamFloat64, &p_rtmSetDivrRatio);
    createParam(rtmSetCurrRatioString,   asynParamFloat64, &p_rtmSetCurrRatio);
    
    createParam(rtmBeamCurrentScaledString, asynParamFloat64Array, &p_rtmBeamCurrentScaled);
    createParam(rtmBeamVoltageScaledString, asynParamFloat64Array, &p_rtmBeamVoltageScaled);
    createParam(rtmFwdPowerScaledString,    asynParamFloat64Array, &p_rtmFwdPowerScaled);
    createParam(rtmRefPowerScaledString,    asynParamFloat64Array, &p_rtmRefPowerScaled);
    
    createParam(rtmPulseIdBeamCurrentString, asynParamInt32,   &p_pulseIdBeamCurrent);
    createParam(rtmPulseIdBeamVoltageString, asynParamInt32,   &p_pulseIdBeamVoltage);
    createParam(rtmPulseIdFwdPowerString,    asynParamInt32,   &p_pulseIdFwdPower);
    createParam(rtmPulseIdRefPowerString,    asynParamInt32,   &p_pulseIdRefPower);
    
    for(int i = 0; i < SIZE_FAULT_SNAPSHOT; i++) {
        sprintf(param_name, rtmBeamCurrentHistWFString, i); createParam(param_name, asynParamFloat64Array, &p_rtmBeamCurrentHist[i]);
        sprintf(param_name, rtmBeamVoltageHistWFString, i); createParam(param_name, asynParamFloat64Array, &p_rtmBeamVoltageHist[i]);
        sprintf(param_name, rtmFwdPowerHistWFString,    i); createParam(param_name, asynParamFloat64Array, &p_rtmFwdPowerHist[i]);
        sprintf(param_name, rtmRefPowerHistWFString,    i); createParam(param_name, asynParamFloat64Array, &p_rtmRefPowerHist[i]); 
        
        sprintf(param_name, rtmPulseIdBeamCurrentHistString, i); createParam(param_name, asynParamInt32, &p_pulseIdBeamCurrentHist[i]);
        sprintf(param_name, rtmPulseIdBeamVoltageHistString, i); createParam(param_name, asynParamInt32, &p_pulseIdBeamVoltageHist[i]);
        sprintf(param_name, rtmPulseIdFwdPowerHistString,    i); createParam(param_name, asynParamInt32, &p_pulseIdFwdPowerHist[i]);
        sprintf(param_name, rtmPulseIdRefPowerHistString,    i); createParam(param_name, asynParamInt32, &p_pulseIdRefPowerHist[i]);
    }
    
    
    createParam(rtmSetThresholdFwdPowerString,    asynParamFloat64, &p_rtmSetThresholdFwdPower);
    createParam(rtmSetThresholdRefPowerString,    asynParamFloat64, &p_rtmSetThresholdRefPower);
    createParam(rtmSetThresholdBeamCurrentString, asynParamFloat64, &p_rtmSetThresholdBeamCurrent);
    createParam(rtmSetThresholdBeamVoltageString, asynParamFloat64, &p_rtmSetThresholdBeamVoltage);

}

asynStatus interlockRtmAsynDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char * functionName = "writeInt32";

    /* set integeger in the parameter library */
    status = (asynStatus) setIntegerParam(function, value);


    if(function == p_rtmClearFault && value) {
        fw->cmdRtmClearFault();  // execute clear fault
      
        // do house keeping process here, for the fault clear
      
        fw->getRtmStatus(&rtmStatus);   // read RTM status register
        setIntegerParam(p_rtmStatus, rtmStatus); // update parameter space for the status
      
        clearRtmFaultHistory();
    }
  
    else if(function == p_rtmDesiredSled) {
        if(value) fw->setRtmDesiredSled(true);  // Request SLED Tuned
        else      fw->setRtmDesiredSled(false); // Request SLED Detuned
    }
      
    /* Do callback so higher layer see any changes */
    status = (asynStatus) callParamCallbacks();
    
    if(status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s:%s: status=%d, function=%d, value=%d", 
                      getDriverName(), functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%d\n",
                  getDriverName(), functionName, function, value);

    return status;
}

asynStatus interlockRtmAsynDriver::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int function = pasynUser->reason;
    asynStatus status = asynSuccess;
    const char * functionName = "writeFloat64";

    status = (asynStatus) setDoubleParam(function, value);

    if(function == p_rtmSetFwdcalRatio) {
        setRtmFwdcalRatio(value);
    }
    else if(function == p_rtmSetRefcalRatio) {
        setRtmRefcalRatio(value);
    }
    else if(function == p_rtmSetDivrRatio) {
        setRtmDivrRatio(value);
    }
    else if(function == p_rtmSetCurrRatio) {
        setRtmCurrRatio(value);
    }
  
    /* Potentiometer calibration is empirical, can't go below 10% or above 90% */
    /* Wiper registers are 8 bit, convert from 0-100% to 0-255 */
    /* Also set Non-Volatile register just in case */
    else if(function == p_rtmSetThresholdFwdPower) {
        double potentiometer = (value - 10) / 0.8;
        fw->setRtmModWiperRegA((uint32_t) (potentiometer * 255 / 100) );
        fw->setRtmModNVRegA((uint32_t) (potentiometer * 255 / 100) );
    }
    else if(function == p_rtmSetThresholdRefPower) {
        double potentiometer = (value - 10) / 0.8;
        fw->setRtmModWiperRegB((uint32_t) (potentiometer * 255 / 100) );
        fw->setRtmModNVRegB((uint32_t) (potentiometer * 255 / 100) );
    }
    else if(function == p_rtmSetThresholdBeamCurrent) {
        double potentiometer = (value - 10) / 0.8;
        fw->setRtmKlyWiperRegA((uint32_t) (potentiometer * 255 / 100) );
        fw->setRtmKlyNVRegA((uint32_t) (potentiometer * 255 / 100) );
    }
    else if(function == p_rtmSetThresholdBeamVoltage) {
        double potentiometer = (value - 10) / 0.8;
        fw->setRtmKlyWiperRegB((uint32_t) (potentiometer * 255 / 100) );
        fw->setRtmKlyNVRegB((uint32_t) (potentiometer * 255 / 100) );
    }
  

    /* Do callback so higher layer see any changes */
    status = (asynStatus) callParamCallbacks();
    
    if(status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s:%s: status=%d, function=%d, value=%lf", 
                      getDriverName(), functionName, status, function, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s:%s: function=%d, value=%lf\n",
                  getDriverName(), functionName, function, value);

    return status;
}




void interlockRtmAsynDriver::getRtmInfo(void)
{
    fw->getRtmFirmwareVersion(&rtmFirmwareVersion);
    fw->getRtmSystemId(rtmSystemId);
    fw->getRtmSubType(rtmSubType);
    fw->getRtmFirmwareDate(rtmFirmwareDate);
}


void interlockRtmAsynDriver::report(int interest)
{
    printf("  cpswRtm\n");
    printf("      RTM firmware version: %x\n", rtmFirmwareVersion);
    printf("      RTM system id       : %s\n", rtmSystemId);
    printf("      RTM subtype         : %s\n", rtmSubType);
    printf("      RTM firmware date   : %s\n", rtmFirmwareDate);
    printf("      interlock task cnt  : %u\n", count);
    printf("      current pulse id    : %u\n", pulseid);
    printf("      current timeslot    : %u\n", current_ts);
    char ts_str[80];
    epicsTimeToStrftime(ts_str, sizeof(ts_str), "%Y/%m/%d %H:%M:%S.%09f", &time);
    printf("      timestamp last proc : %s\n", ts_str);
    
    if(interest) reportRtmWaveformBuffer();
}

void interlockRtmAsynDriver::interlockProcess(epicsTimeStamp time, unsigned timeslot)
{
    this->time       = time;
    this->current_ts = timeslot;
    this->pulseid    = PULSEID(time);

    setTimeStamp(&(this->time));
    if(ready) pevent->signal();
}

void interlockRtmAsynDriver::interlockTask(void *p)
{
   uint32_t st;
   rtmStatus = rtmFaultOutStatus = rtmAdcLockedStatus= rtmRFOffStatus = 0xffffffff;
   ready     = true;
   
   
   
    while(1) {
        pevent->wait(); 
        count++;

        fw->cmdRtmRearm();
        
        fw->getRtmStatus(&st);              // read status register
        if(rtmStatus != st) {
            rtmStatus = st;
            setIntegerParam(p_rtmStatus, rtmStatus);   // update parameter space for the status register
        }
        
        fw->getRtmFaultOutStatus(&st);
        if(rtmFaultOutStatus != st) {
            rtmFaultOutStatus = st;
            setIntegerParam(p_rtmFaultOutStatus, rtmFaultOutStatus);
        }
        
        fw->getRtmAdcLockedStatus(&st);
        if(rtmAdcLockedStatus != st) {
            rtmAdcLockedStatus = st;
            setIntegerParam(p_rtmAdcLockedStatus, rtmAdcLockedStatus);
        }
        
        fw->getRtmRFOffStatus(&st);
        if(rtmRFOffStatus != st) {
            rtmRFOffStatus = st;
            setIntegerParam(p_rtmRFOffStatus, rtmRFOffStatus);
        }
        
        
        getRtmWaveforms();
        if(rtmFaultOutStatus) {  // 0 - no fault, 1 - fault
            takeRtmFaultHistory();
        }
        
        
        getRtmThresholdReadout();
              
        callParamCallbacks();
    }
}


void interlockRtmAsynDriver::initRtmWaveforms(void)
{
    ellInit(&faultSnapShot);
    ellInit(&bufferList);
    
    for(int i =0; i < SIZE_BUFFER_LIST; i++) {
        fastADCWF_data_t * t = new fastADCWF_data_t;
        memset(t, 0, sizeof(fastADCWF_data_t));
        ellAdd(&bufferList, &t->node);
    }
}

void interlockRtmAsynDriver::convertFastADCWaveform(uint32_t adc[], double v[])
{
    for(int i= 0; i< (int)SIZE_FASTADC_DATA; i++) {
        *(v+i) = (double) (*(adc+i)) / 2048. -1.;
    }
}


void interlockRtmAsynDriver::reportRtmWaveformBuffer(void)
{
    printf("        rtm waveform buffer: ring buffer %d, fault snapshot buffer %d\n",
            ellCount(&bufferList), ellCount(&faultSnapShot));
}

void interlockRtmAsynDriver::clearRtmFaultHistory(void)
{
    fastADCWF_data_t  *t;
    
    pRtmLock->lock();
    while(ellCount(&faultSnapShot)) {
        t = (fastADCWF_data_t *) ellFirst(&faultSnapShot);
        ellDelete(&faultSnapShot, &t->node);
        //ellAdd(&bufferList, &t->node);
        ellInsert(&bufferList, NULL, &t->node);
    }
    pRtmLock->unlock();
}


void interlockRtmAsynDriver::takeRtmFaultHistory(void)
{
    fastADCWF_data_t   *t;
    
    
    pRtmLock->lock();
    if(ellCount(&faultSnapShot)) {
        pRtmLock->unlock();
        return;
    }
    
    for(int i = 0; i<SIZE_FAULT_SNAPSHOT; i++) {
        t = (fastADCWF_data_t *) ellLast(&bufferList);
        ellDelete(&bufferList, &t->node);
        ellAdd(&faultSnapShot, &t->node);
    }
    pRtmLock->unlock();
    
    postRtmFaultHistory();
}



void interlockRtmAsynDriver::postRtmFaultHistory(void)
{
    fastADCWF_data_t *t;
    int i =0;
    
    t = (fastADCWF_data_t *) ellFirst(&faultSnapShot);
    while(t) {
        doCallbacksFloat64Array(t->beamCurrent.data, SIZE_FASTADC_DATA, p_rtmBeamCurrentHist[i], 0);
        doCallbacksFloat64Array(t->beamVoltage.data, SIZE_FASTADC_DATA, p_rtmBeamVoltageHist[i], 0);
        doCallbacksFloat64Array(t->forwardPower.data, SIZE_FASTADC_DATA, p_rtmFwdPowerHist[i], 0);
        doCallbacksFloat64Array(t->reflectPower.data, SIZE_FASTADC_DATA, p_rtmRefPowerHist[i], 0); 
        
        setIntegerParam(p_pulseIdBeamCurrentHist[i], t->pulseid);
        setIntegerParam(p_pulseIdBeamVoltageHist[i], t->pulseid);
        setIntegerParam(p_pulseIdFwdPowerHist[i], t->pulseid);
        setIntegerParam(p_pulseIdRefPowerHist[i], t->pulseid);    
    
        t = (fastADCWF_data_t *) ellNext(&t->node);
        if(++i > SIZE_FAULT_SNAPSHOT) break;
    }

}



void interlockRtmAsynDriver::getRtmWaveforms(void)
{

#pragma pack(push)
#pragma pack(1)
    struct data_t {
        uint16_t lower;
        uint16_t upper;
    } *p;
#pragma pack(pop)
    
    fastADCWF_data_t *t;
    uint32_t v[SIZE_FASTADC_DATA];
    
    // Waveforms in high-level engineering units
    double scaledBeamVolts[SIZE_FASTADC_DATA];
    double scaledBeamCurrent[SIZE_FASTADC_DATA];
    double scaledFwdPower[SIZE_FASTADC_DATA];
    double scaledRefPower[SIZE_FASTADC_DATA];
    
    pRtmLock->lock();
    t= (fastADCWF_data_t*) ellFirst(&bufferList);
    if(t) ellDelete(&bufferList, &t->node);
    else { pRtmLock->unlock(); return; }
    pRtmLock->unlock();
    
    t->time = time;
    t->pulseid = pulseid;
    
    
    fw->getRtmFastAdcBufferBeamCurrentVoltage(v);
    for(int i = 0; i < SIZE_FASTADC_DATA; i++) {
        p = (struct data_t*)(v+i+2);
        t->beamCurrent.raw[i] = p->upper;
        t->beamVoltage.raw[i] = p->lower;
    
    }
    
    
    fw->getRtmFastAdcBufferForwardReflect(v);
    for(int i =0; i< SIZE_FASTADC_DATA; i++) {
        p = (struct data_t*) (v+i+2);
        t->forwardPower.raw[i] = p->lower;
        t->reflectPower.raw[i] = p->upper;
    }
    
    convertFastADCWaveform(t->beamCurrent.raw,  t->beamCurrent.data);
    convertFastADCWaveform(t->beamVoltage.raw,  t->beamVoltage.data);
    convertFastADCWaveform(t->forwardPower.raw, t->forwardPower.data);
    convertFastADCWaveform(t->reflectPower.raw, t->reflectPower.data);
    
    doCallbacksFloat64Array(t->beamCurrent.data,  SIZE_FASTADC_DATA, p_rtmBeamCurrentWF, 0);
    doCallbacksFloat64Array(t->beamVoltage.data,  SIZE_FASTADC_DATA, p_rtmBeamVoltageWF, 0);
    doCallbacksFloat64Array(t->forwardPower.data, SIZE_FASTADC_DATA, p_rtmFwdPowerWF,    0);
    doCallbacksFloat64Array(t->reflectPower.data, SIZE_FASTADC_DATA, p_rtmRefPowerWF,    0);

    // Convert scaled waveforms to high-level EGU
    for(int i=0; i<SIZE_FASTADC_DATA; i++) {
      // BeamVolts to input voltage = rtmDivrRatio
      // BeamCurrent to input voltage = rtmCurrRatio
      // ForwardPower to input voltage = rtmFwdcalRatio
      // ReflectedPower to input voltage = rtmRefcalRatio
      // RTM voltage divider = 40
      // FwdPower and RefPower are calibrated from documentation
      scaledBeamVolts[i] = t->beamVoltage.data[i] * rtmDivrRatio; // BeamVolts to kV
      scaledBeamCurrent[i] = t->beamCurrent.data[i] * rtmCurrRatio; // BeamCurrent to A
      scaledFwdPower[i] = t->forwardPower.data[i] * rtmFwdcalRatio; // FwdPower to MW
      scaledRefPower[i] = t->reflectPower.data[i] * rtmRefcalRatio; // RefPower to MW
    }
       
    doCallbacksFloat64Array(scaledBeamVolts, SIZE_FASTADC_DATA, p_rtmBeamVoltageScaled, 0);
    doCallbacksFloat64Array(scaledBeamCurrent, SIZE_FASTADC_DATA, p_rtmBeamCurrentScaled, 0);
    doCallbacksFloat64Array(scaledFwdPower, SIZE_FASTADC_DATA, p_rtmFwdPowerScaled, 0);
    doCallbacksFloat64Array(scaledRefPower, SIZE_FASTADC_DATA, p_rtmRefPowerScaled, 0);

    setIntegerParam(p_pulseIdBeamCurrent, t->pulseid);
    setIntegerParam(p_pulseIdBeamVoltage, t->pulseid);
    setIntegerParam(p_pulseIdFwdPower,    t->pulseid);
    setIntegerParam(p_pulseIdRefPower,    t->pulseid);
    
    pRtmLock->lock();
    ellAdd(&bufferList, &t->node);
    pRtmLock->unlock();
       
}


void interlockRtmAsynDriver::setRtmFwdcalRatio(double value)
{
    rtmFwdcalRatio = value;
}

void interlockRtmAsynDriver::setRtmRefcalRatio(double value)
{
    rtmRefcalRatio = value;
}

void interlockRtmAsynDriver::setRtmDivrRatio(double value)
{
    rtmDivrRatio = value;
}

void interlockRtmAsynDriver::setRtmCurrRatio(double value)
{
    rtmCurrRatio = value;
}

void interlockRtmAsynDriver::getRtmThresholdReadout(void)
{
    uint32_t v[4];

    fw->getRtmAdcIn(v);
    
    if(rtmThresholdAdcInBeamCurrent_raw != v[0]) {
        rtmThresholdAdcInBeamCurrent_raw = v[0];
        
        rtmThresholdAdcInBeamCurrent  = (double) rtmThresholdAdcInBeamCurrent_raw / 65536. * 100;
        setDoubleParam(p_rtmThresholdAdcInBeamCurrent, rtmThresholdAdcInBeamCurrent);
        
    }
    
    if(rtmThresholdAdcInBeamVoltage_raw != v[1]) {
        rtmThresholdAdcInBeamVoltage_raw = v[1];
        
        rtmThresholdAdcInBeamVoltage = (double) rtmThresholdAdcInBeamVoltage_raw /65536. * 100;
        setDoubleParam(p_rtmThresholdAdcInBeamVoltage, rtmThresholdAdcInBeamVoltage);
    }
    
    if(rtmThresholdAdcInFwdPower_raw != v[2]) {
        rtmThresholdAdcInFwdPower_raw    = v[2];
        
        rtmThresholdAdcInFwdPower = (double) rtmThresholdAdcInFwdPower_raw / 65536. * 100;
        setDoubleParam(p_rtmThresholdAdcInFwdPower, rtmThresholdAdcInFwdPower);
    }
    
    if(rtmThresholdAdcInRefPower_raw != v[3]) {
        rtmThresholdAdcInRefPower_raw    = v[3];
        
        rtmThresholdAdcInRefPower = (double) rtmThresholdAdcInRefPower_raw / 65536. * 100;
        setDoubleParam(p_rtmThresholdAdcInRefPower, rtmThresholdAdcInRefPower);
    }
    
}




extern "C" {

static int interlockRtmThread(void *p)
{
    ((pDrvList_t *)p)->pInterlockRtmAsyn->interlockTask(p);
    return 0;
}

int interlockRtmAsynDriverConfigure(const char *portName, const char *regPathString, const char *named_root)
{
    init_drvList();

    pDrvList_t *p = find_drvByPort(portName);
    if(p) {
        printf("interlockRtmAsynDriver found that port name (%s) has been used.\n", portName);
        return 0;
    };

    p             = (pDrvList_t *) mallocMustSucceed(sizeof(pDrvList_t), "interlockRtmAsyn driver: interlockRtmAsynDriverConfigure()");
    p->port       = epicsStrDup(portName);
    p->regPath    = (regPathString && strlen(regPathString))? epicsStrDup(regPathString): epicsStrDup(".");
    p->named_root = (named_root && strlen(named_root))?epicsStrDup(named_root): cpswGetRootName();
    p->pInterlockRtmAsyn = (interlockRtmAsynDriver *) NULL;
    p->pInterlockRtmAsyn = new interlockRtmAsynDriver((const char *) p->port, (const char *) p->regPath, (const char *) p->named_root);

    registerRtm2llrfHlsAsyn((void *) p);

    ellAdd(pDrvEllList, &p->node);

    return 0;
}


static const iocshArg initArg0 = {"port name",        iocshArgString};
static const iocshArg initArg1 = {"register path",    iocshArgString};
static const iocshArg initArg2 = {"named_root",       iocshArgString};
static const iocshArg * const initArgs[] = { &initArg0,
                                             &initArg1,
                                             &initArg2 };
static const iocshFuncDef initFuncDef = {"interlockRtmAsynDriverConfigure", 3, initArgs};
static void  initCallFunc(const iocshArgBuf *args)
{
    interlockRtmAsynDriverConfigure(args[0].sval,
                                    (args[1].sval && strlen(args[1].sval))?args[1].sval: NULL,
                                    (args[2].sval && strlen(args[2].sval))?args[2].sval: NULL);
       
}

static void interlockRtmAsynDriverRegister(void)
{
    iocshRegister(&initFuncDef, initCallFunc);
}


epicsExportRegistrar(interlockRtmAsynDriverRegister);



static int interlockRtmAsynDriverReport(int interest);
static int interlockRtmAsynDriverInitialize(void);

static struct drvet interlockRtmAsynDriver = {
    2,
    (DRVSUPFUN) interlockRtmAsynDriverReport,
    (DRVSUPFUN) interlockRtmAsynDriverInitialize
};

epicsExportAddress(drvet, interlockRtmAsynDriver);


static int interlockRtmAsynDriverReport(int interest)
{
    init_drvList();
    printf("Total %d of interlockRtmAsyn driver instance(s) is(are) registered.\n", ellCount(pDrvEllList));

    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    while(p) {
        printf("\tnamed_root       : %s\n", p->named_root);
        printf("\tport name        : %s\n", p->port);
        printf("\tregister path    : %s\n", p->regPath);
        printf("\tinterlockRtmAsyn : %p\n", p->pInterlockRtmAsyn);
        if(p->pInterlockRtmAsyn) p->pInterlockRtmAsyn->report(interest);
        p = (pDrvList_t *) ellNext(&p->node);
    }
    return 0;
}

static int interlockRtmAsynDriverInitialize(void)
{
    init_drvList();
    pDrvList_t *p = (pDrvList_t *) ellFirst(pDrvEllList);
    while(p) {
        char name[80];
        sprintf(name, "intTask_%s", p->port);
        if(p->pInterlockRtmAsyn) epicsThreadCreate(name, epicsThreadPriorityHigh,
                                                   epicsThreadGetStackSize(epicsThreadStackMedium),
                                                   (EPICSTHREADFUNC) interlockRtmThread, (void *) p);
        p = (pDrvList_t *) ellNext(&p->node);
    }

    return 0;
}



} /* extern C */






