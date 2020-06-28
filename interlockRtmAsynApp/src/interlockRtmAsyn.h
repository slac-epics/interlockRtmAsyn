#ifndef _INTERLOCK_RTM_ASYN_H
#define _INTERLOCK_RTM_ASYN_H


#include <string.h>
#include <math.h>
#include <ellLib.h>
#include <epicsMutex.h>
#include <epicsTime.h>
#include <epicsEvent.h>
#include <asynPortDriver.h>

#include "interlockRtmFw.h"

#define PULSEID(time)  ((time).nsec & 0x0001ffff )
#define  SIZE_FASTADC_DATA (0x200 -2)
#define  SIZE_BUFFER_LIST  16
#define  SIZE_FAULT_SNAPSHOT 4


typedef struct {
    uint32_t raw[SIZE_FASTADC_DATA];
    double   data[SIZE_FASTADC_DATA];
} fastADC_data_t;


typedef struct {
    ELLNODE node;
    epicsTimeStamp    time;
    uint32_t          pulseid;
    fastADC_data_t       reflectPower;
    fastADC_data_t       forwardPower;
    fastADC_data_t       beamCurrent;
    fastADC_data_t       beamVoltage;
} fastADCWF_data_t;



class interlockRtmAsynDriver 
    : asynPortDriver {
    public:
        interlockRtmAsynDriver(const char *portName, const char *pathString, const char *named_root = NULL);
        ~interlockRtmAsynDriver();

        asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
        asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);

        void paramSetup(void);
        void getRtmInfo(void);
        void report(int interest);
        void interlockProcess(epicsTimeStamp time, unsigned timeslot);
        void interlockTask(void *p);
        void initRtmWaveforms(void);
        void convertFastADCWaveform(uint32_t adc[], double v[]);
        void reportRtmWaveformBuffer(void);
        void clearRtmFaultHistory(void);
        void takeRtmFaultHistory(void);
        void postRtmFaultHistory(void);
        void getRtmWaveforms(void);
        void setRtmFwdcalRatio(double value);
        void setRtmRefcalRatio(double value);
        void setRtmDivrRatio(double value);
        void setRtmCurrRatio(double value);
        void getRtmThresholdReadout(void);

    private:
        const char*  port;
        const char*  path;
        ELLLIST      faultSnapShot;
        ELLLIST      bufferList;
        epicsMutex*  pRtmLock;
        epicsEvent*  pevent;
        bool         ready;
        unsigned     count;

        interlockRtmFw fw;

        epicsTimeStamp     time;
        unsigned           current_ts;
        unsigned           pulseid;
        uint32_t           rtmFirmwareVersion;
        char               rtmSystemId[32];
        char               rtmSubType[32];
        char               rtmFirmwareDate[32];
        uint32_t           rtmStatus;
        uint32_t           rtmFaultOutStatus;
        uint32_t           rtmAdcLockedStatus;
        uint32_t           rtmRFOffStatus;
        double             rtmFwdcalRatio;
        double             rtmRefcalRatio;
        double             rtmDivrRatio;
        double             rtmCurrRatio;
        uint32_t           rtmThresholdAdcInFwdPower_raw;
        uint32_t           rtmThresholdAdcInRefPower_raw;
        uint32_t           rtmThresholdAdcInBeamCurrent_raw;
        uint32_t           rtmThresholdAdcInBeamVoltage_raw;
        double             rtmThresholdAdcInFwdPower;
        double             rtmThresholdAdcInRefPower;
        double             rtmThresholdAdcInBeamCurrent;
        double             rtmThresholdAdcInBeamVoltage;
 


    protected:

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
        int firstInterlockRtmParam
#define FIRST_INTERLOCKRTM_PARAM     firstInterlockRtmParam
#endif /* ASYN VERSION CHECK under 4.32 */

  
        // rtm interlock module     
        int p_rtmStatus;                  /* asynInt32 */
        int p_rtmClearFault;              /* asynInt32 */
        int p_rtmFaultOutStatus;          /* asynInt32 */
        int p_rtmAdcLockedStatus;         /* asynInt32 */
        int p_rtmRFOffStatus;             /* asynInt32 */
    
        int p_rtmDesiredSled;             /* asynInt32 */
    
        int p_rtmThresholdAdcInFwdPower;    /* asynFloat64 */
        int p_rtmThresholdAdcInRefPower;    /* asynFloat64 */
        int p_rtmThresholdAdcInBeamCurrent; /* asynFloat64 */
        int p_rtmThresholdAdcInBeamVoltage; /* asynFloat64 */

        int p_rtmModThresholdFwdPower;    /* asynFloat64 */    // modulator threshold A
        int p_rtmModThresholdRefPower;    /* asynFloat64 */    // modulator threshold B
        int p_rtmKlyThresholdBeamCurrent; /* asynFloat64 */    // Klystron  threshold A
        int p_rtmKlyThresholdBeamVoltage; /* asynFloat64 */    // Klystron  threshold B
    
        int p_stroeNVThreadsholdFwdPower;      /* asynInt32 */
        int p_storeNVThreadsholdRefPower;      /* asynInt32 */
        int p_storeNVThreadSholdBeamCurrent;   /* asynInt32 */
        int p_storeNVThreadSholdVoltage;       /* asynInt32 */
    
        int p_rtmBeamCurrentWF;           /* asynFloat64Array */
        int p_rtmBeamVoltageWF;           /* asynFloat64Array */
        int p_rtmFwdPowerWF;              /* asynFloat64Array */
        int p_rtmRefPowerWF;              /* asynFloat64Array */
    
        int p_rtmBeamCurrentScaled;       /* asynFloat64Array */
        int p_rtmBeamVoltageScaled;       /* asynFloat64Array */
        int p_rtmFwdPowerScaled;          /* asynFloat64Array */
        int p_rtmRefPowerScaled;          /* asynFloat64Array */
      
        int p_pulseIdBeamCurrent;         /* asynInt32 */
        int p_pulseIdBeamVoltage;         /* asynInt32 */
        int p_pulseIdFwdPower;            /* asynInt32 */
        int p_pulseIdRefPower;            /* asynInt32 */
     
        int p_rtmSetFwdcalRatio;            /* asynFloat64 */
        int p_rtmSetRefcalRatio;            /* asynFloat64 */
        int p_rtmSetDivrRatio;            /* asynFloat64 */
        int p_rtmSetCurrRatio;            /* asynFloat64 */
      
        int p_rtmBeamCurrentHist[SIZE_FAULT_SNAPSHOT];      /* asynFloat64Array */
        int p_rtmBeamVoltageHist[SIZE_FAULT_SNAPSHOT];      /* asynFloat64Array */
        int p_rtmFwdPowerHist[SIZE_FAULT_SNAPSHOT];         /* asynFloat64Arrray */
        int p_rtmRefPowerHist[SIZE_FAULT_SNAPSHOT];         /* asynFloat64Arrat */
    
        int p_pulseIdBeamCurrentHist[SIZE_FAULT_SNAPSHOT];  /* asynInt32 */
        int p_pulseIdBeamVoltageHist[SIZE_FAULT_SNAPSHOT];  /* asynInt32 */
        int p_pulseIdFwdPowerHist[SIZE_FAULT_SNAPSHOT];     /* asynInt32 */
        int p_pulseIdRefPowerHist[SIZE_FAULT_SNAPSHOT];     /* asynInt32 */
    
        int p_rtmSetThresholdFwdPower;       /* asynFloat64 */
        int p_rtmSetThresholdRefPower;       /* asynFloat64 */
        int p_rtmSetThresholdBeamCurrent;    /* asynFloat64 */
        int p_rtmSetThresholdBeamVoltage;    /* asynFloat64 */

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)      
        int lastInterlockRtmParam
#define LAST_INTERLOCKRTM_PARAM        lastInterlockRtmParam
#endif /* asyn version check, under 4.32 */


};

#if (ASYN_VERSION <<8 | ASYN_REVISION) < (4<<8 | 32)
#define NUM_INTERLOCKRTM_DET_PARAMS ((int)(&LAST_INTERLOCKRTM_PARAM - &FIRST_INTERLOCKRTM_PARAM-1))
#endif /* asyn version check, under 4.32 */


#define rtmStatusString                    "rtmStatus"
#define rtmFaultOutStatusString            "rtmFaultOut"
#define rtmAdcLockedStatusString           "rtmAdcLocked"
#define rtmRFOffStatusString               "rtmRFOff"

#define rtmThresholdAdcInFwdPowerString      "FwdPower_AdcIn"
#define rtmThresholdAdcInRefPowerString      "RefPower_AdcIn"
#define rtmThresholdAdcInBeamCurrentString   "BeamCurrent_AdcIn"
#define rtmThresholdAdcInBeamVoltageString   "BeamVoltage_AdcIn"


#define rtmModThresholdFwdPowerString      "modThresholdFwdPower"       // modulator threshold A
#define rtmModThresholdRefPowerString      "modThresholdRefPower"       // modulator threshold B
#define rtmKlyThresholdBeamCurrentString   "klyThresholdBeamCurrent"    // Klystron  threshold A
#define rtmKlyThresholdBeamVoltageString   "klyThresholdBeamVoltage"    // Klystron  threshold B
#define rtmClearFaultString                "rtmClearFault"
#define rtmDesiredSledString               "rtmDesiredSled"
#define rtmBeamVoltageWFString             "rtmBeamVoltageWF"
#define rtmBeamVoltageScaledString         "rtmBeamVoltageScaled"
#define rtmBeamVoltageHistWFString         "rtmBeamVoltageHist%d"
#define rtmBeamCurrentWFString             "rtmBeamCurrentWF"
#define rtmBeamCurrentScaledString         "rtmBeamCurrentScaled"
#define rtmBeamCurrentHistWFString         "rtmBeamCurrentHist%d"
#define rtmFwdPowerWFString                "rtmFwdPowerWF"
#define rtmFwdPowerScaledString            "rtmFwdPowerScaled"
#define rtmFwdPowerHistWFString            "rtmFwdPowerHist%d"
#define rtmRefPowerWFString                "rtmRefPowerWF"
#define rtmRefPowerScaledString            "rtmRefPowerScaled"
#define rtmRefPowerHistWFString            "rtmRefPowerHist%d"

#define rtmSetFwdcalRatioString            "rtmSetFwdcalRatio"
#define rtmSetRefcalRatioString            "rtmSetRefcalRatio"
#define rtmSetDivrRatioString              "rtmSetDivrRatio"
#define rtmSetCurrRatioString              "rtmSetCurrRatio"

#define rtmPulseIdBeamCurrentString        "pId_beamCurrent"
#define rtmPulseIdBeamVoltageString        "pId_beamVoltage"
#define rtmPulseIdFwdPowerString           "pId_fwdPower"
#define rtmPulseIdRefPowerString           "pId_refPower"

#define rtmPulseIdBeamCurrentHistString    "pId_beamCurrentHist%d"
#define rtmPulseIdBeamVoltageHistString    "pId_beamVoltageHist%d"
#define rtmPulseIdFwdPowerHistString       "pId_fwdPowerHist%d"
#define rtmPulseIdRefPowerHistString       "pId_refPowerHist%d"

#define rtmSetThresholdFwdPowerString            "rtmSetFwdPower"
#define rtmSetThresholdRefPowerString            "rtmSetRefPower"
#define rtmSetThresholdBeamCurrentString         "rtmSetBeamCurrent"
#define rtmSetThresholdBeamVoltageString         "rtmSetBeamVoltage"

// RTM interlock 






#endif    /* _INTERLOCK_RTM_ASYN_H */
