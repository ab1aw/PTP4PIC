#ifndef INCLUDE_DP83640CFG1588_H
#define INCLUDE_DP83640CFG1588_H

#include <GenericTypeDefs.h> // BOOL, UINT32
#include "epl_regs.h"

// P640_ZZZ are defined in epl_regs.h
#define PTPEVT_TRANSMIT_TIMESTAMP_BIT   P640_TXTS_RDY
#define PTPEVT_RECEIVE_TIMESTAMP_BIT    P640_RXTS_RDY
#define PTPEVT_TRIGGER_DONE_BIT         P640_TRIG_DONE
#define PTPEVT_EVENT_TIMESTAMP_BIT      P640_EVENT_RDY

void PTPEnable(BOOL enableFlag);
void PTPPulseOutEnable(BOOL enable); // temporary disable/enable
void PTPPulseOutStart(); // periodic trigger
void PTPClockOutEnable(BOOL enable, unsigned char divider);
void PTPClockRead(unsigned int* seconds, unsigned int* nanoseconds);
void PTPClockSet(unsigned int seconds, unsigned int nanoseconds);
void PTPClockStepAdjustment(int seconds, int nanoseconds);

unsigned int PTPCheckForEvents();
BOOL PTPGetEvent(unsigned int *eventBits, unsigned int *riseFlags,
     unsigned int *eventTimeSeconds, unsigned int *eventTimeNanoSeconds, unsigned int *eventsMissed);
void PTPSetEventConfig (UINT event, BOOL eventRiseFlag, BOOL eventFallFlag, BOOL eventSingle, UINT gpioConnection);
BOOL PTPGetTxTimestamp(unsigned int* seconds, unsigned int* nanoSeconds);
BOOL PTPGetRxTimestamp(unsigned int* seconds, unsigned int* nanoSeconds);

void PTPEnableDelayReqTimestampInsertion(BOOL enable);
void PTPEnableSyncTimestampInsertion(BOOL enable_sync);
void PTPConfigRxTimeInsert();

void EnableSynchronousEthernet(BOOL enable);

#endif