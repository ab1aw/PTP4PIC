//****************************************************************************
// epl_1588.c
// 
// Copyright (c) 2007-2008 National Semiconductor Corporation.
// All Rights Reserved
// 
// Contains sources for IEEE 1588 related functions.
//
//****************************************************************************

// This file is heavily modified. See comments in original epl_1588.c for function descriptions
#include <peripheral/eth.h>
#include <sys/endian.h>
#include "ETHPIC32ExtPhy.h"
#include "epl_regs.h"
#include "dp83640ptp1588.h"
#include "dp83640cfg1588.h"

// Adj for pin input delay and edge detection time (35ns = 8ns(refclk) * 4 + 3)
#define	PIN_INPUT_DELAY		35

// Note: Bits [7:5] of rIx specify the register page (000-pg0, 001-pg1, 010-pg2, 011-pg3, etc.)
unsigned short EPLReadReg(unsigned int rIx){
  unsigned int phyAdd = EthPhyMIIMAddress();
  EthMIIMWriteStart(PHY_PAGESEL, phyAdd, (rIx>>5)&7); // select proper page
  EthMIIMReadStart(rIx&0x1F, phyAdd);
  return EthMIIMReadResult();
}


// Note: Bits [7:5] of rIx specify the register page (000-pg0, 001-pg1, 010-pg2, 011-pg3, etc.)
void EPLWriteReg(unsigned int rIx, unsigned short wData){
  unsigned int phyAdd = EthPhyMIIMAddress();
  EthMIIMWriteStart(PHY_PAGESEL, phyAdd, (rIx>>5)&7); // select proper page
  while( EthMIIMBusy() );
  EthMIIMWriteStart (rIx&0x1F, phyAdd, wData );
  while( EthMIIMBusy() ); // wait for write to complete
}


void PTPEnable(BOOL enableFlag){
    EPLWriteReg(PHY_PG4_PTP_CTL, enableFlag ? P640_PTP_ENABLE : P640_PTP_DISABLE);
}


void PTPClockRead(unsigned int* seconds, unsigned int* nanoSeconds){
  EPLWriteReg(PHY_PG4_PTP_CTL, P640_PTP_RD_CLK);
  *nanoSeconds  = EPLReadReg(PHY_PG4_PTP_TDR);
  *nanoSeconds |= EPLReadReg(PHY_PG4_PTP_TDR) << 16;
  *seconds  = EPLReadReg(PHY_PG4_PTP_TDR);
  *seconds |= EPLReadReg(PHY_PG4_PTP_TDR) << 16;
}


void PTPClockSet(unsigned int seconds, unsigned int nanoSeconds){
  PTPPulseOutEnable(FALSE);
  EPLWriteReg(PHY_PG4_PTP_TDR, nanoSeconds & 0xFFFF);
  EPLWriteReg(PHY_PG4_PTP_TDR, nanoSeconds >> 16);
  EPLWriteReg(PHY_PG4_PTP_TDR, seconds & 0xFFFF);
  EPLWriteReg(PHY_PG4_PTP_TDR, seconds >> 16);
  EPLWriteReg(PHY_PG4_PTP_CTL, P640_PTP_LOAD_CLK);
  PTPPulseOutStart();
}


void PTPClockStepAdjustment(int seconds, int nanoSeconds){
    PTPPulseOutEnable(FALSE);
// TODO: adjust +16ns for the time it takes to add to the counter
//    nanoSeconds = htonl(ntohl(nanoSeconds)+16); // compensate for 2-cycle addition
//    if(abs(ntohl(nanoSeconds)) > 1000000000){   // did compensation rolled over 1 seconds mark?
//        nanoSeconds = htonl(ntohl(nanoSeconds)%1000000000);
//        seconds = htonl((ntohl(seconds)>0)? ntohl(seconds)+1 : ntohl(seconds)-1);
//    }

//    PTPPulseOutStart();
//    return; // DEBUGGING!

//    nanoSeconds = (nanoSeconds > 0) ? htonl(8) : htonl(-8);

//    if(nanoSeconds &0x80000000){ // disable negative adjustments while DEBUGGING
//        nanoSeconds = -1000;
//        nanoSeconds = htons(nanoSeconds);
//    }

    EPLWriteReg(PHY_PG4_PTP_TDR, nanoSeconds & 0xFFFF);
    EPLWriteReg(PHY_PG4_PTP_TDR, nanoSeconds >> 16);
    EPLWriteReg(PHY_PG4_PTP_TDR, seconds & 0xFFFF);
    EPLWriteReg(PHY_PG4_PTP_TDR, seconds >> 16);
    EPLWriteReg(PHY_PG4_PTP_CTL, P640_PTP_STEP_CLK);
    PTPPulseOutStart();
}


// enable insertion of the timestamp into Delay_Resp message
// not mutually exclusive with PTPEnableSyncTimeStampInsertion()
// registers: PTP_TXCFG0:DR_INSERT,SYNC_1STEP
void PTPEnableDelayReqTimestampInsertion(BOOL enable){
    unsigned short txcfg0;
    txcfg0 = EPLReadReg(PHY_PG5_PTP_TXCFG0) & 0x1FFF;
    EPLWriteReg(PHY_PG5_PTP_TXCFG0, (txcfg0&0x1FFF) | (enable?P640_DR_INSERT:P640_SYNC_1STEP) );
}

// configures Transmit Timestamp Operation
// registers involved in configuration: PTP_TXCFG0, PTP_TXCFG1
void PTPEnableSyncTimestampInsertion(BOOL enable_sync){
    EPLWriteReg(PHY_PG5_PTP_TXCFG0, (enable_sync?P640_SYNC_1STEP:P640_DR_INSERT)|P640_IGNORE_2STEP|P640_CRC_1STEP|P640_TX_L2_EN|P640_TX_TS_EN); //
}

// could use offsetof() macro in stddef.h
#define RX_STAMP_NS_OFFSET  (16<<6) // Frame1588.rxNanoSeconds into bits 11:6
#define RX_STAMP_SEC_OFFSET (5)     // Frame1588.rxSeconds
#define TS_SEC_LEN (0<<12)          // 00-LSB, 01-2LSBs, 10-3LSBs, 11-4bytes

// configure receive timestamp insertion into the packet
// registers PTP_RXCFG0, PTP_RXCFG1, PTP_RXCFG2, PTP_RXCFG3, PTP_RXCFG4
void PTPConfigRxTimeInsert(){
    EPLWriteReg(PHY_PG5_PTP_RXCFG0, P640_RX_L2_EN|P640_RX_TS_EN);
    EPLWriteReg(PHY_PG5_PTP_RXCFG3, P640_ACC_CRC|P640_TS_INSERT);
    EPLWriteReg(PHY_PG5_PTP_RXCFG4, P640_TS_SEC_EN|TS_SEC_LEN|RX_STAMP_NS_OFFSET|RX_STAMP_SEC_OFFSET);
}


/*
# Example Configuration of the Output Clock:
# Nominal Output Clock Frequency = 10 MHz (divide-by-25), 250 MHz Clock Source
# 1. Write 0x8019 to PTP_COC
# This enables the clock output using a divide-by-25 (0x19) from the 250 MHz FCO clock.
        PTP_COC = 0x14 # on page 6
        PAGESEL = 0x13
	extMod.WriteReg(PAGESEL,6)
	extMod.WriteReg(PTP_COC,0x8019)
 */
void PTPClockOutEnable(BOOL enable, unsigned char divider){
    EPLWriteReg(PHY_PG6_PTP_COC, (enable?0x8000:0) |P640_PTP_CLKOUT_SPSEL|divider);
}

#define TRIGGER (0x0)
#define TRIGGER_GPIO_PIN (0x3)
#define PULSE_WIDTH (1000000ll)
void PTPPulseOutStart(){
    EPLWriteReg(PHY_PG5_PTP_TRIG,P640_TRIG_PER|(TRIGGER_GPIO_PIN<<8)|(TRIGGER<<1)|P640_TRIG_WR); //P640_TRIG_IF_LATE|

    unsigned int sec, ns;
    PTPClockRead(&sec,&ns);

    long long time;
    time = TimeFuse(sec,ns);
    // skip cycles but keep the edge aligned
    time = time/PULSE_WIDTH;
    time = time + 2ll;
    time = time * PULSE_WIDTH;
//    time = (2ll+(time/PULSE_WIDTH)) * PULSE_WIDTH;
    sec = TimeSeconds(time);
    ns = TimeNanoSeconds(time);

    EPLWriteReg(PHY_PG4_PTP_CTL, (TRIGGER<<10)|P640_TRIG_DIS|P640_TRIG_LOAD );
    EPLWriteReg(PHY_PG4_PTP_TDR, ns  & 0xFFFF);      // start time ns lo
    EPLWriteReg(PHY_PG4_PTP_TDR, ns  >> 16); // wait for seconds rollover | start time ns hi
    EPLWriteReg(PHY_PG4_PTP_TDR, sec & 0xFFFF);      // start time sec
    EPLWriteReg(PHY_PG4_PTP_TDR, sec >>16); // start time sec
    EPLWriteReg(PHY_PG4_PTP_TDR, PULSE_WIDTH & 0xFFFF ); // pulse width lo
    EPLWriteReg(PHY_PG4_PTP_TDR, PULSE_WIDTH >> 16);      // pulse width hi
    EPLWriteReg(PHY_PG4_PTP_TDR,0); // pulse width 2 lo
    EPLWriteReg(PHY_PG4_PTP_TDR,0); // pulse width 2 hi

    EPLWriteReg(PHY_PG4_PTP_CTL, (TRIGGER<<10)|P640_TRIG_EN );
}

// PHYTER Software Development Guide page 20:
// "If a large adjustment to the PTP time is made, the PPS trigger may need to be rearmed to avoid a long period of inactivity or a PEROD OF RAPID PULSING."
void PTPPulseOutEnable(BOOL enable){
    EPLWriteReg(PHY_PG4_PTP_CTL, (enable?P640_TRIG_EN:P640_TRIG_DIS)|(TRIGGER<<10) );
}



void EnableSynchronousEthernet(BOOL enable){
    unsigned short val;
    val = EPLReadReg(PHY_PG0_PHYCR2);
    EPLWriteReg(PHY_PG0_PHYCR2, (enable?PHYCR2_SYNC_ENET_EN:0) | val );
}


BOOL PTPGetTxTimestamp(unsigned int* seconds, unsigned int* nanoSeconds){
    if( !(P640_TXTS_RDY & EPLReadReg(PHY_PG4_PTP_STS)) ){
        return FALSE; // no events
    }

    *nanoSeconds = EPLReadReg(PHY_PG4_PTP_TXTS);
    unsigned short reg;
    reg = EPLReadReg(PHY_PG4_PTP_TXTS);
    *nanoSeconds |= (reg & 0x3FFF) << 16;

    *seconds = EPLReadReg(PHY_PG4_PTP_TXTS);
    *seconds |= EPLReadReg(PHY_PG4_PTP_TXTS) << 16;
    return TRUE;
}

BOOL PTPGetRxTimestamp(unsigned int* seconds, unsigned int* nanoSeconds){
    if( !(P640_RXTS_RDY & EPLReadReg(PHY_PG4_PTP_STS)) ){
        return FALSE; // no events
    }

    unsigned short reg;

    *nanoSeconds = EPLReadReg(PHY_PG4_PTP_RXTS);
    reg = EPLReadReg(PHY_PG4_PTP_RXTS);
//    *overflowCount = (reg & 0xC000) >> 14;
    *nanoSeconds |= (reg & 0x3FFF) << 16;
    *seconds = EPLReadReg(PHY_PG4_PTP_RXTS);
    *seconds |= EPLReadReg(PHY_PG4_PTP_RXTS) << 16;
//  *sequenceId =
    EPLReadReg(PHY_PG4_PTP_RXTS);
    reg = EPLReadReg(PHY_PG4_PTP_RXTS);
//    *messageType = reg >> 12;
//    *hashValue = reg & 0x0FFF;
    return TRUE;
}


//  Configures the operational behavior of an individual event.
//  event - The event to configure, 0 - 7.
//  eventRiseFlag
//      If set to TRUE, enables detection of rising edge on Event input.
//  eventFallFlag
//      If set to TRUE, enables detection of falling edge on Event input.
//  eventSingle
//      If set to TRUE, enables single event capture operation
//  gpioConnection
//      The GPIO pin the event should be connected to. A value of 0 - 12.
//      If 0 is specified no GPIO pin connection is made.
void PTPSetEventConfig (UINT event, BOOL eventRiseFlag,BOOL eventFallFlag,
     BOOL eventSingle, UINT gpioConnection){

    UINT reg;
    reg = 0;

    reg |= gpioConnection << P640_EVNT_GPIO_SHIFT;
    reg |= event << P640_EVNT_SEL_SHIFT;
    reg |= P640_EVNT_WR;
    EPLWriteReg(PHY_PG5_PTP_EVNT, reg);

    if ( eventRiseFlag) reg |= P640_EVNT_RISE;
    if ( eventFallFlag) reg |= P640_EVNT_FALL;
    if ( eventSingle) reg |= P640_EVNT_SINGLE;
    EPLWriteReg(PHY_PG5_PTP_EVNT, reg);
    return;
}


/*
Enabling Event Monitoring
For example, to enable event monitor 2 to monitor GPIO3 for rising edge detection:
 1. Write 0x0305 to PTP_EVNT to select GPIO3 for event monitor 2.
 2. Write 0x4305 to PTP_EVNT to enable event rise detection on GPIO3.

The process for reading event timestamps is as follows:
1. Read PTP_ESTS to determine if an event timestamp is available.
2. Read from PTP_EDATA: Extended Event Status[15:0] (available only if PTP_ESTS:MULT_EVNT is set to 1)
3. Read from PTP_EDATA: Timestamp_ns[15:0]
4. Read from PTP_EDATA: Timestamp_ns[29:16] (upper 2 bits are always 0)
5. Read from PTP_EDATA: Timestamp_sec[15:0]
6. Read from PTP_EDATA: Timestamp_sec[31:16]
7. Repeat Steps 1-6 until PTP_ESTS = 0
 */



//  Checks to determine if any of the following hardware events are
//  outstanding: Transmit timestamp, receive timestamp, trigger done and event
//  timestamp.
//
//  Returns
//      A bit map of zero or more bits set indicating the types of events that
//      are available from the hardware. The defined bits are:
//          PTPEVT_TRANSMIT_TIMESTAMP_BIT
//          PTPEVT_RECEIVE_TIMESTAMP_BIT
//          PTPEVT_EVENT_TIMESTAMP_BIT
//          PTPEVT_TRIGGER_DONE_BIT
//
//  This must be called prior to retrieving individual events from the
//  hardware. The bit map must be used to determine which "Get" functions
//  need to be called. The applicable "Get" functions are:
//
//      PTPGetTransmitTimestamp
//      PTPGetReceiveTimestamp
//      PTPGetEvent
//      PTPHasTriggerExpired
unsigned int PTPCheckForEvents(){
  return EPLReadReg(PHY_PG4_PTP_STS) & (P640_TXTS_RDY|P640_RXTS_RDY|P640_TRIG_DONE|P640_EVENT_RDY);
}


//  Returns the available asynchronous event timestamps.
//
//  portHandle
//      Handle that represents a port. This is obtained using the EPLEnumPort
//      function.
//  eventBits
//      Set on return to a bit map of events that have occurred. Bit 0 is
//      event 0, etc. 0 or more bits may be set.
//  riseFlags
//      Set on return to a bit map indicating if an event occurred on the
//      rising edge (set to 1), or on the falling edge (set to 0). Bit 0 is
//      is the rising edge flag for event 0, etc.
//  eventTimeSeconds
//      The seconds portion of the IEEE 1588 clock timestamp when this event
//      occurred.
//  eventTimeNanoSeconds
//      The nanosecond portion of the IEEE 1588 clock timestamp when this
//      event occurred. This value will not be larger then 1e9 (1 second).
//  eventsMissed
//      Set on return to indicate the number of events that have been missed
//      prior to this event due to internal event queue overflow. The maximum
//      value is 7.
//
//  Returns
//      TRUE if an event was returned, FALSE otherwise.
//
//  The caller must have previously called PTPCheckForEvents() and determined
//  that the PTPEVT_EVET_TIMESTAMP_BIT bit was set prior to invoking this
//  function. This function does NOT check to determine if an outstanding
//  event is available.
//
//  This function properly tracks and handles events that occur at the same
//  exact time. It also adjusts the timestamp values to compensate for input
//  path and synchronization delays.
//****************************************************************************
BOOL PTPGetEvent(unsigned int *eventBits, unsigned int *riseFlags,
     unsigned int *eventTimeSeconds, unsigned int *eventTimeNanoSeconds, unsigned int *eventsMissed){
    unsigned int reg, exSts, x;
    *eventBits = 0;
    *riseFlags = 0;

    reg = EPLReadReg(PHY_PG4_PTP_ESTS);
    *eventsMissed = (reg & P640_EVNTS_MISSED_MASK) >> P640_EVNTS_MISSED_SHIFT;
    if ( !(reg & P640_EVENT_DET)){ return FALSE; }

    if ( reg & P640_MULT_EVENT){
        exSts = EPLReadReg(PHY_PG4_PTP_EDATA);
        for ( x = 8; x; x--){
            if ( exSts & 0x40)
                *eventBits |= 1 << (x-1);
            if ( exSts & 0x80)
                *riseFlags |= 1 << (x-1);
            exSts <<= 2;
        }
    } else {
        *eventBits |= 1 << ((reg & P640_EVNT_NUM_MASK) >> P640_EVNT_NUM_SHIFT);
        *riseFlags |= ((reg & P640_EVNT_RF) ? 1 : 0) << ((reg & P640_EVNT_NUM_MASK) >> P640_EVNT_NUM_SHIFT);
    }

    *eventTimeNanoSeconds = EPLReadReg(PHY_PG4_PTP_EDATA);
    *eventTimeNanoSeconds |= EPLReadReg(PHY_PG4_PTP_EDATA) << 16;
    *eventTimeSeconds = EPLReadReg(PHY_PG4_PTP_EDATA);
    *eventTimeSeconds |= EPLReadReg(PHY_PG4_PTP_EDATA) << 16;

    // Adj for pin input delay and edge detection time
    if( *eventTimeNanoSeconds < PIN_INPUT_DELAY ){
        if( *eventTimeSeconds > 0 ){
	    *eventTimeSeconds -= 1;
            *eventTimeNanoSeconds += ((unsigned int)1e9 - PIN_INPUT_DELAY);
        } else {
            *eventTimeSeconds = *eventTimeNanoSeconds = 0;
        }
    } else {
        *eventTimeNanoSeconds -= PIN_INPUT_DELAY;
    }
    return TRUE;
}


