#include <GenericTypeDefs.h> // BYTE, WORD
#include <string.h> // memcpy()
#include <sys/endian.h>
#include "configs/HWP PIC32_ETH_SK_ETH795.h" // LED1_IO
#include "dp83640cfg1588.h"  // PTPClockStepAdjustment()
#include "dp83640ptp1588.h"


// IEEE 1588 v2 frame definitions.
typedef struct {
  unsigned char  clockIdentity[8];
  unsigned short portNumber;
} PortIdentity;


typedef struct {                                     // Offset  Length (bytes)
  unsigned char     transportSpecificAndMessageType; // 00       1 (2 4-bit fields)
  unsigned char     reservedAndVersionPTP;           // 01       1 (2 4-bit fields)
  unsigned short    messageLength;                   // 02       2
  unsigned char     domainNumber;                    // 04       1
  unsigned char     rxSeconds;                       // 05       1  // reserved - stores LSB of seconds receive timestamp inserted by DP83640
  unsigned char     flags[2];                        // 06       2
  long long         correctionField;                 // 08       8
  unsigned int      rxNanoSeconds;                   // 16       4
  PortIdentity      sourcePortId;                    // 20      10
  unsigned short    sequenceId;                      // 30       2
  unsigned char     control;                         // 32       1
  unsigned char     logMeanMessageInterval;          // 33       1
  unsigned short    txEpoch;                         // 34       2  // used in Sync, Delay_Req, Delay_Resp
  unsigned int      txSeconds;                       // 36       4  // used in Sync, Delay_Req, Delay_Resp
  unsigned int      txNanoSeconds;                   // 40       4  // used in Sync, Delay_Req, Delay_Resp
  PortIdentity      requestingPortId;                // 44       10 // used in Delay_Resp
} Frame1588;


#define PTP_MSG_SYNC           0
#define PTP_MSG_DELAY_REQ      1
#define PTP_MSG_PDELAY_RESP    3
#define PTP_MSG_DELAY_RESP     9
#define PTP_MSG_DELAY_RESP_CTL 3

#define RX_TIMESTAMP_ADJUSTMENT (215)    // ns for 100Base-TX as per PHYTER Dev Guide
#define BILION (0x3B9ACA00ll) // long long not one one at the end!
#define LOWER30BITS (0x3FFFFFFF)
#define FRAME_TYPE_1588 (0x88F7)
//#define ABS(x) (((x)>0)?(x):(-(x)))


// copied from mac.h to avoid the include dependencies of mac.h and StackTsk.h
typedef struct __attribute__((__packed__)){ BYTE v[6]; } MAC_ADDR;
WORD MACGetArray(BYTE *val, WORD len);
void MACPutArray(BYTE *val, WORD len);
void MACPutHeader(MAC_ADDR *remote, WORD type, WORD dataLen);
BOOL MACIsTxReady(void);
BOOL MACIsLinked();
void MACFlush(void);

// local prototypes
void PTPSendDelayReq();
void PTPSendDelayResp(long long timeStamp);
BOOL PTPSendFrame();

static Frame1588 inFrame;
static Frame1588 outFrame;
static MAC_ADDR remote_mac_addr = {{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}}; //broadcast

//******************* Time Conversion *********************************
long long TimeFuse(unsigned int seconds, unsigned int ns){
//    long long lsec, lns, ret;
//    lsec = seconds;
//    lns = ns;

//    ret = lsec*BILION;
//    ret+= lns;
//    return ret;
    return ((long long)seconds)*BILION + ((long long)ns);
}

int TimeSeconds(long long time){
//    long long ret;
//    ret = time/BILION;
//    int retint;
//    retint = ret;
//    return retint;
    return ((long long)time)/BILION;
}

int TimeNanoSeconds(long long time){
//    long long ret;
//    ret = time%BILION;
//    int retint;
//    retint = ret;
//    return retint;
    return ((long long)time) % BILION;
}


//******************* Synchronization *********************************
static long long time1, time2, avgRndTrip;

void SyncAddT1T2(long long T1, long long T2){
    time1 = T1;
    time2 = T2;
}

void SyncAddT3T4(long long time3, long long time4){
    long long lastRoundTrip, deltaT;
//TODO: incorporate RX_TIMESTAMP_ADJUSTMENT into calculations of t4 and t2???

//    long long dt41,dt32,dt21,dt43;
//    dt41 = (time4-time1);
//    dt32 = (time3-time2);
//    dt21 = (time2-time1);
//    dt43 = (time4-time3);

//    if(dt41<0 || dt32<0){
//        return;
//    }

    lastRoundTrip = (time4-time1)-(time3-time2);
    if(avgRndTrip){
//      avgRndTrip  = (avgRndTrip*31ll + lastRoundTrip ) / 32ll;
      avgRndTrip  = (avgRndTrip*7ll + lastRoundTrip ) / 8ll;
      deltaT = time4 - time3 - avgRndTrip/2 +16; // +16 for the addition adjustment time
      PTPClockStepAdjustment(TimeSeconds(deltaT), TimeNanoSeconds(deltaT));
//      char msg[128];
//      sprintf(msg,"clock adjustment %us %uns", TimeSeconds(deltaT), TimeNanoSeconds(deltaT));
//      Debug(msg);
    } else {
      avgRndTrip = lastRoundTrip;
      deltaT = time4+avgRndTrip/2ll;
      PTPClockSet( TimeSeconds(deltaT), TimeNanoSeconds(deltaT) );
    }
}


//**********************************************************************
// implementation will change if timestamp is appended
long long GetPhyInsertedTime(Frame1588* frame){
    unsigned int seconds, nanoSeconds;
    PTPClockRead(&seconds, &nanoSeconds); // need upper 3 bytes of seconds

    if( (seconds&0xFF) < frame->rxSeconds){ // second byte of seconds was incremented after timestamp was inserted
        seconds = htonl(ntohl(seconds)-256); // decrement
    }
    
    seconds = (seconds & 0xFFFFFF00) | frame->rxSeconds;
    nanoSeconds = ntohl(frame->rxNanoSeconds) & LOWER30BITS;

    long long ret;
    ret = TimeFuse(seconds, nanoSeconds);
    return  ret;
}


void PTPEthInit(){
    // make slave wait for Sync in PTPSend()
    inFrame.transportSpecificAndMessageType = PTP_MSG_DELAY_RESP;
    outFrame.reservedAndVersionPTP = 2;
    outFrame.messageLength = htons((unsigned short)sizeof(Frame1588));
}


void PTPProcess(MAC_ADDR* remote){
    int readBytes;
    readBytes = MACGetArray((BYTE*)&inFrame, sizeof(Frame1588));

    // I am using a switch to test and can't sniff if the actuall mac is used
    //memcpy(&remote_mac_addr, remote, sizeof(MAC_ADDR)); // save remote MAC address

    long long timeStamp;
    timeStamp = GetPhyInsertedTime(&inFrame);
    // Delay_Req Timestamp was disabled and did not record the time
    if(!timeStamp){ return; }
    
    // send Delay_Resp right away
    if(inFrame.transportSpecificAndMessageType == PTP_MSG_DELAY_REQ){
        return PTPSendDelayResp(timeStamp);
    }

    long long remoteTime;
    remoteTime = TimeFuse(ntohl(inFrame.txSeconds), ntohl(inFrame.txNanoSeconds));

    if(inFrame.transportSpecificAndMessageType == PTP_MSG_SYNC){
        SyncAddT1T2(remoteTime,timeStamp);
        PTPSendDelayReq(); // since we got Sync, we are in slave mode
        inFrame.transportSpecificAndMessageType = PTP_MSG_DELAY_RESP; // avoid sending multiple Delay_Req
    } else { // PTP_MSG_DELAY_RESP
        if(inFrame.sequenceId != outFrame.sequenceId){
            return; // response is not for the latest request we sent out
        }
        SyncAddT3T4(timeStamp,remoteTime);
    }
}

void PTPSendSync(){
    static unsigned short nextSequenceId;
    outFrame.transportSpecificAndMessageType = PTP_MSG_SYNC;
    outFrame.control = PTP_MSG_SYNC;
    outFrame.sequenceId = htons(++nextSequenceId);
    outFrame.txSeconds = 0;     // will be inserted by DP83640
    outFrame.txNanoSeconds = 0; // will be inserted by DP83640
    PTPSendFrame();
}

void PTPSendDelayReq(){
    outFrame.transportSpecificAndMessageType = PTP_MSG_DELAY_REQ;
    outFrame.control = PTP_MSG_DELAY_REQ;
    outFrame.sequenceId = inFrame.sequenceId; // has to match for timestamps to be valid
    outFrame.txSeconds = 0;
    outFrame.txNanoSeconds = 0;
    PTPSendFrame();
}

void PTPSendDelayResp(long long timeStamp){
    outFrame.transportSpecificAndMessageType = PTP_MSG_DELAY_RESP;
    outFrame.control = PTP_MSG_DELAY_RESP_CTL;
    outFrame.sequenceId = inFrame.sequenceId; // has to match for timestamp to be valid
    outFrame.txSeconds = htonl(TimeSeconds(timeStamp));
    outFrame.txNanoSeconds = htonl(TimeNanoSeconds(timeStamp));
    PTPSendFrame();
}

// use Debug instead
//void PTPSendPDelayResp(unsigned int seconds, unsigned int nanoSeconds, BOOL master){
//    static unsigned lastFrameSecondsValue;
//    if(seconds == lastFrameSecondsValue){ // debounce (1 event per second)
//        return;
//    }
//    lastFrameSecondsValue = seconds;
//
//    outFrame.transportSpecificAndMessageType = PTP_MSG_PDELAY_RESP;
//    outFrame.control = PTP_MSG_DELAY_RESP_CTL;
//    outFrame.sequenceId = htons(master?0:1);
//    outFrame.txSeconds = htonl(seconds);
//    outFrame.txNanoSeconds = htonl(nanoSeconds);
//    PTPSendFrame();
//}

// send ethernet frame header with type = 0x88F7 plus outFrame
BOOL PTPSendFrame(){
    if(! MACIsLinked() ){ return FALSE; }
    while(!MACIsTxReady()); //TODO: queue the frames?

    // size could change depending on the type of the frame ( TODO )
    MACPutHeader(&remote_mac_addr, FRAME_TYPE_1588,sizeof(outFrame));
    MACPutArray((BYTE*)&outFrame, sizeof(outFrame));
    MACFlush();
    return TRUE;
}

BOOL Debug(char * msg){
    if(! MACIsLinked() ){ return FALSE; }
    while(!MACIsTxReady()); //TODO: queue the frames?

    unsigned int len;
    len = strlen(msg)+1;
    MACPutHeader(&remote_mac_addr, 0x7777, len);
    MACPutArray((BYTE*)msg, len);
    MACFlush();
    return TRUE;
}