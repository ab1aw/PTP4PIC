#ifndef INCLUDE_DP83640PTP1588_H
#define INCLUDE_DP83640PTP1588_H

#include <GenericTypeDefs.h> // BOOL

void PTPEthInit();
void PTPSendSync();
void PTPProcess();
// use it to broadcast timestamps on the wire
void PTPSendPDelayResp(unsigned int seconds, unsigned int nanoSeconds, BOOL master);
BOOL Debug(char * msg);

long long TimeFuse(unsigned int seconds, unsigned int ns);
int TimeSeconds(long long time);
int TimeNanoSeconds(long long time);

#endif