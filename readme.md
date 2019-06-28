This project is an implementation of the Precision Time Protocol (IEEE 1588 v2) for Microchip PIC32MX MCUs.  It is used to synchronize time between two DP83640 PHYs (network cards) connected directly to each other via an ethernet cable.  The project depends on the legacy (2012) Microchip TCP/IP stack.

dp83640ptp1588.c  - my code related to IEEE 1588 v2 (PTP)
dp83640ptp1588.h  - function definitions from dp83640ptp1588.c
epl_regs.h        - National Semiconductor Corporation's DP83640 PHY register definitions
dp83640cfg1588.c  - National Semiconductor Corporation's epl_1588.c (heavily modified)
dp83640cfg1588.h  - function definitions from dp83640cfg1588.c
MainDemo.c        - Part of microchip's Application Library from which everything is called

MainDemo.c is a mess. It contains hooks for all of Microchips's network demo code. Unfortunately this is what I got.  Look for function names starting with PTP*.

Code ?MIGHT? depend on modifications in one of the following headers: "ETHPIC32ExtPhy.h", "configs/HWP PIC32_ETH_SK_ETH795.h", <peripheral/eth.h> I remember some protocol id had to be changed from 8 bit to 16 bit somewhere...
