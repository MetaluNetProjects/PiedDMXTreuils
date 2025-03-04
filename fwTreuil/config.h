#ifndef _CONFIG_H_
#define _CONFIG_H_

#define ANALOG_FILTER 3

#define RAMP_UINCPOW 8 // 1 increment = 1024 milli-increments (mincs) = 1024x1024 micro-increments (uincs)
#define RAMP_VPOW 8
#define RAMP_TO_POS_POW (RAMP_UINCPOW - 4)
#define RAMP_MAXERROR 4 //  

// rotation motor:
#define MOTA_END K3
#define MOTA_ENDLEVEL 0
#define MOTA_A K2
#define MOTA_B K1

/*#define TRANS_LOSW K2
#define TRANS_HISW K3
#define TRANS_SWLEVEL 1*/

//#define SERVO1 K4

//#define PWLED1 MB1
//#define PWLED2 MB2

#endif // _CONFIG_H_

