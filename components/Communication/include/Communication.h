#ifndef _H_ORIGIN_FLOW_COMMUNICATION_H
#define _H_ORIGIN_FLOW_COMMUNICATION_H

/*
PC to Wristband:
_________________________________________________________________________________
|               |               |               |               | | |    0x55 |
CMD       |     LENGTH    |     DATA      |     0x0A      | |               | |
|               |               |
---------------------------------------------------------------------------------

Wristband to PC:
_________________________________________________________________________________
|               |               |               |               | | |    0xAA |
TYPE      |     LENGTH    |     DATA      |     0x0A      | |               | |
|               |               |
---------------------------------------------------------------------------------

*/
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif