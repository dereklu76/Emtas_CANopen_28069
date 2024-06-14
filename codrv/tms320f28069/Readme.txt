This directory contains the driver for the CAN driver
for the Texas Instruments TMS320F28335  and TMS320F28069M

Requirement:
- C/C++ header from TI
	28335 tested: v1.31 located in the example
	28069M - requires controlSuite

Example project is located in:
- slave1_28335
- slave1_28069

LaunchPad-TMS320F28069M
========================
The LaunchPad don't have a quartz crystal oscillator. To support this board without
hardware changes we use the internal clock.
Do not use this clock configuration with CAN controller in production!
	
C99 support
===========

- In case you enable the C99 support of the compiler co_datatype.h miss
  int8_t and uint8_t.

Workaround:
Define your own datatypes. Use the Little endian definitions of co_datatype.h
in your own definition.
Set the Define CO_CUSTOMER_DATATYPES.

Minimum:

#ifdef CO_CUSTOMER_DATATYPES
#include <stdint.h>
typedef uint_least8_t		UNSIGNED8;
typedef uint16_t 			UNSIGNED16;
typedef uint32_t 			UNSIGNED32;
typedef int_least8_t		INTEGER8;
typedef int16_t				INTEGER16;
typedef int32_t				INTEGER32;

typedef	char	 			*VIS_STRING;
typedef	char 				OCTET_STRING;
typedef	unsigned char 		*CO_DOMAIN_PTR;
typedef float				REAL32;
#endif

Note:
Do not change co_datatype.h. This changes will overwrite with the next update.
Use gen_define.h or an header, that is included in gen_define.h.
e.g.:
CANopen Device Designer -> General Settings -> Hardware -> Additional .. defines


