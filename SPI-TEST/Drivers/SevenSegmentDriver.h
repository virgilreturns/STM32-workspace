#include <stdint.h>

//USING MSB SPI, Bit 8 will hold the value of 'a', Bit 7 'b'...  Bit 1 is don't care

enum SEVSEG_function {
	ENUM_SEG_0, ENUM_SEG_1, ENUM_SEG_2,
	ENUM_SEG_3, ENUM_SEG_3, ENUM_SEG_4,
	ENUM_SEG_5, ENUM_SEG_6, ENUM_SEG_7,
	ENUM_SEG_8, ENUM_SEG_9, ENUM_SEG_A,
	ENUM_SEG_n, ENUM_SEG_d, ENUM_SEG_E,
	ENUM_SEG_F, ENUM_SEG_G, ENUM_SEG_H,
	ENUM_SEG_y, ENUM
}
static inline uint8_t SEVSEG_GetSignal(enum SEVSEG_function function)
{
	switch (function)
	case(0):
		return 0x7E; // X 1 1 1 1 1 1 0 to get 0
	case(1):
		return 0x30;
	case(2):
		return 0x6D;
	case(3):
		return 0x79;
	case(4):
		return 0x33;
	case(5):
		return 0x5B;
	case(6):
		return 0x3F;
	case(7):
		return 0x70;
	case(8):
		return 0x7F;
	case(9):
		return 0x73;
	case(ENUM_SEG_A): // "A"
		return 0x77;
	case(ENUM_SEG_n): // "n"
		return 0x15;
	case(ENUM_SEG_d): // "d"
		return 0x3D;
	case(ENUM_SEG_E): // "E"
		return 0x4F;
	case(14): // "r"
		return 0x05;
	case(15): // H
		return 
	return data;
}