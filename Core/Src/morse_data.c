#include "morse_data.h"

/* asciiMorseBinary 2D array. It is accessible from all files that include the "morse_data.h" header file
 * Each member of this array consists of two elements: a binary number, and an int value
 * The binary number is the morse representation of the ascii character, where a dot is 1, and a dash is 111
 * The rest between dots and dashes is 0, and the rest between letters is 000
 * 
 * So for instance, JOE in morse would be: ·---/---/·
 * Converting this to binary, we get: 1011101110111 000 11101110111 000 1
 * We see that the lengths of each letter are: 13-11-1, and these values are the second elements of each member in the array
 * 
 * Later on in the program this binary representation gets converted to a uint8_t array
 * For that reason it is easier to store the binary representation in reverse order 
 * Therefor JOE would become: 00000001000111011101110001110111011101 
 * Because we are repeating the sos message, a rest of seven units (0000000) is appended to the morse binary 
 * 
 * Then lastly, total length of the morse-binary message is the length of each element, plus 2 * three-unit spaces, plus a seven-unit space 
 * This is later used to create a mask of 1's, the same length as the morse-binary messsage. 
 * 
 * In updateClockValue() defined in system.c, and run in TIM4_IRQHandler() in stm32f3xx_it.c, the morse mask is being checked for a zero
 * When a zero is encountered, then the message resets and begins again */
const uint32_t asciiMorseBinary[][2] = {
	ASCII_SPACE,
	ASCII_EXCLAMATION,
	ASCII_QUOTATION,
	ASCII_HASH,
	ASCII_DOLLAR,
	ASCII_PERCENT,
	ASCII_AMPERSAND,
	ASCII_APOSTROPHE,
	ASCII_LPARENTHESIS,
	ASCII_RPARENTHESIS,
	ASCII_ASTERISK,
	ASCII_PLUS,
	ASCII_COMMA,
	ASCII_DASH,
	ASCII_POINT,
	ASCII_SLASH,
	ASCII_0,
	ASCII_1,
	ASCII_2,
	ASCII_3,
	ASCII_4,
	ASCII_5,
	ASCII_6,
	ASCII_7,
	ASCII_8,
	ASCII_9,
	ASCII_COLON,
	ASCII_SEMICOLON,
	ASCII_LESSTHAN,
	ASCII_EQUAL,
	ASCII_GREATERTHAN,
	ASCII_QUESTION,
	ASCII_ATSIGN,
	ASCII_A,
	ASCII_B,
	ASCII_C,
	ASCII_D,
	ASCII_E,
	ASCII_F,
	ASCII_G,
	ASCII_H,
	ASCII_I,
	ASCII_J,
	ASCII_K,
	ASCII_L,
	ASCII_M,
	ASCII_N,
	ASCII_O,
	ASCII_P,
	ASCII_Q,
	ASCII_R,
	ASCII_S,
	ASCII_T,
	ASCII_U,
	ASCII_V,
	ASCII_W,
	ASCII_X,
	ASCII_Y,
	ASCII_Z
};
