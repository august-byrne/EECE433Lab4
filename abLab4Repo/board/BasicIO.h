/******************************************************************************************
 * BasicIO.c - is a module with public functions used to send and receive
 * information from a serial port. In this case UART2 configured for the
 * Segger debug USB serial port. K65TWR board.
 * v1.1
 *  Created by: Todd Morton, 10/09/2014
 *  With Contributions by: Jacob Gilbert And Adam Slater
 * v2.1
 *  Created by Todd Morton
 *  Contributions to BIOGetStrg() by Chance Eldridge
 * v2.2
 *  Created by Todd Morton with contribution from Daniel Kowalski, 04/05/2017
 *  Added rate parameter to BIOOpen()
 *
 * v3.1
 *  Created by Todd Morton
 *  Deprecated includes.h. Replaced with MCUType.h
 * v4.1
 *  Created by Todd Morton
 *  Modified for MCUXpresso header file macros
 * v4.2
 *  Created by Todd Morton
 *  Modified to fix bug in BOIGetStrg() so a BS can be the first character pressed.
********************************************************************/
#ifndef BIO_INCL
#define BIO_INCL

/******************************************************************************************
 * Defined UART bit rates
 ******************************************************************************************/
#define BIO_BIT_RATE_9600   0
#define BIO_BIT_RATE_19200  1
#define BIO_BIT_RATE_38400  2
#define BIO_BIT_RATE_57600  3
#define BIO_BIT_RATE_115200 4

/*************************************************************************
* Enumerated type for mode parameter in BIOOutDecWord()
*************************************************************************/

typedef enum {
    BIO_OD_MODE_LZ,
    BIO_OD_MODE_AR,
    BIO_OD_MODE_AL
} BIO_OUTDEC_MODE;


/********************************************************************
* Public Function Prototypes 
********************************************************************/
/********************************************************************
* BIOOpen() - Initialization routine for BasicIO()
* Acceptable rates:
*  BIO_BIT_RATE_9600
*  BIO_BIT_RATE_19200
*  BIO_BIT_RATE_38400
*  BIO_BIT_RATE_57600
*  BIO_BIT_RATE_115200
********************************************************************/
void BIOOpen(INT8U rate);

/********************************************************************
* BIORead() - Checks for a character received
*    return: ASCII character received or 0 if no character received
********************************************************************/
INT8C BIORead(void);     /* Reads received character, 0 if none */

/********************************************************************
* BIOGetChar() - Blocks until character is received
*    return: ASCII character
********************************************************************/
INT8C BIOGetChar(void);  /* Blocks until a character is received */

/********************************************************************
* BIOGetStrg() - Inputs a string and stores it into an array.
*
* Descritpion: A routine that inputs a character string to an array
*              until a carraige return is received or strglen is exceeded.
*              Only printable characters are recognized except carriage
*              return and backspace.
*              Backspace erases displayed character and array character.
*              A NULL is always placed at the end of the string.
*              All printable characters are echoed.
* Return value: 0 -> if ended with CR
*               1 -> if strglen exceeded.
* Arguments: *strg is a pointer to the string array
*            strglen is the max string length, includes CR/NULL.
********************************************************************/
INT8U BIOGetStrg(INT8U strglen,INT8C *const strg); /*input a string */

/********************************************************************
* BIOWrite() - Sends an ASCII character
*              Blocks as much as one character time
*    parameter: c is the ASCII character to be sent
********************************************************************/
void BIOWrite(INT8C c);  /* Send an ascii character */

/********************************************************************
* BIOPutStrg() - Sends a C string
*    parameter: strg is a pointer to the string
********************************************************************/
void BIOPutStrg(const INT8C *const strg);

/*******************************************************************************************
* BIOOutDecWord() - Outputs a decimal value of a 32-bit word.
*    Parameters: binword is the word to be sent,
*                field is the maximum number of digits to be shown. Range 1-10. field
*                   starts at cursor.
*                mode determines the behavior of field and binword,
*                3 modes:
*                   1. BIO_OD_MODE_LZ: Shows leading zeros (digits will use the entire field).
*                   2. BIO_OD_MODE_AR: Aligns binword to rightmost field digits.
*                   3. BIO_OD_MODE_AL: Aligns binword to leftmost field digits.
*    Examples:
*    binword = 123, field = 5, mode = BIO_OD_MODE_LZ, Result: 00123
*    binword = 123, field = 5, mode = BIO_OD_MODE_AR, Result: __123 (_'s are spaces)
*    binword = 123, field = 5, mode = BIO_OD_MODE_AL, Result: 123__
*    binword = 123, field = 2, mode = BIO_OD_MODE_LZ, Result: --    (binword exceeds field)

*******************************************************************************************/
void BIOOutDecWord (INT32U binword, INT8U field, BIO_OUTDEC_MODE mode);

/********************************************************************
* BIOOutCRLF() - Outputs a carriage return and line feed.
*
********************************************************************/
void BIOOutCRLF(void);

/************************************************************************
* BIOOutHexByte() - Output one byte in hex.
* bin is the byte to be sent
*************************************************************************/
void BIOOutHexByte(INT8U bin);

/************************************************************************
* BIOOutHexHWord() - Output 16-bit word in hex.
* bin is the word to be sent
*************************************************************************/
void BIOOutHexHWord(INT16U bin);

/************************************************************************
* BIOOutHexWord() - Output 32-bit word in hex.
* bin is the word to be sent
*************************************************************************/
void BIOOutHexWord(INT32U bin);

/********************************************************************
* BIOHexStrgtoWord() - Converts a string of hex characters to a 32-bit
*                      word until NULL is reached.
* Return value: 0 -> if no error.
*               1 -> if string is too long for word.
*               2 -> if a non-hex character is in the string.
*               3 -> No characters in string. Started with NULL.
* Arguments: *strg is a pointer to the string array
*            *bin is the word that will hold the converted string.
********************************************************************/
INT8U BIOHexStrgtoWord(INT8C *const strg,INT32U *bin);
/*******************************************************************/
#endif
