/**
 * @file bitmaskros.h
 * @brief sets offset
 * @copyright 2008 Jean-Philippe Meuret
 **/


 
#pragma once

// Number of bits for 1 unsigned char
#define nBitsPerUchar          (sizeof(unsigned char) * 8)

// Number of unsigned chars to contain a given number of bits
#define nUcharsForNBits(nBits) ((((nBits)-1)/nBitsPerUchar)+1)

// Index=Offset of given bit in 1 unsigned char
#define bitOffsetInUchar(bit)  ((bit)%nBitsPerUchar)

// Index=Offset of the unsigned char associated to the bit
//   at the given index=offset
#define ucharIndexForBit(bit)  ((bit)/nBitsPerUchar)

// Value of an unsigned char with bit set at given index=offset
#define ucharValueForBit(bit)  (((unsigned char)(1)) << bitOffsetInUchar(bit))

// Test the bit with given index=offset in an unsigned char array
#define testBit(bit, array)    ((array[ucharIndexForBit(bit)] >> bitOffsetInUchar(bit)) & 1)
