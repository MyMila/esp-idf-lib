//
// Created by Mladen Đurić - Private on 12.2.22..
//

#ifndef EXAMPLE_ENS210_CRC7_H
#define EXAMPLE_ENS210_CRC7_H

#include <stdint.h>
//               7654 3210
// Polynomial 0b 1000 1001 ~ x^7+x^3+x^0
//            0x    8    9
#define CRC7WIDTH  7    // A 7 bits CRC has polynomial of 7th order, which has 8 terms
#define CRC7POLY   0x89 // The 8 coefficients of the polynomial
#define CRC7IVEC   0x7F // Initial vector has all 7 bits high
// Payload data
#define DATA7WIDTH 17
#define DATA7MASK  ((1UL<<DATA7WIDTH)-1) // 0b 0 1111 1111 1111 1111
#define DATA7MSB   (1UL<<(DATA7WIDTH-1)) // 0b 1 0000 0000 0000 0000

// Compute the CRC-7 of 'val' (should only have 17 bits)
// https://en.wikipedia.org/wiki/Cyclic_redundancy_check#Computation
static uint32_t crc7(uint32_t val)
{
    // Setup polynomial
    uint32_t pol = CRC7POLY;
    // Align polynomial with data
    pol = pol << (DATA7WIDTH - CRC7WIDTH - 1);
    // Loop variable (indicates which bit to test, start with highest)
    uint32_t bit = DATA7MSB;
    // Make room for CRC value
    val = val << CRC7WIDTH;
    bit = bit << CRC7WIDTH;
    pol = pol << CRC7WIDTH;
    // Insert initial vector
    val |= CRC7IVEC;
    // Apply division until all bits done
    while (bit & (DATA7MASK << CRC7WIDTH)) {
        if (bit & val) val ^= pol;
        bit >>= 1;
        pol >>= 1;
    }
    return val;
}

#endif //EXAMPLE_ENS210_CRC7_H
