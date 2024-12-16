#ifndef FONTS_H
#define FONTS_H

const uint8_t Font_Numbers[10][8] = {
    {0x3C, 0x42, 0x42, 0x42, 0x42, 0x42, 0x42, 0x3C}, // 0
    {0x00, 0x04, 0x0C, 0x14, 0x04, 0x04, 0x04, 0x04}, // 1
    {0x3C, 0x42, 0x02, 0x0C, 0x30, 0x40, 0x42, 0x3C}, // 2
    ...};

const uint8_t Font_Lowercase[26][8] = {
    {0x00, 0x00, 0x3C, 0x02, 0x3E, 0x42, 0x3E, 0x00}, // a
    {0x40, 0x40, 0x5C, 0x62, 0x42, 0x62, 0x5C, 0x00}, // b
    ...};

const uint8_t Font_Uppercase[26][8] = {
    {0x00, 0x3C, 0x42, 0x42, 0x7E, 0x42, 0x42, 0x00}, // A
    {0x00, 0x7C, 0x42, 0x42, 0x7C, 0x42, 0x42, 0x7C}, // B
    ...};

#endif