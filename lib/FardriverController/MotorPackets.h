/*

*/

#pragma once

#include <inttypes.h>

typedef struct __attribute__ ((__packed__))
{
    uint8_t start;
    uint8_t id;
} packet_start_t;

typedef struct __attribute__ ((__packed__))
{
    uint16_t crc;
} packet_end_t;

typedef struct __attribute__ ((__packed__))
{
    packet_start_t _start;
    uint8_t D0;
    uint8_t D1;
    uint8_t D2;
    uint8_t D3;
    uint8_t D4;
    uint8_t D5;
    uint8_t D6;
    uint8_t D7;
    uint8_t D8;
    uint8_t D9;
    uint8_t D10;
    uint8_t D11;
    packet_end_t _end;
} packet_raw_t;

typedef struct __attribute__ ((__packed__))
{
    packet_start_t _start;
    uint8_t MTPAAngle;
    uint8_t Hall;
    uint8_t Gear : 4;
    uint8_t Roll : 4;
    uint8_t Follow;
    uint16_t RPM;
    uint16_t Errors;
    uint16_t iqout;
    uint16_t idout;
    packet_end_t _end;
} packet_0_t;

typedef struct __attribute__ ((__packed__))
{
    packet_start_t _start;
    uint16_t Voltage;
    uint16_t Current;
    uint8_t ModRatio;
    uint8_t WorkStat;
    uint16_t iqin;
    uint16_t idin;
    uint16_t Trottle;
    packet_end_t _end;
} packet_1_t;





//char (*__qqq)[sizeof( packet_1_t )] = 1;