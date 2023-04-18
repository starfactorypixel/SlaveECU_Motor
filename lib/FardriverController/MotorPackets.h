/*
    Пакеты контроллера.
*/

#pragma once

#include <inttypes.h>


// Контроллер флудит rx пакетом и чтобы установить связь нужно ответить на него tx пакетом.
static const uint8_t motor_packet_init_rx[] = {0x41, 0x54, 0x2B ,0x50 ,0x41 ,0x53, 0x53, 0x3D ,0x32, 0x39, 0x36, 0x38, 0x38, 0x37, 0x38, 0x31};
static const uint8_t motor_packet_init_tx[] = {0x2B, 0x50, 0x41, 0x53, 0x53, 0x3D, 0x4F, 0x4E, 0x4E, 0x44, 0x4F, 0x4E, 0x4B, 0x45};

// Чтобы запросить данные с контроллера нужно отправить пакет tx и получить пачку (38) rx пакетов.
static const uint8_t motor_packet_request_tx[] = {0xAA, 0x13, 0xEC, 0x07, 0x09, 0x6F, 0x28, 0xD7};

// Далее идут пакеты ответа на вышеотправленный tx пакет.
typedef struct __attribute__ ((__packed__))
{
    uint8_t start;
    uint8_t id;
} _motor_packet_start_t;

typedef struct __attribute__ ((__packed__))
{
    uint16_t crc;
} _motor_packet_end_t;

typedef struct __attribute__ ((__packed__))
{
    _motor_packet_start_t _start;
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
    _motor_packet_end_t _end;
} motor_packet_raw_t;

typedef struct __attribute__ ((__packed__))
{
    _motor_packet_start_t _start;
    uint8_t MTPAAngle;
    uint8_t Hall;
    uint8_t Gear : 4;
    uint8_t Roll : 4;
    uint8_t Follow;
    uint16_t RPM;
    motor_error_t ErrorFlags;
    uint16_t iqout;
    uint16_t idout;
    _motor_packet_end_t _end;
} motor_packet_0_t;
// делаем статическую проверку, чтобы при правках случайно не пропустить разный размер структур
static_assert(sizeof(motor_packet_raw_t) == sizeof(motor_packet_0_t), "Structures should have the same size!");

typedef struct __attribute__ ((__packed__))
{
    _motor_packet_start_t _start;
    uint16_t Voltage;
    uint16_t Current;
    uint8_t ModRatio;
    uint8_t WorkStat;
    uint16_t iqin;
    uint16_t idin;
    uint16_t Trottle;
    _motor_packet_end_t _end;
} motor_packet_1_t;
static_assert(sizeof(motor_packet_raw_t) == sizeof(motor_packet_1_t), "Structures should have the same size!");





//char (*__qqq)[sizeof( packet_1_t )] = 1;