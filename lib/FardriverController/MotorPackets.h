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
    uint16_t _CRC;
    uint8_t D11;
    uint8_t D10;
    uint8_t D9;
    uint8_t D8;
    uint8_t D7;
    uint8_t D6;
    uint8_t D5;
    uint8_t D4;
    uint8_t D3;
    uint8_t D2;
    uint8_t D1;
    uint8_t D0;
    uint8_t _A1;
    uint8_t _A0;
} motor_packet_raw_t;

typedef struct __attribute__ ((__packed__))
{
    uint16_t _CRC;
    uint16_t idout;
    uint16_t iqout;
    motor_error_t ErrorFlags;
    uint16_t RPM;
    uint8_t Follow;
    uint8_t Gear : 4;
    uint8_t Roll : 4;
    uint8_t Hall;
    uint8_t MTPAAngle;
    uint8_t _A1;
    uint8_t _A0;
} motor_packet_0_t;
// делаем статическую проверку, чтобы при правках случайно не пропустить разный размер структур
static_assert(sizeof(motor_packet_raw_t) == sizeof(motor_packet_0_t), "Structures should have the same size!");

typedef struct __attribute__ ((__packed__))
{
    uint16_t _CRC;
    uint16_t Trottle;
    uint16_t idin;
    uint16_t iqin;
    uint8_t WorkStat;
    uint8_t ModRatio;
    uint16_t Current;
    uint16_t Voltage;
    uint8_t _A1;
    uint8_t _A0;
} motor_packet_1_t;
static_assert(sizeof(motor_packet_raw_t) == sizeof(motor_packet_1_t), "Structures should have the same size!");
