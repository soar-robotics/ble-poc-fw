#pragma once

#include <stdint.h>

#define INTERVAL_MAX_NUM 7

/* 16 Bit Custom Service UUID */
#define BLE_CURSTOM_UUID16 0x18FF

/* 16 Bit Custom Service Characteristic UUIDs */
#define BLE_CURSTOM_CHR_UUID16_A 0x2AFF
#define BLE_CURSTOM_CHR_UUID16_B 0x2AFF
#define BLE_CURSTOM_CHR_UUID16_C 0x2ACC

typedef struct
{
    struct
    {
        uint8_t supported : 1;
    } flags;

    uint16_t user_data;
    uint16_t interval_buf[INTERVAL_MAX_NUM];
} __attribute__((packed)) esp_ble_cust_svc_t;
