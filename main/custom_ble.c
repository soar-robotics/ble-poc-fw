
#include <string.h>

#include "esp_ble_conn_mgr.h"
#include "custom_ble.h"

static const esp_ble_conn_character_t nu_lookup_table[] = {
    {"A", BLE_CONN_UUID_TYPE_16, BLE_CONN_GATT_CHR_NOTIFY, {BLE_CURSTOM_CHR_UUID16_A}, NULL},
    {"B", BLE_CONN_UUID_TYPE_16, BLE_CONN_GATT_CHR_READ, {BLE_CURSTOM_CHR_UUID16_B}, NULL},
    {"C", BLE_CONN_UUID_TYPE_16, BLE_CONN_GATT_CHR_WRITE, {BLE_CURSTOM_CHR_UUID16_C}, NULL},
};

static const esp_ble_conn_svc_t svc = {
    .type = BLE_CONN_UUID_TYPE_16,
    .uuid = {
        .uuid16 = BLE_CURSTOM_UUID16,
    },
    .nu_lookup_count = sizeof(nu_lookup_table) / sizeof(nu_lookup_table[0]),
    .nu_lookup = (esp_ble_conn_character_t *)nu_lookup_table};