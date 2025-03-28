
// spp_and_gatt_counter.h generated from ../../example/spp_and_gatt_counter.gatt for BTstack
// it needs to be regenerated when the .gatt file is updated. 

// To generate spp_and_gatt_counter.h:
// ../../tool/compile_gatt.py ../../example/spp_and_gatt_counter.gatt spp_and_gatt_counter.h

// att db format version 1

// binary attribute representation:
// - size in bytes (16), flags(16), handle (16), uuid (16/128), value(...)

#include <stdint.h>

// Reference: https://en.cppreference.com/w/cpp/feature_test
#if __cplusplus >= 200704L
constexpr
#endif
const uint8_t profile_data[] =
{
    // ATT DB Version
    1,

    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18, 
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a, 
    // 0x0003 VALUE CHARACTERISTIC-GAP_DEVICE_NAME - READ -'SPP+LE Counter' SIGSCOPE (0x53, 0x49, 0x47, 0x53, 0x43, 0x4f, 0x50, 0x45, 0x20, 0x20)
    // READ_ANYBODY
    0x16, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x53, 0x50, 0x50, 0x2b, 0x4c, 0x45, 0x20, 0x43, 0x6f, 0x75, 0x6e, 0x74, 0x65, 0x72, 
    // 0x16, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 0x53, 0x50, 0x50, 0x2b, 0x53, 0x49, 0x47, 0x53, 0x43, 0x4f, 0x50, 0x45, 0x20, 0x20, 
    // 0x0004 PRIMARY_SERVICE-GATT_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 0x01, 0x18, 
    // 0x0005 CHARACTERISTIC-GATT_DATABASE_HASH - READ
    0x0d, 0x00, 0x02, 0x00, 0x05, 0x00, 0x03, 0x28, 0x02, 0x06, 0x00, 0x2a, 0x2b, 
    // 0x0006 VALUE CHARACTERISTIC-GATT_DATABASE_HASH - READ -''
    // READ_ANYBODY
    0x18, 0x00, 0x02, 0x00, 0x06, 0x00, 0x2a, 0x2b, 0xb8, 0x21, 0xb9, 0x36, 0x62, 0x3d, 0xec, 0x6c, 0x40, 0xb4, 0x1d, 0x11, 0xe8, 0x6c, 0xfc, 0x1b, 

    // Counter Service
    // 0x0007 PRIMARY_SERVICE-0000FF10-0000-1000-8000-00805F9B34FB
    0x18, 0x00, 0x02, 0x00, 0x07, 0x00, 0x00, 0x28, 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x10, 0xff, 0x00, 0x00, 
    
    // Counter Characteristic, with read, write, write_without_response and notify
    // 0x0008 CHARACTERISTIC-0000FF11-0000-1000-8000-00805F9B34FB - READ | NOTIFY | WRITE | WRITE_WITHOUT_RESPONSE | DYNAMIC
    0x1b, 0x00, 0x02, 0x00, 0x08, 0x00, 0x03, 0x28, 0x1e, 0x09, 0x00, 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x11, 0xff, 0x00, 0x00, 

    // 0x0009 VALUE CHARACTERISTIC-0000FF11-0000-1000-8000-00805F9B34FB - READ | NOTIFY | WRITE | WRITE_WITHOUT_RESPONSE | DYNAMIC
    // READ_ANYBODY, WRITE_ANYBODY
    0x16, 0x00, 0x0e, 0x03, 0x09, 0x00, 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x11, 0xff, 0x00, 0x00, 
    // 0x000a CLIENT_CHARACTERISTIC_CONFIGURATION
    // READ_ANYBODY, WRITE_ANYBODY
    0x0a, 0x00, 0x0e, 0x01, 0x0a, 0x00, 0x02, 0x29, 0x00, 0x00, 
    // END
    0x00, 0x00, 
}; // total size 99 bytes 


//
// list service handle ranges
//
#define ATT_SERVICE_GAP_SERVICE_START_HANDLE 0x0001
#define ATT_SERVICE_GAP_SERVICE_END_HANDLE 0x0003
#define ATT_SERVICE_GAP_SERVICE_01_START_HANDLE 0x0001
#define ATT_SERVICE_GAP_SERVICE_01_END_HANDLE 0x0003
#define ATT_SERVICE_GATT_SERVICE_START_HANDLE 0x0004
#define ATT_SERVICE_GATT_SERVICE_END_HANDLE 0x0006
#define ATT_SERVICE_GATT_SERVICE_01_START_HANDLE 0x0004
#define ATT_SERVICE_GATT_SERVICE_01_END_HANDLE 0x0006
#define ATT_SERVICE_0000FF10_0000_1000_8000_00805F9B34FB_START_HANDLE 0x0007
#define ATT_SERVICE_0000FF10_0000_1000_8000_00805F9B34FB_END_HANDLE 0x000a
#define ATT_SERVICE_0000FF10_0000_1000_8000_00805F9B34FB_01_START_HANDLE 0x0007
#define ATT_SERVICE_0000FF10_0000_1000_8000_00805F9B34FB_01_END_HANDLE 0x000a

//
// list mapping between characteristics and handles
//
#define ATT_CHARACTERISTIC_GAP_DEVICE_NAME_01_VALUE_HANDLE 0x0003
#define ATT_CHARACTERISTIC_GATT_DATABASE_HASH_01_VALUE_HANDLE 0x0006
#define ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE 0x0009
#define ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_CLIENT_CONFIGURATION_HANDLE 0x000a
