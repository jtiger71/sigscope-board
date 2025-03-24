/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "btstack_run_loop.h"
#include "pico/stdlib.h"
#include "ble/picow_bt_base.h"
#include "adc_dma/adc_fft.h"

int main() {
    stdio_init_all();

    int res = picow_bt_init();
    if (res){
        return -1;
    }

    init_adc_fft();

    btstack_run_loop_execute();
}
