// -------------------------------------------------- //
// This file is autogenerated by pioasm; do not edit! //
// -------------------------------------------------- //

#pragma once

#if !PICO_NO_HARDWARE
#include "hardware/pio.h"
#endif

// --------------- //
// dshot_bidir_600 //
// --------------- //

#define dshot_bidir_600_wrap_target 0
#define dshot_bidir_600_wrap 31

#define dshot_bidir_600_BIT_PERIOD 40

static const uint16_t dshot_bidir_600_program_instructions[] = {
            //     .wrap_target
    0xff81, //  0: set    pindirs, 1             [31]
    0xff01, //  1: set    pins, 1                [31]
    0x80a0, //  2: pull   block                      
    0x6050, //  3: out    y, 16                      
    0x00e6, //  4: jmp    !osre, 6                   
    0x000e, //  5: jmp    14                         
    0x6041, //  6: out    y, 1                       
    0x006b, //  7: jmp    !y, 11                     
    0xfd00, //  8: set    pins, 0                [29]
    0xe501, //  9: set    pins, 1                [5] 
    0x0004, // 10: jmp    4                          
    0xee00, // 11: set    pins, 0                [14]
    0xf401, // 12: set    pins, 1                [20]
    0x0004, // 13: jmp    4                          
    0xe055, // 14: set    y, 21                      
    0x1f8f, // 15: jmp    y--, 15                [31]
    0xe080, // 16: set    pindirs, 0                 
    0xe05f, // 17: set    y, 31                      
    0xa142, // 18: nop                           [1] 
    0x0076, // 19: jmp    !y, 22                     
    0x0095, // 20: jmp    y--, 21                    
    0x00d2, // 21: jmp    pin, 18                    
    0xe05e, // 22: set    y, 30                      
    0x4901, // 23: in     pins, 1                [9] 
    0x0097, // 24: jmp    y--, 23                    
    0x4801, // 25: in     pins, 1                [8] 
    0x8020, // 26: push   block                      
    0xe05f, // 27: set    y, 31                      
    0x4a01, // 28: in     pins, 1                [10]
    0x009c, // 29: jmp    y--, 28                    
    0x8020, // 30: push   block                      
    0x0000, // 31: jmp    0                          
            //     .wrap
};

#if !PICO_NO_HARDWARE
static const struct pio_program dshot_bidir_600_program = {
    .instructions = dshot_bidir_600_program_instructions,
    .length = 32,
    .origin = -1,
};

static inline pio_sm_config dshot_bidir_600_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + dshot_bidir_600_wrap_target, offset + dshot_bidir_600_wrap);
    return c;
}

static inline void dshot_bidir_600_program_init(PIO pio, uint sm, uint offset, uint pin) {
    pio_sm_config c = dshot_bidir_600_program_get_default_config(offset);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_in_pins(&c, pin);
    sm_config_set_jmp_pin (&c, pin);
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    gpio_set_pulls(pin, true, false);  // up, down
    sm_config_set_out_shift(&c, false, false, 32);   // auto-pull enabled
    sm_config_set_in_shift(&c, false, false, 32);
    double clocks_per_us = clock_get_hz(clk_sys) / 1000000;
    // 1.667us per bit for dshot600
    sm_config_set_clkdiv(&c, 1.667 / dshot_bidir_600_BIT_PERIOD * clocks_per_us);
    pio_sm_init(pio, sm, offset, &c);
}

#endif
