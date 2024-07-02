// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2022 Lee Atkinson, MeanStride Technology, Inc.
// SPDX-FileCopyrightText: Copyright (c) 2021 Raspberry Pi (Trading) Ltd.
//
// SPDX-License-Identifier: MIT

#include <stdio.h>
#include "common-hal/analogbufio/BufferedIn.h"
#include "shared-bindings/analogbufio/BufferedIn.h"
#include "shared-bindings/microcontroller/Pin.h"
#include "shared/runtime/interrupt_char.h"
#include "py/runtime.h"
#include "src/rp2_common/hardware_adc/include/hardware/adc.h"
#include "src/rp2_common/hardware_dma/include/hardware/dma.h"
#include "src/common/pico_stdlib/include/pico/stdlib.h"

#include "pico.h"
#include "src/rp2040/hardware_structs/include/hardware/structs/sio.h"
// #include "hardware/structs/padsbank0.h"
#include "src/rp2040/hardware_structs/include/hardware/structs/iobank0.h"
// #include "hardware/irq.h"
#include "src/rp2_common/hardware_clocks/include/hardware/clocks.h"


#include "SEGGER_RTT.h"


#define ADC_FIRST_PIN_NUMBER 26
#define ADC_PIN_COUNT 4

#define ADC_CLOCK_INPUT 48000000
#define ADC_MAX_CLOCK_DIV (1 << (ADC_DIV_INT_MSB - ADC_DIV_INT_LSB + 1))

void common_hal_analogbufio_bufferedin_construct(analogbufio_bufferedin_obj_t *self, const mcu_pin_obj_t *pin, uint32_t sample_rate) {

    SEGGER_RTT_WriteString(0, "bufferedin_construct\r\n");
    SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    // Make sure pin number is in range for ADC
    if (pin->number < ADC_FIRST_PIN_NUMBER || pin->number >= (ADC_FIRST_PIN_NUMBER + ADC_PIN_COUNT)) {
        raise_ValueError_invalid_pins();
    }

    // Validate sample rate here
    sample_rate = (uint32_t)mp_arg_validate_int_range(sample_rate, ADC_CLOCK_INPUT / ADC_MAX_CLOCK_DIV, ADC_CLOCK_INPUT / 96, MP_QSTR_sample_rate);

    // Set pin and channel
    self->pin = pin;
    claim_pin(pin);

    // TODO: find a way to accept ADC4 for temperature
    self->chan = pin->number - ADC_FIRST_PIN_NUMBER;

    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    // TODO: Make sure we share the ADC well. Right now we just assume it is
    // unused.
    adc_init();
    adc_gpio_init(pin->number);
    adc_select_input(self->chan); // chan = pin - 26 ??

    // Divisor of 0 -> full speed. Free-running capture with the divider is
    // equivalent to pressing the ADC_CS_START_ONCE button once per `div + 1`
    // cycles (div not necessarily an integer). Each conversion takes 96
    // cycles, so in general you want a divider of 0 (hold down the button
    // continuously) or > 95 (take samples less frequently than 96 cycle
    // intervals). This is all timed by the 48 MHz ADC clock.
    // sample rate determines divisor, not zero.

    // sample_rate is forced to be >= 1 in shared-bindings
    float clk_div = (float)ADC_CLOCK_INPUT / (float)sample_rate;
    adc_set_clkdiv(clk_div);


    self->dma_chan[3] = dma_claim_unused_channel(true);
    self->dma_chan[2] = dma_claim_unused_channel(true);
    self->dma_chan[1] = dma_claim_unused_channel(true);
    self->dma_chan[0] = dma_claim_unused_channel(true);

    // Field       : CLOCKS_CLK_GPOUT0_CTRL_AUXSRC
// Description : Selects the auxiliary clock source, will glitch when switching
//               0x0 -> clksrc_pll_sys
//               0x1 -> clksrc_gpin0
//               0x2 -> clksrc_gpin1
//               0x3 -> clksrc_pll_usb
//               0x4 -> rosc_clksrc
//               0x5 -> xosc_clksrc
//               0x6 -> clk_sys
//               0x7 -> clk_usb
//               0x8 -> clk_adc
//               0x9 -> clk_rtc
//               0xa -> clk_ref
    clock_gpio_init(24, 3, 6 * 4800); // send clksrc_pll_usb/6*4800 to GPIO 24
//    clock_gpio_init(24, 8, 6*4800); // send clk_adc/6*4800 to GPIO 24
    clock_gpio_init(25, 6, 6 * 12500); // send clk_sys/6*12500 to GPIO 24

    // Channel 0 transfers data from ADC to buffer

    self->cfg[0] = dma_channel_get_default_config(self->dma_chan[0]);
    // Read from constant address
    channel_config_set_read_increment(&(self->cfg[0]), false);
    // Write to incrementing addresses
    channel_config_set_write_increment(&(self->cfg[0]), true);
    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&(self->cfg[0]), DREQ_ADC);
    // Chain to toggle channel
    channel_config_set_chain_to(&(self->cfg[0]), self->dma_chan[1]);

    // Channel 1 resets channel 0 write address

    self->cfg[1] = dma_channel_get_default_config(self->dma_chan[1]);
    // Read from incrementing address
    channel_config_set_read_increment(&(self->cfg[1]), true);
    // Write to constant address (data dma write address register)
    channel_config_set_write_increment(&(self->cfg[1]), false);
    // Writing to 32-bit register
    channel_config_set_transfer_data_size(&(self->cfg[1]), DMA_SIZE_32);
    // Run as fast as possible
    channel_config_set_dreq(&(self->cfg[1]), 0x3F);
    // set ring to read one 32-bit value (the starting write address) over and over
    channel_config_set_ring(&(self->cfg[1]), false, 2); // ring is 1<<2 = 4 bytes
    // Chain to adc channel
    channel_config_set_chain_to(&(self->cfg[1]), self->dma_chan[3]);


// Channel 2 toggles a GPIO so we can time DMA

    self->cfg[2] = dma_channel_get_default_config(self->dma_chan[2]);
    // Read from incrementing address
    channel_config_set_read_increment(&(self->cfg[2]), true);
    // Write to constant address
    channel_config_set_write_increment(&(self->cfg[2]), false);
    // Writing to 32-bit register
    channel_config_set_transfer_data_size(&(self->cfg[2]), DMA_SIZE_32);
    // Run as fast as possible
    channel_config_set_dreq(&(self->cfg[2]), 0x3F);
    // set ring to read one 32 bit value (the toggle command) over and over
    channel_config_set_ring(&self->cfg[2], false, 2); // ring is 1<<2 = 4 bytes
    // Chain to reconfig channel
    channel_config_set_chain_to(&(self->cfg[2]), self->dma_chan[2]);

    self->cfg[3] = dma_channel_get_default_config(self->dma_chan[3]);
    // Read from incrementing address
    channel_config_set_read_increment(&(self->cfg[3]), true);
    // Write to constant address
    channel_config_set_write_increment(&(self->cfg[3]), false);
    // Writing to 32-bit register
    channel_config_set_transfer_data_size(&(self->cfg[3]), DMA_SIZE_32);
    // Run as fast as possible
    channel_config_set_dreq(&(self->cfg[3]), 0x3F);
    // set ring to read two 32 bit value (the toggle command) over and over
    channel_config_set_ring(&self->cfg[3], false, 3); // ring is 1<<3 = 8 bytes
    // Chain to reconfig channel
    channel_config_set_chain_to(&(self->cfg[3]), self->dma_chan[0]);

    // clear any previous activity
    adc_fifo_drain();
    adc_run(false);
}

bool common_hal_analogbufio_bufferedin_deinited(analogbufio_bufferedin_obj_t *self) {
    return self->pin == NULL;
}

void common_hal_analogbufio_bufferedin_deinit(analogbufio_bufferedin_obj_t *self) {
    if (common_hal_analogbufio_bufferedin_deinited(self)) {
        return;
    }

    // stop DMA
    dma_channel_abort(self->dma_chan[0]);
    dma_channel_abort(self->dma_chan[1]);
    dma_channel_abort(self->dma_chan[2]);

    // Release ADC pin
    reset_pin_number(self->pin->number);
    self->pin = NULL;

    // Release DMA channels
    dma_channel_unclaim(self->dma_chan[0]);
    dma_channel_unclaim(self->dma_chan[1]);
    dma_channel_unclaim(self->dma_chan[2]);
}

uint8_t *active_buffer;
uint32_t gpio_toggle_mask = 1 << 10; // gpio 10
uint32_t gpio_toggle_mask2 = 1 << 11; // gpio 10
volatile uint32_t *toggle_addr;
uint32_t test[16];

uint32_t *GPIO10OVERRIDE = (uint32_t *)0x40014054;

uint32_t gpio_out[2] __attribute__ ((aligned(8))) = {0x00003300, 0x0003200};

uint32_t common_hal_analogbufio_bufferedin_readinto(analogbufio_bufferedin_obj_t *self, uint8_t *buffer, uint32_t len, uint8_t bytes_per_sample) {
    // RP2040 Implementation Detail
    // Fills the supplied buffer with ADC values using DMA transfer.
    // If the buffer is 8-bit, then values are 8-bit shifted and error bit is off.
    // If buffer is 16-bit, then values are 12-bit and error bit is present. We
    // stretch the 12-bit value to 16-bits and truncate the number of valid
    // samples at the first sample with the error bit set.
    // Number of transfers is always the number of samples which is the array
    // byte length divided by the bytes_per_sample.

    SEGGER_RTT_WriteString(0, "readinto\r\n");

    uint dma_size = DMA_SIZE_8;

    bool show_error_bit = false;

    if (bytes_per_sample == 2) {
        dma_size = DMA_SIZE_16;
        show_error_bit = true;
    }

    adc_fifo_setup(
        true,                 // Write each completed conversion to the sample FIFO
        true,                 // Enable DMA data request (DREQ)
        1,                    // DREQ (and IRQ) asserted when at least 1 sample present
        show_error_bit,       // See the ERR bit
        bytes_per_sample == 1 // Shift each sample to 8 bits when pushing to FIFO
        );

    uint32_t sample_count = len / bytes_per_sample;

    channel_config_set_transfer_data_size(&(self->cfg[0]), dma_size);

    active_buffer = buffer;
    toggle_addr = &sio_hw->gpio_togl;



    dma_channel_configure(self->dma_chan[0], &(self->cfg[0]),
        buffer,                                                  // write address
        &adc_hw->fifo,                                           // read address
        sample_count,                                            // transfer count
        true                                                    // start immediately
        );

    dma_channel_configure(self->dma_chan[1], &(self->cfg[1]),
        &dma_hw->ch[self->dma_chan[0]].al2_write_addr_trig,      // write address
        //    &dma_hw->ch[self->dma_chan[0]].write_addr,      // write address
        &active_buffer,                                          // read address
        1,                                                       // transfer count
        false                                                    // don't start yet
        );



    dma_channel_configure(self->dma_chan[2], &(self->cfg[2]),
        test,
        // &sio_hw->gpio_togl,                                      // write address
        &gpio_toggle_mask,                                       // read address
        1,                                                       // transfer count
        true                                                     // start
        );

    dma_channel_configure(self->dma_chan[3], &(self->cfg[3]),
        // test,
        // &sio_hw->gpio_togl,                                      // write address
        // &sio_hw->gpio_out,                                      // write address
        GPIO10OVERRIDE,
        &gpio_out,                                               // read address
        // &gpio_toggle_mask,                                       // read address
        1,                                                       // transfer count
        true                                                     // start
        );

    // Start the ADC
    adc_run(true);

    return 0;
}

uint32_t common_hal_analogbufio_bufferedin_get_sum(analogbufio_bufferedin_obj_t *self) {
    test[0] = gpio_toggle_mask2;
    sio_hw->gpio_togl = gpio_toggle_mask;
    return 42;
}

uint32_t common_hal_analogbufio_bufferedin_get_count(analogbufio_bufferedin_obj_t *self) {
    dma_channel_set_read_addr(self->dma_chan[2], &gpio_toggle_mask, true);
    dma_channel_set_write_addr(self->dma_chan[3], GPIO10OVERRIDE, true);
    return 63;
}

/* toggle GPIO9
>>> import memorymap
>>> io = memorymap.AddressRange(start=0x40014000, length=0x100)
>>> u = int(0x00003300)
>>> d = int(0x00003200)
>>> io[0x4C:0x50] = u.to_bytes(4, "little")
>>> io[0x4C:0x50] = d.to_bytes(4, "little")
>>> io[0x4C:0x50] = u.to_bytes(4, "little")
*/

#define GPIO_SET(v) *((uint32_t *)(0x40014004 + (v * 2))) = 0x00003300
#define GPIO_RESET(v) *((uint32_t *)(0x40014004 + (v * 2))) = 0x00003200

uint32_t *GPIO09OVERRIDE2 = (uint32_t *)0x4001404C;

uint32_t common_hal_analogbufio_bufferedin_start(analogbufio_bufferedin_obj_t *self) {
    *GPIO09OVERRIDE2 = 0x00003300;
    return 84;

}

uint32_t common_hal_analogbufio_bufferedin_stop(analogbufio_bufferedin_obj_t *self) {
    *GPIO09OVERRIDE2 = 0x00003200;
    return 168;
}

uint32_t common_hal_analogbufio_bufferedin_reset(analogbufio_bufferedin_obj_t *self) {
    *GPIO09OVERRIDE2 = 0x00003300;
    *GPIO09OVERRIDE2 = 0x00003200;
    SEGGER_RTT_WriteString(0, "r\r\n");
    *GPIO09OVERRIDE2 = 0x00003300;
    *GPIO09OVERRIDE2 = 0x00003200;
    SEGGER_RTT_WriteString(0, "r\r\n");
    GPIO_SET(9);
    GPIO_RESET(9);
    SEGGER_RTT_WriteString(0, "r\r\n");
    GPIO_SET(9);
    GPIO_RESET(9);
    SEGGER_RTT_WriteString(0, "r\r\n");
    GPIO_SET(9);
    GPIO_RESET(9);
    SEGGER_RTT_WriteString(0, "r\r\n");
    *GPIO09OVERRIDE2 = 0x00003300;
    *GPIO09OVERRIDE2 = 0x00003200;
    SEGGER_RTT_WriteString(0, "r\r\n");
    *GPIO09OVERRIDE2 = 0x00003300;
    *GPIO09OVERRIDE2 = 0x00003200;
    SEGGER_RTT_WriteString(0, "r\r\n");
    *GPIO09OVERRIDE2 = 0x00003300;
    *GPIO09OVERRIDE2 = 0x00003200;
    SEGGER_RTT_WriteString(0, "r\r\n");
    *GPIO09OVERRIDE2 = 0x00003300;
    *GPIO09OVERRIDE2 = 0x00003200;
    SEGGER_RTT_WriteString(0, "r\r\n");
    *GPIO09OVERRIDE2 = 0x00003300;
    *GPIO09OVERRIDE2 = 0x00003200;
    SEGGER_RTT_WriteString(0, "r\r\n");
    return 336;
}

uint32_t common_hal_analogbufio_bufferedin_get_inputs(analogbufio_bufferedin_obj_t *self) {
    return self->inputs;
}

uint32_t timc_inputs;

uint32_t common_hal_analogbufio_bufferedin_set_inputs(analogbufio_bufferedin_obj_t *self, uint32_t inputs) {
    self->inputs = inputs;
    timc_inputs = inputs;
    SEGGER_RTT_printf(0, "inputs: %d\r\n", inputs);
    adc_set_clkdiv((float)inputs);
    // dma_channel_set_read_addr(self->dma_chan[2], &gpio_toggle_mask, true);

    /*
    dma_channel_configure(self->dma_chan[2], &(self->cfg[2]),
        test,
        // &sio_hw->gpio_togl,                                      // write address
        &gpio_toggle_mask,                                       // read address
        1,                                                       // transfer count
        true                                                     // start immediately
        );
    */

    return inputs;
}

/*

    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    uint32_t remaining_transfers = sample_count;
    while (dma_channel_is_busy(self->dma_chan) &&
           !mp_hal_is_interrupted()) {
        RUN_BACKGROUND_TASKS;
    }
    remaining_transfers = dma_channel_hw_addr(self->dma_chan)->transfer_count;

    //  Clean up
    adc_run(false);
    // Stopping early so abort.
    if (dma_channel_is_busy(self->dma_chan)) {
        dma_channel_abort(self->dma_chan);
    }
    adc_fifo_drain();

    size_t captured_count = sample_count - remaining_transfers;
    if (dma_size == DMA_SIZE_16) {
        uint16_t *buf16 = (uint16_t *)buffer;
        for (size_t i = 0; i < captured_count; i++) {
            uint16_t value = buf16[i];
            // Check the error bit and "truncate" the buffer if there is an error.
            if ((value & ADC_FIFO_ERR_BITS) != 0) {
                captured_count = i;
                break;
            }
            // Scale the values to the standard 16 bit range.
            buf16[i] = (value << 4) | (value >> 8);
        }
    }
    return captured_count;
}
*/

/* Test code:

import board
import analogbufio
import array
import audiocore
import audiopwmio
import digitalio

led = digitalio.DigitalInOut(board.D10)
led.direction = digitalio.Direction.OUTPUT

led2 = digitalio.DigitalInOut(board.D9)
led2.direction = digitalio.Direction.OUTPUT

buffer = array.array("H", [0x0000] * 6)

adc = analogbufio.BufferedIn(board.A0, sample_rate=10000)
adc.readinto(buffer)

# pwm = audiopwmio.PWMAudioOut(left_channel=board.D12, right_channel=board.D13, left_shift=4)
pwm = audiopwmio.PWMAudioOut(board.D12, left_shift=4)
sample = audiocore.RawSample(buffer, sample_rate=10000, single_buffer=False)
pwm.play(sample)

>>> sample = audiocore.RawSample(buffer, sample_rate=9999, single_buffer=False); pwm.play(sample)
>>> adc.deinit(); adc = analogbufio.BufferedIn(board.A0, sample_rate=10000); adc.readinto(buffer)

while True:
    adc.inputs += 1
    print(adc.inputs)




    mybuffer[0:16]

adcbuf.deinit()

classsynthio.Biquad(b0: float, b1: float, b2: float, a1: float, a2: float)¶

Gain of 16 amplifier:
synthio.Biquad(b0=16)

    y[n] = (b0/a0)*x[n] + (b1/a0)*x[n-1] + (b2/a0)*x[n-2]
                        - (a1/a0)*y[n-1] - (a2/a0)*y[n-2]



length = 8000 // 440
sine_wave = array.array("H", [0] * length)
for i in range(length):
    sine_wave[i] = int(math.sin(math.pi * 2 * i / length) * (2 ** 15) + 2 ** 15)

dac = audiopwmio.PWMAudioOut(board.SPEAKER)
sine_wave = audiocore.RawSample(sine_wave, sample_rate=8000)
dac.play(sine_wave, loop=True)


// No DMA to SIO pins:  https://forums.raspberrypi.com/viewtopic.php?t=358429


void gpio_set_outover(uint gpio, uint value) {
    check_gpio_param(gpio);
    hw_write_masked(&iobank0_hw->io[gpio].ctrl,
                   value << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB,
                   IO_BANK0_GPIO0_CTRL_OUTOVER_BITS
    );
}


#include "hardware/structs/sio.h"
#include "hardware/structs/padsbank0.h"
#include "hardware/structs/iobank0.h"

#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/claim.h"
 lo = timer_hw->timerawl;


import audiocore
import audiopwmio
import board
import array
import time
import math

# Generate one period of sine wav.
length = 8000 // 440
sine_wave = array.array("H", [0] * length)
for i in range(length):
    sine_wave[i] = int(math.sin(math.pi * 2 * i / length) * (2 ** 15) + 2 ** 15)

dac = audiopwmio.PWMAudioOut(board.D12)
sample = audiocore.RawSample(sine_wave, sample_rate=8000)
dac.play(sample, loop=True)











*/
