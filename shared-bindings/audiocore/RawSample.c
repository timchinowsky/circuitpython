// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2017 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#include <stdint.h>

#include "shared/runtime/context_manager_helpers.h"
#include "py/binary.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "shared-bindings/util.h"
#include "shared-bindings/audiocore/RawSample.h"

//| class RawSample:
//|     """A raw audio sample buffer in memory"""
//|
//|     def __init__(
//|         self, buffer: ReadableBuffer, *, channel_count: int = 1, sample_rate: int = 8000
//|     ) -> None:
//|         """Create a RawSample based on the given buffer of values. If channel_count is more than
//|         1 then each channel's samples should alternate. In other words, for a two channel buffer, the
//|         first sample will be for channel 1, the second sample will be for channel two, the third for
//|         channel 1 and so on.
//|
//|         :param ~circuitpython_typing.ReadableBuffer buffer: A buffer with samples
//|         :param int channel_count: The number of channels in the buffer
//|         :param int sample_rate: The desired playback sample rate
//|
//|         Simple 8ksps 440 Hz sin wave::
//|
//|           import audiocore
//|           import audioio
//|           import board
//|           import array
//|           import time
//|           import math
//|
//|           # Generate one period of sine wav.
//|           length = 8000 // 440
//|           sine_wave = array.array("h", [0] * length)
//|           for i in range(length):
//|               sine_wave[i] = int(math.sin(math.pi * 2 * i / length) * (2 ** 15))
//|
//|           dac = audioio.AudioOut(board.SPEAKER)
//|           sine_wave = audiocore.RawSample(sine_wave)
//|           dac.play(sine_wave, loop=True)
//|           time.sleep(1)
//|           dac.stop()"""
//|         ...
static mp_obj_t audioio_rawsample_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum { ARG_buffer, ARG_channel_count, ARG_sample_rate, ARG_single_buffer };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_buffer, MP_ARG_OBJ | MP_ARG_REQUIRED, {.u_obj = MP_OBJ_NULL } },
        { MP_QSTR_channel_count, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 1 } },
        { MP_QSTR_sample_rate, MP_ARG_INT | MP_ARG_KW_ONLY, {.u_int = 8000} },
        { MP_QSTR_single_buffer, MP_ARG_BOOL | MP_ARG_KW_ONLY, {.u_bool = true} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    audioio_rawsample_obj_t *self = mp_obj_malloc(audioio_rawsample_obj_t, &audioio_rawsample_type);
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(args[ARG_buffer].u_obj, &bufinfo, MP_BUFFER_READ);
    uint8_t bytes_per_sample = 1;
    bool signed_samples = bufinfo.typecode == 'b' || bufinfo.typecode == 'h';
    if (bufinfo.typecode == 'h' || bufinfo.typecode == 'H') {
        bytes_per_sample = 2;
    } else if (bufinfo.typecode != 'b' && bufinfo.typecode != 'B' && bufinfo.typecode != BYTEARRAY_TYPECODE) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("%q must be a bytearray or array of type 'h', 'H', 'b', or 'B'"), MP_QSTR_buffer);
    }
    common_hal_audioio_rawsample_construct(self, ((uint8_t *)bufinfo.buf), bufinfo.len,
        bytes_per_sample, signed_samples, args[ARG_channel_count].u_int,
        args[ARG_sample_rate].u_int, args[ARG_single_buffer].u_bool);

    return MP_OBJ_FROM_PTR(self);
}

//|     def deinit(self) -> None:
//|         """Deinitialises the RawSample and releases any hardware resources for reuse."""
//|         ...
static mp_obj_t audioio_rawsample_deinit(mp_obj_t self_in) {
    audioio_rawsample_obj_t *self = MP_OBJ_TO_PTR(self_in);
    common_hal_audioio_rawsample_deinit(self);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(audioio_rawsample_deinit_obj, audioio_rawsample_deinit);

static void check_for_deinit(audioio_rawsample_obj_t *self) {
    if (common_hal_audioio_rawsample_deinited(self)) {
        raise_deinited_error();
    }
}

//|     def __enter__(self) -> RawSample:
//|         """No-op used by Context Managers."""
//|         ...
//  Provided by context manager helper.

//|     def __exit__(self) -> None:
//|         """Automatically deinitializes the hardware when exiting a context. See
//|         :ref:`lifetime-and-contextmanagers` for more info."""
//|         ...
static mp_obj_t audioio_rawsample_obj___exit__(size_t n_args, const mp_obj_t *args) {
    (void)n_args;
    common_hal_audioio_rawsample_deinit(args[0]);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(audioio_rawsample___exit___obj, 4, 4, audioio_rawsample_obj___exit__);

//|     sample_rate: Optional[int]
//|     """32 bit value that dictates how quickly samples are played in Hertz (cycles per second).
//|     When the sample is looped, this can change the pitch output without changing the underlying
//|     sample. This will not change the sample rate of any active playback. Call ``play`` again to
//|     change it."""
//|
static mp_obj_t audioio_rawsample_obj_get_sample_rate(mp_obj_t self_in) {
    audioio_rawsample_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);
    return MP_OBJ_NEW_SMALL_INT(common_hal_audioio_rawsample_get_sample_rate(self));
}
MP_DEFINE_CONST_FUN_OBJ_1(audioio_rawsample_get_sample_rate_obj, audioio_rawsample_obj_get_sample_rate);

static mp_obj_t audioio_rawsample_obj_set_sample_rate(mp_obj_t self_in, mp_obj_t sample_rate) {
    audioio_rawsample_obj_t *self = MP_OBJ_TO_PTR(self_in);
    check_for_deinit(self);
    common_hal_audioio_rawsample_set_sample_rate(self, mp_obj_get_int(sample_rate));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(audioio_rawsample_set_sample_rate_obj, audioio_rawsample_obj_set_sample_rate);

MP_PROPERTY_GETSET(audioio_rawsample_sample_rate_obj,
    (mp_obj_t)&audioio_rawsample_get_sample_rate_obj,
    (mp_obj_t)&audioio_rawsample_set_sample_rate_obj);

static const mp_rom_map_elem_t audioio_rawsample_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&audioio_rawsample_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR___enter__), MP_ROM_PTR(&default___enter___obj) },
    { MP_ROM_QSTR(MP_QSTR___exit__), MP_ROM_PTR(&audioio_rawsample___exit___obj) },

    // Properties
    { MP_ROM_QSTR(MP_QSTR_sample_rate), MP_ROM_PTR(&audioio_rawsample_sample_rate_obj) },
};
static MP_DEFINE_CONST_DICT(audioio_rawsample_locals_dict, audioio_rawsample_locals_dict_table);

static const audiosample_p_t audioio_rawsample_proto = {
    MP_PROTO_IMPLEMENT(MP_QSTR_protocol_audiosample)
    .sample_rate = (audiosample_sample_rate_fun)common_hal_audioio_rawsample_get_sample_rate,
    .bits_per_sample = (audiosample_bits_per_sample_fun)common_hal_audioio_rawsample_get_bits_per_sample,
    .channel_count = (audiosample_channel_count_fun)common_hal_audioio_rawsample_get_channel_count,
    .reset_buffer = (audiosample_reset_buffer_fun)audioio_rawsample_reset_buffer,
    .get_buffer = (audiosample_get_buffer_fun)audioio_rawsample_get_buffer,
    .get_buffer_structure = (audiosample_get_buffer_structure_fun)audioio_rawsample_get_buffer_structure,
};

MP_DEFINE_CONST_OBJ_TYPE(
    audioio_rawsample_type,
    MP_QSTR_RawSample,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, audioio_rawsample_make_new,
    locals_dict, &audioio_rawsample_locals_dict,
    protocol, &audioio_rawsample_proto
    );
