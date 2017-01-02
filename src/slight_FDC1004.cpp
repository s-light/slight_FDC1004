/******************************************************************************

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/

******************************************************************************/
/******************************************************************************
    The MIT License (MIT)

    Copyright (c) 2016 Stefan Krüger

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
******************************************************************************/


// include Core Arduino functionality
#include <Arduino.h>
// include I2C library
#include <Wire.h>

// include own headerfile
#include "slight_FDC1004.h"

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// slight_FDC1004 functions
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


slight_FDC1004::slight_FDC1004() {
    ready = false;

    touch_status = 0;
    touch_status_old = 0;

    timestamp_lastread = 0;
    update_interval = 90;
}


bool slight_FDC1004::begin() {
    if (ready == false) {
        // setup TWI
        Wire.begin();

        // set ready
        // otherwise write and read commands are not executed
        ready = true;

        // reset all registers
        // takes ~300ms
        soft_reset();

        // configuration_load_defaults();
    }
    return ready;
}

void slight_FDC1004::update() {
    sensor_status_update();
}

// void slight_FDC1004::configuration_load_defaults() {
//     // setup 'good' starting configuration
//     // calculate and set auto config values for 3.3V
//     auto_config_calculate_values_and_set(33);
//     // setup auto config to enabled with good defaults
//     auto_config_load_recommend_config();
//     // auto_config_load_recommend_config(&Serial);
// }


// void slight_FDC1004::update_interval_set_autofit() {
//     // TODO(s-light): implement update calculation with help from datasheet
//     // 90ms is fine for default configuration??
//     update_interval = 90;
// }
//
// void slight_FDC1004::update_interval_set(uint32_t interval) {
//     update_interval = interval;
// }
//
// uint32_t slight_FDC1004::update_interval_get() {
//     return update_interval;
// }


void slight_FDC1004::sensor_event_set_callback(
    callback_t callback_function
) {
    callback_sensor_event = callback_function;
}

// private
void slight_FDC1004::sensor_status_update() {
    // check if sensor is present
    if (ready) {
        // poll sensor every 90ms
        // at default configuration
        // this is the cycle time till all sensors are read.
        // uint32_t duration = millis() - timestamp_lastread;
        // if (duration > update_interval) {
        //     timestamp_lastread =  millis();
        //
        //     // get current state
        //     uint16_t touch_status_raw = touch_status_read();
        //     // check for overcurrentflag
        //     if (touch_status_raw & touch_status_overcurrent_flag_mask) {
        //         touch_status_overcurrent_flag = true;
        //     } else  {
        //         touch_status_overcurrent_flag = false;
        //     }
        //     // filter overcurrentflag out
        //     touch_status = touch_status_raw & touch_status_electrode_mask;
        //
        //     // filter for changes
        //     if (touch_status != touch_status_old) {
        //         touch_status_old = touch_status;
        //
        //         // change in touch detected
        //         sensor_event_callback();
        //         // Serial.print(F("touched: "));
        //         // for (size_t i=0; i<8; i++) {
        //         //     if (touch_status & (1 << i)) {
        //         //         Serial.print("1");
        //         //     } else {
        //         //         Serial.print("0");
        //         //     }
        //         // }
        //         // Serial.println();
        //     }  // filter for changes
        // }  // update_interval
    }  // if ready
}


void slight_FDC1004::sensor_event_callback() {
    if (callback_sensor_event) {
        callback_sensor_event(this);
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// register helper
// capitle title numers are referencing to:
// http://www.ti.com/lit/ds/symlink/fdc1004.pdf
// http://www.ti.com/lit/ds/symlink/fdc1004.pdf#page=16&zoom=auto,0,720
// more infos in the header-file (slight_FDC1004.h)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// 8.6 Register Maps
// all registers are 16 Bits!!

// 8.6.1.1 Capacitive Measurement Registers
// read only
// MEAS1_MSB, MEAS1_LSB .. MEAS4_MSB, MEAS4_LSB
// MSB_MEASn
//     [15:0] Most significant 16 bits of Measurement
// LSB_MEASn
//     [15:8] Least significant 8 bits of Measurement
//     [7:0] reserved, always 0
// 24bit registers
// LSBs D[7:0] are always 0x00
// Two's complement format
// result can be calculated with:
//  Capacitance (pf) =
//      (
//          (Twos complement (measurement [23:0]))
//          / 2^19
//      ) + C_offset

uint32_t slight_FDC1004::measurement_read(uint8_t measurement_id) {
    return measurement_read(
        measurement_id_bounded(measurement_id)
    );
}

uint32_t slight_FDC1004::measurement_read(measurement_id_t measurement_id) {
    uint32_t value = 0;
    // read 32bit value in two 16bit steps and combine
    value = read_register16bit(REG_MEAS1_LSB);
    value |= read_register16bit(REG_MEAS1_MSB) << 16;
    _reg_MEASn_VALUE[measurement_id] = value;
    return value;
}

float slight_FDC1004::capacitance_read(uint8_t measurement_id) {
    return capacitance_read(
        measurement_id_bounded(measurement_id)
    );
}

float slight_FDC1004::capacitance_read(measurement_id_t measurement_id) {
    return convert_measurement_to_capacitance(
        measurement_read(measurement_id)
    );
}


uint32_t slight_FDC1004::measurement_get(uint8_t measurement_id) {
    return measurement_get(
        measurement_id_bounded(measurement_id)
    );
}

uint32_t slight_FDC1004::measurement_get(measurement_id_t measurement_id) {
    return _reg_MEASn_VALUE[measurement_id];
}

float slight_FDC1004::capacitance_get(uint8_t measurement_id) {
    return capacitance_get(
        measurement_id_bounded(measurement_id)
    );
}

float slight_FDC1004::capacitance_get(measurement_id_t measurement_id) {
    return convert_measurement_to_capacitance(
        measurement_get(measurement_id)
    );
}


float slight_FDC1004::convert_measurement_to_capacitance(uint32_t measurement_value) {
    float capacitance = 0.0;
    // calculate capacitance from raw value.
    //  Capacitance (pf) =
    //      (
    //          (Twos complement (measurement [23:0]))
    //          / 2^19
    //      ) + C_offset
    uint32_t mask = 0b00000000011111111111111111111111;
    uint32_t mask_twocomp = (1 << 23);
    uint32_t temp = 0;
    temp |= (measurement_value & mask);
    boolean twocomp = (measurement_value & mask_twocomp);
    temp |= twocomp << 31;
    return capacitance;
}


// 8.6.2 Measurement Configuration Registers
// read and write
// default: 0x1C00
// 0b0001110000000000
// 0b0001 1100 0000 0000
// 0b000 111 00000 00000
// --> CAPDAC 0pF
// --> CHB DISABLED
// --> CHA CIN1
// CONF_MEAS1 .. CONF_MEAS4
// MEASn_CONFIG
//     [15:13] CHA Positive input channel capacitive to digital converter
//         b000 CIN1
//         b001 CIN2
//         b010 CIN3
//         b011 CIN4
//     [12:10] CHB Negative  input channel capacitive to digital converter
//         b000 CIN1
//         b001 CIN2
//         b010 CIN3
//         b011 CIN4
//         b100 CAPDAC
//         b111 DISABLED
//     [09:05] CAPDAC Offset Capacitance
//         b00000 0pF (min programmable offset)
//         ...    C_offset = CAPDAC x 3.125pF
//         b11111 96.875pF (max programmable offset)
//     [04:00] RESERVED (read only)
// configuration limitations:
//     ChA != ChB
//     ChA < ChB for differential measurement


// read & write from & to chip
uint16_t slight_FDC1004::measurement_config_read(uint8_t measurement_id) {
    return measurement_config_read(
        measurement_id_bounded(measurement_id)
    );
}

uint16_t slight_FDC1004::measurement_config_read(measurement_id_t measurement_id) {
    uint8_t reg = REG_MEAS1_CONFIG;
    reg += measurement_id;
    _reg_MEASn_CONFIG[measurement_id] = read_register16bit(reg);
    return _reg_MEASn_CONFIG[measurement_id];
}

void slight_FDC1004::measurement_config_write(uint8_t measurement_id) {
    measurement_config_write(
        measurement_id_bounded(measurement_id)
    );
}

void slight_FDC1004::measurement_config_write(measurement_id_t measurement_id) {
    uint8_t reg = REG_MEAS1_CONFIG;
    reg += measurement_id;
    write_register16bit(reg, _reg_MEASn_CONFIG[measurement_id]);
}

// read & write to internal memory
// full register
uint16_t slight_FDC1004::measurement_config_get(uint8_t measurement_id) {
    return measurement_config_get(
        measurement_id_bounded(measurement_id)
    );
}

uint16_t slight_FDC1004::measurement_config_get(measurement_id_t measurement_id) {
    return _reg_MEASn_CONFIG[measurement_id];
}

void slight_FDC1004::measurement_config_set(uint8_t measurement_id, uint16_t value) {
    measurement_config_set(
        measurement_id_bounded(measurement_id),
        value
    );
}

void slight_FDC1004::measurement_config_set(measurement_id_t measurement_id, uint16_t value) {
    _reg_MEASn_CONFIG[measurement_id] = value;
}


// chA
slight_FDC1004::measurement_config_chA_t slight_FDC1004::measurement_config_chA_get(
    uint8_t measurement_id
) {
    return measurement_config_chA_get(
        measurement_id_bounded(measurement_id)
    );
}

slight_FDC1004::measurement_config_chA_t slight_FDC1004::measurement_config_chA_get(
    measurement_id_t measurement_id
) {
    return (measurement_config_chA_t)get_register16bit_part(
        _reg_MEASn_CONFIG[measurement_id],
        measurement_config_chA_mask,
        measurement_config_chA_shift
    );
}

void slight_FDC1004::measurement_config_chA_set(
    uint8_t measurement_id,
    measurement_config_chA_t config
) {
    measurement_config_chA_set(
        measurement_id_bounded(measurement_id),
        config
    );
}

void slight_FDC1004::measurement_config_chA_set(
    measurement_id_t measurement_id,
    measurement_config_chA_t config
) {
    _reg_MEASn_CONFIG[measurement_id] = set_register16bit_part(
        _reg_MEASn_CONFIG[measurement_id],
        measurement_config_chA_mask,
        measurement_config_chA_shift,
        config
    );
}


// chB
slight_FDC1004::measurement_config_chB_t slight_FDC1004::measurement_config_chB_get(
    uint8_t measurement_id
) {
    return measurement_config_chB_get(
        measurement_id_bounded(measurement_id)
    );
}

slight_FDC1004::measurement_config_chB_t slight_FDC1004::measurement_config_chB_get(
    measurement_id_t measurement_id
) {
    return (measurement_config_chB_t)get_register16bit_part(
        _reg_MEASn_CONFIG[measurement_id],
        measurement_config_chB_mask,
        measurement_config_chB_shift
    );
}

void slight_FDC1004::measurement_config_chB_set(
    uint8_t measurement_id,
    measurement_config_chB_t config
) {
    measurement_config_chB_set(
        measurement_id_bounded(measurement_id),
        config
    );
}

void slight_FDC1004::measurement_config_chB_set(
    measurement_id_t measurement_id,
    measurement_config_chB_t config
) {
    _reg_MEASn_CONFIG[measurement_id] = set_register16bit_part(
        _reg_MEASn_CONFIG[measurement_id],
        measurement_config_chB_mask,
        measurement_config_chB_shift,
        config
    );
}


// CAPDAC
uint8_t slight_FDC1004::measurement_config_CAPDAC_get(
    uint8_t measurement_id
) {
    return measurement_config_CAPDAC_get(
        measurement_id_bounded(measurement_id)
    );
}

uint8_t slight_FDC1004::measurement_config_CAPDAC_get(
    measurement_id_t measurement_id
) {
    return get_register16bit_part(
        _reg_MEASn_CONFIG[measurement_id],
        measurement_config_CAPDAC_mask,
        measurement_config_CAPDAC_shift
    );
}

float slight_FDC1004::measurement_config_CAPDAC_get_capacitance(uint8_t measurement_id) {
    return measurement_config_CAPDAC_get_capacitance(
        measurement_config_CAPDAC_get_capacitance(measurement_id)
    );
}

float slight_FDC1004::measurement_config_CAPDAC_get_capacitance(measurement_id_t measurement_id) {
    uint8_t raw = measurement_config_CAPDAC_get(measurement_id);
    float capacitance_pF = 0.0;
    //     [09:05] CAPDAC Offset Capacitance
    //         b00000 0pF (min programmable offset)
    //         ...    C_offset = CAPDAC x 3.125pF
    //         b11111 96.875pF (max programmable offset)
    capacitance_pF = raw * 3.125;
    return capacitance_pF;
}


void slight_FDC1004::measurement_config_CAPDAC_set(
    uint8_t measurement_id,
    uint8_t config
) {
    measurement_config_CAPDAC_set(
        measurement_id_bounded(measurement_id),
        config
    );
}

void slight_FDC1004::measurement_config_CAPDAC_set(
    measurement_id_t measurement_id,
    uint8_t config
) {
    _reg_MEASn_CONFIG[measurement_id] = set_register16bit_part(
        _reg_MEASn_CONFIG[measurement_id],
        measurement_config_CAPDAC_mask,
        measurement_config_CAPDAC_shift,
        config
    );
}

void slight_FDC1004::measurement_config_CAPDAC_set_capacitance(
    uint8_t measurement_id,
    float value
) {
    measurement_config_CAPDAC_set_capacitance(
        measurement_id_bounded(measurement_id),
        value
    );
}

void slight_FDC1004::measurement_config_CAPDAC_set_capacitance(
    measurement_id_t measurement_id,
    float value
) {
    // value = capacitance in pF
    //     [09:05] CAPDAC Offset Capacitance
    //         b00000 0pF (min programmable offset)
    //         ...    C_offset = CAPDAC x 3.125pF
    //         b11111 96.875pF (max programmable offset)
    uint8_t raw = value / 3.125;
    measurement_config_CAPDAC_set(measurement_id, raw);
}



// 8.6.3 FDC Configuration Register
// read and write
// config for triggering and measurement completion
// FDC_CONFIG
//     [15] RST Reset
//         write 1 to initiate reset. returns to 0 after completion
//         0 normal operation
//         1 software reset;
//     [14:12] RESERVED (read only)
//     [11:10] RATE Measurement Rate
//         b00 reserved
//         b01 100S/s
//         b10 200S/s
//         b11 400S/s
//     [9] RESERVED (read only)
//     [8] REPEAT Repeat Measurements
//         0 dispabled
//         1 enabled
//     [7] MEAS_1 initiate measurement
//         0 dispabled
//         1 enabled
//     [6] MEAS_2 initiate measurement
//         0 dispabled
//         1 enabled
//     [5] MEAS_3 initiate measurement
//         0 dispabled
//         1 enabled
//     [4] MEAS_4 initiate measurement
//         0 dispabled
//         1 enabled
//     [3] DONE_1 measurement done
//         0 dispabled
//         1 enabled
//     [2] DONE_2 measurement done
//         0 dispabled
//         1 enabled
//     [1] DONE_3 measurement done
//         0 dispabled
//         1 enabled
//     [0] DONE_4 measurement done
//         0 dispabled
//         1 enabled

// full register
uint16_t slight_FDC1004::fdc_config_read() {
    _reg_FDC_CONFIG = read_register16bit(REG_FDC_CONFIG);
    return _reg_FDC_CONFIG;
}

void slight_FDC1004::fdc_config_write() {
    write_register16bit(REG_FDC_CONFIG, _reg_FDC_CONFIG);
}

uint16_t slight_FDC1004::fdc_config_get() {
    return _reg_FDC_CONFIG;
}

void slight_FDC1004::fdc_config_set(uint16_t value) {
    _reg_FDC_CONFIG = value;
}


// measurement done
boolean slight_FDC1004::measurement_done_get(uint8_t measurement_id) {
    return measurement_done_get(
        measurement_id_bounded(measurement_id)
    );
}

boolean slight_FDC1004::measurement_done_get(measurement_id_t measurement_id) {
    uint8_t shift = shift_DONE_1;
    shift += measurement_id;
    uint16_t mask = mask_DONE_1;
    mask += measurement_id;
    return (boolean)get_register16bit_part(
        _reg_FDC_CONFIG,
        mask,
        shift
    );
}


// measurement init
boolean slight_FDC1004::measurement_init_get(uint8_t measurement_id) {
    return measurement_init_get(
        measurement_id_bounded(measurement_id)
    );
}

boolean slight_FDC1004::measurement_init_get(measurement_id_t measurement_id) {
    uint8_t shift = shift_INIT_1;
    shift += measurement_id;
    uint16_t mask = mask_INIT_1;
    mask += measurement_id;
    return (boolean)get_register16bit_part(
        _reg_FDC_CONFIG,
        mask,
        shift
    );
}

void slight_FDC1004::measurement_init_set(uint8_t measurement_id, boolean value) {
    measurement_init_set(
        measurement_id_bounded(measurement_id),
        value
    );
}

void slight_FDC1004::measurement_init_set(measurement_id_t measurement_id, boolean value) {
    uint8_t shift = shift_INIT_1;
    shift += measurement_id;
    uint16_t mask = mask_INIT_1;
    mask += measurement_id;
    set_register16bit_part(
        _reg_FDC_CONFIG,
        mask,
        shift,
        value
    );
}

void slight_FDC1004::measurement_init(uint8_t measurement_id) {
    measurement_init(
        measurement_id_bounded(measurement_id)
    );
}

void slight_FDC1004::measurement_init(measurement_id_t measurement_id) {
    measurement_init_set(
        measurement_id,
        true
    );
}


// measurement repeate
boolean slight_FDC1004::measurement_repeate_get() {
    return get_register16bit_part(
        _reg_FDC_CONFIG,
        mask_REPEATE,
        shift_REPEATE
    );
}

void slight_FDC1004::measurement_repeate_set(boolean value) {
    set_register16bit_part(
        _reg_FDC_CONFIG,
        mask_REPEATE,
        shift_REPEATE,
        value
    );
}


// measurement repeate rate
slight_FDC1004::fdc_config_repeate_rate_t slight_FDC1004::measurement_rate_get() {
    return (fdc_config_repeate_rate_t)get_register16bit_part(
        _reg_FDC_CONFIG,
        mask_RATE,
        shift_RATE
    );
}

void slight_FDC1004::measurement_rate_set(fdc_config_repeate_rate_t value) {
    set_register16bit_part(
        _reg_FDC_CONFIG,
        mask_RATE,
        shift_RATE,
        (uint8_t)value
    );
}

void slight_FDC1004::measurement_rate_set(uint8_t value) {
    fdc_config_repeate_rate_t rate = repeate_rate_100Ss;
    switch (value) {
        case 1: {
            rate = repeate_rate_100Ss;
        } break;
        case 2: {
            rate = repeate_rate_200Ss;
        } break;
        case 3: {
            rate = repeate_rate_200Ss;
        } break;
        case 4: {
            rate = repeate_rate_400Ss;
        } break;
        default: {
            rate = repeate_rate_100Ss;
        } break;
    }
    measurement_rate_set(rate);
}


// device reset
boolean slight_FDC1004::soft_reset_read() {
    return read_register16bit_part(
        REG_FDC_CONFIG,
        mask_RESET,
        shift_RESET
    );
}

void slight_FDC1004::soft_reset() {
    write_register16bit_part(
        REG_FDC_CONFIG,
        mask_RESET,
        shift_RESET,
        true
    );
}



// 8.6.4 Offset Calibration Registers
// read and write
// fixed point number
// digitized capacitance value: -16pF..+16pF
// OFFSET_CALn
//     [15:11] Integer part
//     [10:0] Decimal part

uint16_t slight_FDC1004::offset_calibration_get(uint8_t measurement_id) {
    return offset_calibration_get(
        measurement_id_bounded(measurement_id)
    );
}

uint16_t slight_FDC1004::offset_calibration_get(measurement_id_t measurement_id) {
    uint8_t reg = REG_CIN1_OFFSET_CALIBRATION;
    reg += measurement_id;
    return read_register16bit(reg);
}

void slight_FDC1004::offset_calibration_set(uint8_t measurement_id, uint16_t value) {
    offset_calibration_set(
        measurement_id_bounded(measurement_id),
        value
    );
}

void slight_FDC1004::offset_calibration_set(measurement_id_t measurement_id, uint16_t value) {
    uint8_t reg = REG_CIN1_OFFSET_CALIBRATION;
    reg += measurement_id;
    write_register16bit(reg, value);
}


void slight_FDC1004::offset_calibration_print_capacitance(Print &out, uint16_t raw) {
    uint8_t integer_part = raw >> 10;
    uint16_t decimal_part = raw & 0b0000011111111111;
    out.print(integer_part);
    out.print(".");
    out.print(decimal_part);
}

float slight_FDC1004::offset_calibration_get_capacitance(uint8_t measurement_id) {
    return offset_calibration_get_capacitance(
        measurement_id_bounded(measurement_id)
    );
}


// float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
//     // https://www.arduino.cc/en/Reference/Map
//     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

float slight_FDC1004::offset_calibration_get_capacitance(measurement_id_t measurement_id) {
    uint16_t raw = offset_calibration_get(measurement_id);
    // float capacitance = 0.0;
    // TODO convert raw value to capacitance
    // fixed point number
    // digitized capacitance value: -16pF..+16pF
    // OFFSET_CALn
    //     [15:11] Integer part
    //     [10:0] Decimal part
    // we first convert the 16bit to a float type and move the decimal part.
    float capacitance = raw / 10E10;
    // eventually we have to map the range?!
    // capacitance = map_float(raw_cap, ?, ?, -16.0, 16.0);
    return capacitance;
}

void slight_FDC1004::offset_calibration_set_capacitance(uint8_t measurement_id, float value) {
    offset_calibration_set_capacitance(
        measurement_id_bounded(measurement_id),
        value
    );
}

void slight_FDC1004::offset_calibration_set_capacitance(
    measurement_id_t measurement_id,
    float value
) {
    // TODO convert value to raw
    // fixed point number
    // digitized capacitance value: -16pF..+16pF
    // OFFSET_CALn
    //     [15:11] Integer part
    //     [10:0] Decimal part
    // we multiply the value with 10E10 -
    // so we have move the decimal part into the integer area
    uint16_t raw = (value * 10E10);
    // uint16_t raw = 0;
    offset_calibration_set(
        measurement_id,
        raw
    );
}



// 8.6.5 Gain Calibration Registers
// read and write
// fixed point number
// gain factor correction range: 0..4
// Gain = GAIN_CAL[15:0]/2^14
// OFFSET_CALn
//     [15:14] Integer part
//     [13:0] Decimal part

uint16_t slight_FDC1004::gain_calibration_get(uint8_t measurement_id) {
    return gain_calibration_get(
        measurement_id_bounded(measurement_id)
    );
}

uint16_t slight_FDC1004::gain_calibration_get(measurement_id_t measurement_id) {
    uint8_t reg = REG_CIN1_GAIN_CALIBRATION;
    reg += measurement_id;
    return read_register16bit(reg);
}

void slight_FDC1004::gain_calibration_set(uint8_t measurement_id, uint16_t value) {
    gain_calibration_set(
        measurement_id_bounded(measurement_id),
        value
    );
}

void slight_FDC1004::gain_calibration_set(measurement_id_t measurement_id, uint16_t value) {
    uint8_t reg = REG_CIN1_GAIN_CALIBRATION;
    reg += measurement_id;
    write_register16bit(reg, value);
}


void slight_FDC1004::gain_calibration_print_factor(Print &out, uint16_t raw) {
    uint8_t integer_part = raw >> 13;
    uint16_t decimal_part = raw & (1 << 13);
    out.print(integer_part);
    out.print(".");
    out.print(decimal_part);
}


float slight_FDC1004::gain_calibration_get_factor(uint8_t measurement_id) {
    return gain_calibration_get_factor(
        measurement_id_bounded(measurement_id)
    );
}

float slight_FDC1004::gain_calibration_get_factor(measurement_id_t measurement_id) {
    uint16_t raw = offset_calibration_get(measurement_id);
    float factor = raw / 10E10;
    return factor;
}

void slight_FDC1004::gain_calibration_set_factor(uint8_t measurement_id, float value) {
    gain_calibration_set_factor(
        measurement_id_bounded(measurement_id),
        value
    );
}

void slight_FDC1004::gain_calibration_set_factor(measurement_id_t measurement_id, float value) {
    uint16_t raw = (value * 10E13);
    gain_calibration_set(
        measurement_id,
        raw
    );
}


// 8.6.6 Manufacturer ID Register
// read only
// fixed at 0x5449
uint16_t slight_FDC1004::manufacturer_id_get() {
    uint16_t reg = read_register16bit(REG_MANUFACTURER_ID);
    return reg;
}

// 8.6.6 Device ID Register
// read only
// fixed at 0x1004
uint16_t slight_FDC1004::device_id_get() {
    uint16_t reg = read_register16bit(REG_DEVICE_ID);
    return reg;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// END OF REGISTER FUNCTIONS
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Helpers / Allow Writes
// void slight_FDC1004::switch_mode_temporarily_to_allow_write() {
//     Serial.println(F("switch_mode_temporarily_to_allow_write"));
//     switch_mode_backup_electrode_config = read_register(
//         REG_Electrode_Configuration
//     );
//     write_register(
//         REG_Electrode_Configuration,
//         electrode_config_StopMode
//     );
// }
//
// void slight_FDC1004::switch_mode_restore() {
//     Serial.println(F("switch_mode_restore"));
//     write_register(
//         REG_Electrode_Configuration,
//         switch_mode_backup_electrode_config
//     );
// }


slight_FDC1004::measurement_id_t slight_FDC1004::measurement_id_bounded(
    uint8_t measurement_id
) {
    if (measurement_id > 3) {
        measurement_id = 3;
    }
    return (measurement_id_t)measurement_id;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// advanced read write operations
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// alterantive to __builtin_popcount:
// found at https://en.wikipedia.org/wiki/Hamming_weight
// int popcount_4(uint64_t x) {
//     int count;
//     for (count=0; x; count++)
//         x &= x-1;
//     return count;
// }

uint8_t slight_FDC1004::ones_in_mask_get(uint8_t mask) {
    uint8_t ones_in_mask = __builtin_popcount(mask);
    return ones_in_mask;
}

uint8_t slight_FDC1004::value_max_get(uint8_t mask) {
    uint8_t ones_in_mask = ones_in_mask_get(mask);
    uint8_t value_max = 0b11111111 >> (8 - ones_in_mask);
    return value_max;
}

uint8_t slight_FDC1004::value_max_get(uint8_t mask, uint8_t shift) {
    uint8_t value_max = mask >> shift;
    return value_max;
}

uint16_t slight_FDC1004::value_max_get(uint16_t mask, uint16_t shift) {
    uint16_t value_max = mask >> shift;
    return value_max;
}

uint8_t slight_FDC1004::value_limit(
    uint8_t mask,
    uint8_t shift,
    uint8_t value
) {
    uint8_t value_max = value_max_get(mask, shift);
    if (value > value_max) {
        value = value_max;
    }
    return value;
}



// public
// uint8_t slight_FDC1004::read_register_part(
//     register_name_t reg_name,
//     uint16_t reg_mask,
//     uint8_t reg_shift
// ) {
//     return read_register_part(
//         (uint8_t)reg_name,
//         reg_mask,
//         reg_shift
//     );
// }
uint8_t slight_FDC1004::read_register16bit_part(
    register_name_t reg_name,
    uint16_t reg_mask,
    uint8_t reg_shift
) {
    return read_register16bit_part(
        (uint8_t)reg_name,
        reg_mask,
        reg_shift
    );
}


uint8_t slight_FDC1004::get_register16bit_part(
    uint16_t reg_value,
    uint16_t reg_mask,
    uint8_t reg_shift
) {
    // isolate bits
    reg_value = reg_value & reg_mask;
    // shift to correct bits.
    reg_value = reg_value >> reg_shift;
    // only values <8bit are returned.
    return (uint8_t)reg_value;
}

// void slight_FDC1004::write_register_part(
//     register_name_t reg_name,
//     uint16_t reg_mask,
//     uint8_t reg_shift,
//     uint8_t value
// ) {
//     write_register_part(
//         (uint8_t)reg_name,
//         reg_mask,
//         reg_shift,
//         value
//     );
// }
void slight_FDC1004::write_register16bit_part(
    register_name_t reg_name,
    uint16_t reg_mask,
    uint8_t reg_shift,
    uint8_t value
) {
    write_register16bit_part(
        (uint8_t)reg_name,
        reg_mask,
        reg_shift,
        value
    );
}

uint16_t slight_FDC1004::set_register16bit_part(
    uint16_t reg_value,
    uint16_t reg_mask,
    uint8_t reg_shift,
    uint8_t value
) {
    value = value_limit(reg_mask, reg_shift, value);
    // clear bits
    reg_value = reg_value & (~reg_mask);
    // set bits
    reg_value = reg_value | (value << reg_shift);
    // write register
    return reg_value;
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// private
// uint8_t slight_FDC1004::read_register_part(
//     uint8_t reg_name,
//     uint8_t reg_mask,
//     uint8_t reg_shift
// ) {
//     // read register
//     uint8_t reg_value = read_register(reg_name);
//     // isolate bits
//     reg_value = reg_value & reg_mask;
//     // shift to correct bits.
//     reg_value = reg_value >> reg_shift;
//     return reg_value;
// }
uint8_t slight_FDC1004::read_register16bit_part(
    uint8_t reg_name,
    uint16_t reg_mask,
    uint8_t reg_shift
) {
    // read register
    uint16_t reg_value = read_register16bit(reg_name);
    return get_register16bit_part(
        reg_value,
        reg_mask,
        reg_shift
    );
}


// void slight_FDC1004::write_register_part(
//     uint8_t reg_name,
//     uint8_t reg_mask,
//     uint8_t reg_shift,
//     uint8_t value
// ) {
//     value = value_limit(reg_mask, reg_shift, value);
//     // read register
//     uint8_t reg_value = read_register(reg_name);
//     // clear bits
//     reg_value = reg_value & (~reg_mask);
//     // set bits
//     reg_value = reg_value | (value << reg_shift);
//     // write register
//     write_register(reg_name, reg_value);
// }
void slight_FDC1004::write_register16bit_part(
    uint8_t reg_name,
    uint16_t reg_mask,
    uint8_t reg_shift,
    uint8_t value
) {
    // read register
    uint16_t reg_value = read_register16bit(reg_name);
    // set part
    reg_value = set_register16bit_part(
        reg_value,
        reg_mask,
        reg_shift,
        value
    );
    // write register
    write_register16bit(reg_name, reg_value);
}







//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// basic read write operations
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// uint8_t slight_FDC1004::read_register(uint8_t reg_name) {
//     uint8_t result_value = 0;
//     if (ready) {
//         twi_state = TWI_STATE_undefined;
//         // set register
//         Wire.beginTransmission(twi_address);
//         Wire.write(reg_name);
//         twi_state = (twi_state_t)Wire.endTransmission();
//         if (twi_state == TWI_STATE_success) {
//             // read data
//             Wire.requestFrom(twi_address, (uint8_t)1);
//             result_value = Wire.read();
//         } else {
//             // print_transmission_state(Serial, twi_state);
//         }
//     }
//     return result_value;
// }
//
// uint8_t slight_FDC1004::read_register(register_name_t reg_name) {
//     return read_register((uint8_t)reg_name);
// }

uint16_t slight_FDC1004::read_register16bit(uint8_t reg_name) {
    uint16_t result_value = 0;
    if (ready) {
        twi_state = TWI_STATE_undefined;
        // set register
        Wire.beginTransmission(twi_address);
        Wire.write(reg_name);
        twi_state = (twi_state_t)Wire.endTransmission();
        if (twi_state == TWI_STATE_success) {
            // read data
            Wire.requestFrom(twi_address, (uint8_t)2);
            result_value = Wire.read();
            uint16_t highbyte = ((uint16_t)Wire.read()) << 8;
            result_value |= highbyte;
        } else {
            // print_transmission_state(Serial, twi_state);
        }
    }
    return result_value;
}

uint16_t slight_FDC1004::read_register16bit(register_name_t reg_name) {
    return read_register16bit((uint8_t)reg_name);
}


// void slight_FDC1004::write_register(uint8_t reg_name, uint8_t value) {
//     if (ready) {
//         twi_state = TWI_STATE_undefined;
//         // set register
//         Wire.beginTransmission(twi_address);
//         Wire.write(reg_name);
//         Wire.write(value);
//         twi_state = (twi_state_t)Wire.endTransmission();
//         if (twi_state == TWI_STATE_success) {
//             // all fine.
//         } else {
//             // print_transmission_state(Serial, twi_state);
//         }
//     }
// }
//
// void slight_FDC1004::write_register(
//     register_name_t reg_name,
//     uint8_t value
// ) {
//     write_register((uint8_t)reg_name, value);
// }

void slight_FDC1004::write_register16bit(
    uint8_t reg_name,
    uint16_t value
) {
    if (ready) {
        twi_state = TWI_STATE_undefined;
        // set register
        Wire.beginTransmission(twi_address);
        Wire.write(reg_name);
        Wire.write(value);
        twi_state = (twi_state_t)Wire.endTransmission();
        if (twi_state == TWI_STATE_success) {
            // all fine.
        } else {
            // print_transmission_state(Serial, twi_state);
        }
    }
}

void slight_FDC1004::write_register16bit(
    register_name_t reg_name,
    uint16_t value
) {
    write_register16bit((uint8_t)reg_name, value);
}



void slight_FDC1004::twi_state_print(Print &out) {
    twi_state_print(out, twi_state);
}

void slight_FDC1004::twi_state_print(Print &out, twi_state_t state) {
    switch (state) {
        case TWI_STATE_success: {
            // out.print(F("success"));
        } break;
        case TWI_STATE_data_to_long: {
            // out.print(F("data too long to fit in transmit buffer"));
        } break;
        case TWI_STATE_rec_NACK_on_address: {
            // out.print(F("received NACK on transmit of address"));
        } break;
        case TWI_STATE_rec_NACK_on_data: {
            // out.print(F("received NACK on transmit of data"));
        } break;
        case TWI_STATE_other_error: {
            // out.print(F("other error"));
        } break;
        default: {
            // out.print(F("??"));
        }
    }
}
