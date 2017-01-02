/******************************************************************************

    arduino library for TI FDC1004 Capacitive.
    tested with adafruit MPR121 breakoutboard
        (https://www.adafruit.com/product/1982)

    basic library api is compatible to https://github.com/s-light/slight_CAP1188

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



#ifndef slight_FDC1004_H_
#define slight_FDC1004_H_

// include Core Arduino functionality
#include <Arduino.h>


class slight_FDC1004 {
public:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // constructor

    slight_FDC1004();
    // slight_FDC1004(uint8_t twi_address, uint8_t interrupt_pin);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public types

    enum twi_state_t {
        TWI_STATE_success = 0,
        TWI_STATE_data_to_long = 1,
        TWI_STATE_rec_NACK_on_address = 2,
        TWI_STATE_rec_NACK_on_data = 3,
        TWI_STATE_other_error = 4,
        TWI_STATE_undefined = 99,
    };

    // registers
    // 8.6 Register Maps
    // all registers are 16 Bits!!
    enum register_name_t {

        // 8.6.1.1 Capacitive Measurement Registers
        REG_MEAS1_MSB = 0x00,
        REG_MEAS1_LSB = 0x01,
        REG_MEAS2_MSB = 0x02,
        REG_MEAS2_LSB = 0x03,
        REG_MEAS3_MSB = 0x04,
        REG_MEAS3_LSB = 0x05,
        REG_MEAS4_MSB = 0x06,
        REG_MEAS4_LSB = 0x07,

        // 8.6.2 Measurement Configuration Registers
        REG_MEAS1_CONFIG = 0x08,
        REG_MEAS2_CONFIG = 0x09,
        REG_MEAS3_CONFIG = 0x0A,
        REG_MEAS4_CONFIG = 0x0B,

        // 8.6.3 FDC Configuration Register
        REG_FDC_CONFIG = 0x0C,

        // 8.6.4 Offset Calibration Registers
        REG_CIN1_OFFSET_CALIBRATION = 0x0D,
        REG_CIN2_OFFSET_CALIBRATION = 0x0E,
        REG_CIN3_OFFSET_CALIBRATION = 0x0F,
        REG_CIN4_OFFSET_CALIBRATION = 0x10,

        // 8.6.5 Gain Calibration Registers
        REG_CIN1_GAIN_CALIBRATION = 0x11,
        REG_CIN2_GAIN_CALIBRATION = 0x12,
        REG_CIN3_GAIN_CALIBRATION = 0x13,
        REG_CIN4_GAIN_CALIBRATION = 0x14,

        // 8.6.6 Manufacturer ID Register
        REG_MANUFACTURER_ID = 0xFE,

        // 8.6.6 Device ID Register
        REG_DEVICE_ID = 0xFF,
    };

    // callback
    typedef void (* callback_t) (slight_FDC1004 *instance);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // public functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // basic library api
    bool begin();
    void update();

    // sensor configurations
    // void configuration_load_defaults();

    // poll-update scheduler / IRQ handling
    // void update_interval_set_autofit();
    // void update_interval_set(uint32_t interval);
    // uint32_t update_interval_get();

    void sensor_event_set_callback(callback_t);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // register helper functions


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // gloal helpers

    enum measurement_id_t {
        measurement_1 = 0,
        measurement_2 = 1,
        measurement_3 = 2,
        measurement_4 = 3,
    };
    static measurement_id_t measurement_id_convert(uint8_t);
    static void measurement_id_print(
        Print &out,
        measurement_id_t value
    );


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // individual registers:

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

    uint32_t measurement_read(uint8_t measurement_id);
    uint32_t measurement_read(measurement_id_t measurement_id);
    float capacitance_read(uint8_t measurement_id);
    float capacitance_read(measurement_id_t measurement_id);

    uint32_t measurement_get(uint8_t measurement_id);
    uint32_t measurement_get(measurement_id_t measurement_id);
    float capacitance_get(uint8_t measurement_id);
    float capacitance_get(measurement_id_t measurement_id);

    float convert_measurement_to_capacitance(uint32_t measurement_value);

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

    static const uint16_t measurement_config_RESERVED_mask = 0b0000000000011111;
    static const uint8_t measurement_config_RESERVED_shift = 0;
    static const uint16_t measurement_config_CAPDAC_mask = 0b0000001111100000;
    static const uint8_t measurement_config_CAPDAC_shift = 0 + 5;
    static const uint16_t measurement_config_chB_mask = 0b0001110000000000;
    static const uint8_t measurement_config_chB_shift = 0 + 5 + 5;
    static const uint16_t measurement_config_chA_mask = 0b1110000000000000;
    static const uint8_t measurement_config_chA_shift = 0 + 5 + 5 + 3;

    enum measurement_config_chA_t {
        config_chA_CIN1 = 0b000,
        config_chA_CIN2 = 0b001,
        config_chA_CIN3 = 0b010,
        config_chA_CIN4 = 0b011,
    };
    enum measurement_config_chB_t {
        config_chB_CIN1 = 0b000,
        config_chB_CIN2 = 0b001,
        config_chB_CIN3 = 0b010,
        config_chB_CIN4 = 0b011,
        config_chB_CAPDAC = 0b100,
        config_chB_DISPABLED = 0b111,
    };

    // read & write from & to chip
    uint16_t measurement_config_read(uint8_t measurement_id);
    uint16_t measurement_config_read(measurement_id_t measurement_id);
    void measurement_config_write(uint8_t measurement_id);
    void measurement_config_write(measurement_id_t measurement_id);

    // get internal stored values
    // full register
    uint16_t measurement_config_get(uint8_t measurement_id);
    uint16_t measurement_config_get(measurement_id_t measurement_id);
    void measurement_config_set(uint8_t measurement_id, uint16_t value);
    void measurement_config_set(measurement_id_t measurement_id, uint16_t value);

    // chA
    measurement_config_chA_t measurement_config_chA_get(uint8_t measurement_id);
    measurement_config_chA_t measurement_config_chA_get(measurement_id_t measurement_id);
    void measurement_config_chA_set(uint8_t measurement_id, measurement_config_chA_t config);
    void measurement_config_chA_set(measurement_id_t measurement_id, measurement_config_chA_t config);

    // chB
    measurement_config_chB_t measurement_config_chB_get(uint8_t measurement_id);
    measurement_config_chB_t measurement_config_chB_get(measurement_id_t measurement_id);
    void measurement_config_chB_set(uint8_t measurement_id, measurement_config_chB_t config);
    void measurement_config_chB_set(measurement_id_t measurement_id, measurement_config_chB_t config);

    // CAPDAC
    uint8_t measurement_config_CAPDAC_get(uint8_t measurement_id);
    uint8_t measurement_config_CAPDAC_get(measurement_id_t measurement_id);
    float measurement_config_CAPDAC_get_capacitance(uint8_t measurement_id);
    float measurement_config_CAPDAC_get_capacitance(measurement_id_t measurement_id);
    void measurement_config_CAPDAC_set(uint8_t measurement_id, uint8_t value);
    void measurement_config_CAPDAC_set(measurement_id_t measurement_id, uint8_t value);
    void measurement_config_CAPDAC_set_capacitance(uint8_t measurement_id, float value);
    void measurement_config_CAPDAC_set_capacitance(measurement_id_t measurement_id, float value);


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

    // static const uint16_t fdc_config_DONE4_mask =       0b0000000000000001;
    // static const uint8_t  fdc_config_DONE4_shift = 0;
    // static const uint16_t fdc_config_DONE3_mask =       0b0000000000000010;
    // static const uint8_t  fdc_config_DONE3_shift = 1;
    // static const uint16_t fdc_config_DONE2_mask =       0b0000000000000100;
    // static const uint8_t  fdc_config_DONE2_shift = 2;
    // static const uint16_t fdc_config_DONE1_mask =       0b0000000000001000;
    // static const uint8_t  fdc_config_DONE1_shift = 3;
    // static const uint16_t fdc_config_MEAS4_mask =       0b0000000000010000;
    // static const uint8_t  fdc_config_MEAS4_shift = 4;
    // static const uint16_t fdc_config_MEAS3_mask =       0b0000000000100000;
    // static const uint8_t  fdc_config_MEAS3_shift = 5;
    // static const uint16_t fdc_config_MEAS2_mask =       0b0000000001000000;
    // static const uint8_t  fdc_config_MEAS2_shift = 6;
    // static const uint16_t fdc_config_MEAS1_mask =       0b0000000010000000;
    // static const uint8_t  fdc_config_MEAS1_shift = 7;
    // static const uint16_t fdc_config_REPEATE_mask =     0b0000000100000000;
    // static const uint8_t  fdc_config_REPEATE_shift = 8;
    // static const uint16_t fdc_config_RESERVED0_mask =   0b0000001000000000;
    // static const uint8_t  fdc_config_RESERVED0_shift = 9;
    // static const uint16_t fdc_config_RATE_mask =        0b0000110000000000;
    // static const uint8_t  fdc_config_RATE_shift = 10;
    // static const uint16_t fdc_config_RESERVED1_mask =   0b0111000000000000;
    // static const uint8_t  fdc_config_RESERVED1_shift = 12;
    // static const uint16_t fdc_config_RESET_mask =       0b1000000000000000;
    // static const uint8_t  fdc_config_RESET_shift = 15;

    enum fdc_config_mask {
        mask_DONE_1 = 0b0000000000001000,
        mask_DONE_2 = 0b0000000000000100,
        mask_DONE_3 = 0b0000000000000010,
        mask_DONE_4 = 0b0000000000000001,
        mask_INIT_4 = 0b0000000000010000,
        mask_INIT_3 = 0b0000000000100000,
        mask_INIT_2 = 0b0000000001000000,
        mask_INIT_1 = 0b0000000010000000,
        mask_REPEATE = 0b0000000100000000,
        // REVERSED = 0b0000001000000000,
        mask_RATE = 0b0000110000000000,
        // REVERSED = 0b0111000000000000,
        mask_RESET = 0b1000000000000000,
    };

    enum fdc_config_shift {
        shift_DONE_1 = 3,
        shift_DONE_2 = 2,
        shift_DONE_3 = 1,
        shift_DONE_4 = 0,
        shift_INIT_1 = 7,
        shift_INIT_2 = 6,
        shift_INIT_3 = 5,
        shift_INIT_4 = 4,
        shift_REPEATE = 8,
        // REVERSED = 9,
        shift_RATE = 10,
        // REVERSED = 12,
        shift_RESET = 15,
    };

    enum fdc_config_repeate_rate_t {
        repeate_rate_100Ss = 0b001,
        repeate_rate_200Ss = 0b010,
        repeate_rate_400Ss = 0b011,
    };

    // read & write to chip
    uint16_t fdc_config_read();
    void fdc_config_write();

    // full register
    uint16_t fdc_config_get();
    void fdc_config_set(uint16_t value);

    // measurement done
    boolean measurement_done_get(uint8_t measurement_id);
    boolean measurement_done_get(measurement_id_t measurement_id);

    // measurement init
    boolean measurement_init_get(uint8_t measurement_id);
    boolean measurement_init_get(measurement_id_t measurement_id);
    void measurement_init_set(uint8_t measurement_id, boolean value);
    void measurement_init_set(measurement_id_t measurement_id, boolean value);
    void measurement_init(uint8_t measurement_id);
    void measurement_init(measurement_id_t measurement_id);

    // measurement repeate
    boolean measurement_repeate_get();
    void measurement_repeate_set(boolean value);

    // measurement repeate rate
    fdc_config_repeate_rate_t measurement_rate_get();
    void measurement_rate_set(fdc_config_repeate_rate_t value);
    void measurement_rate_set(uint8_t value);

    // device reset
    boolean soft_reset_read();
    void soft_reset();


    // 8.6.4 Offset Calibration Registers
    // read and write
    // fixed point number
    // digitized capacitance value: -16pF..+16pF
    // OFFSET_CALn
    //     [15:11] Integer part
    //     [10:0] Decimal part

    uint16_t offset_calibration_get(uint8_t measurement_id);
    uint16_t offset_calibration_get(measurement_id_t measurement_id);
    void offset_calibration_set(uint8_t measurement_id, uint16_t value);
    void offset_calibration_set(measurement_id_t measurement_id, uint16_t value);

    void offset_calibration_print_capacitance(Print &out, uint16_t raw);

    float offset_calibration_get_capacitance(uint8_t measurement_id);
    float offset_calibration_get_capacitance(measurement_id_t measurement_id);
    void offset_calibration_set_capacitance(uint8_t measurement_id, float value);
    void offset_calibration_set_capacitance(measurement_id_t measurement_id, float value);


    // 8.6.5 Gain Calibration Registers
    // read and write
    // fixed point number
    // gain factor correction range: 0..4
    // Gain = GAIN_CAL[15:0]/2^14
    // OFFSET_CALn
    //     [15:14] Integer part
    //     [13:0] Decimal part

    uint16_t gain_calibration_get(uint8_t measurement_id);
    uint16_t gain_calibration_get(measurement_id_t measurement_id);
    void gain_calibration_set(uint8_t measurement_id, uint16_t value);
    void gain_calibration_set(measurement_id_t measurement_id, uint16_t value);

    void gain_calibration_print_factor(Print &out, uint16_t raw);

    float gain_calibration_get_factor(uint8_t measurement_id);
    float gain_calibration_get_factor(measurement_id_t measurement_id);
    void gain_calibration_set_factor(uint8_t measurement_id, float value);
    void gain_calibration_set_factor(measurement_id_t measurement_id, float value);

    // 8.6.6 Manufacturer ID Register
    // read only
    // fixed at 0x5449
    uint16_t manufacturer_id_get();

    // 8.6.6 Device ID Register
    // read only
    // fixed at 0x1004
    uint16_t device_id_get();


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // advanced read write operations
    // uint8_t read_register_part(
    //     register_name_t reg_name,
    //     uint8_t reg_mask,
    //     uint8_t reg_shift
    // );
    uint8_t read_register16bit_part(
        register_name_t reg_name,
        uint16_t reg_mask,
        uint8_t reg_shift
    );

    uint8_t get_register16bit_part(
        uint16_t reg_value,
        uint16_t reg_mask,
        uint8_t reg_shift
    );

    // void write_register_part(
    //     register_name_t reg_name,
    //     uint8_t reg_mask,
    //     uint8_t reg_shift,
    //     uint8_t value
    // );
    void write_register16bit_part(
        register_name_t reg_name,
        uint16_t reg_mask,
        uint8_t reg_shift,
        uint8_t value
    );

    uint16_t set_register16bit_part(
        uint16_t reg_value,
        uint16_t reg_mask,
        uint8_t reg_shift,
        uint8_t value
    );

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // basic read write operations
    // void write_register(register_name_t reg_name, uint8_t value);
    void write_register16bit(register_name_t reg_name, uint16_t value);
    // uint8_t read_register(register_name_t reg_name);
    uint16_t read_register16bit(register_name_t reg_name);


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // twi state helper
    twi_state_t twi_state_get();
    void twi_state_print(Print &out);
    static void twi_state_print(Print &out, twi_state_t state);

private:

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // private functions
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    void sensor_status_update();

    void sensor_event_callback();

    // write helper
    void switch_mode_temporarily_to_allow_write();
    void switch_mode_restore();
    uint8_t switch_mode_backup_electrode_config;

    // measurement helper
    measurement_id_t measurement_id_bounded(uint8_t measurement_id);

    // advanced read write operations / helper
    uint8_t ones_in_mask_get(uint8_t mask);
    uint8_t value_max_get(uint8_t mask);
    uint8_t value_max_get(uint8_t mask, uint8_t shift);
    uint16_t value_max_get(uint16_t mask, uint16_t shift);
    uint8_t value_limit(uint8_t mask, uint8_t shift, uint8_t);

    // uint8_t read_register_part(
    //     uint8_t reg_name,
    //     uint8_t reg_mask,
    //     uint8_t reg_shift
    // );
    uint8_t read_register16bit_part(
        uint8_t reg_name,
        uint16_t reg_mask,
        uint8_t reg_shift
    );

    // void write_register_part(
    //     uint8_t reg_name,
    //     uint8_t reg_mask,
    //     uint8_t reg_shift,
    //     uint8_t value
    // );
    void write_register16bit_part(
        uint8_t reg_name,
        uint16_t reg_mask,
        uint8_t reg_shift,
        uint8_t value
    );

    // basic read write operations
    // void write_register(uint8_t reg_name, uint8_t value);
    void write_register16bit(uint8_t reg_name, uint16_t value);
    // uint8_t read_register(uint8_t reg_name);
    uint16_t read_register16bit(uint8_t reg_name);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // attributes

    bool ready;
    twi_state_t twi_state;

    static const uint8_t twi_address = 0x50;

    uint8_t touch_status;
    uint8_t touch_status_old;
    bool touch_status_overcurrent_flag;

    uint32_t timestamp_lastread;
    uint32_t update_interval;

    callback_t callback_sensor_event;

    // internal storage of register settings
    // defaults = datasheet/chip Reset Values
    uint16_t _reg_FDC_CONFIG = 0;
    uint16_t _reg_MEASn_CONFIG[4] = {
        0x1C00,
        0x1C00,
        0x1C00,
        0x1C00
    };

    uint32_t _reg_MEASn_VALUE[4] = {0, 0, 0, 0};


};  // class slight_FDC1004

#endif  // slight_FDC1004_H_
