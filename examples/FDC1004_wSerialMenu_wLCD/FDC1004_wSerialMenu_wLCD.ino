/******************************************************************************

    FDC1004_wSerialMenu_wLCD.ino
        sketch for basic test with the FDC1004 sensor breakout.
        debugout on usbserial interface: 115200baud

    hardware:
        Board:
            Arduino compatible (with serial port)
            LED on pin 13
            FDC1004
                TWI (SDA & SCL)
            LCD
                A5..A0


    libraries used:
        ~ LiquidCrystal
            arduino build in LiquidCrystal library
        ~ slight_FDC1004
        ~ slight_DebugMenu
            written by stefan krueger (s-light),
                github@s-light.eu, http://s-light.eu, https://github.com/s-light/
            license: MIT

    written by stefan krueger (s-light),
        github@s-light.eu, http://s-light.eu, https://github.com/s-light/

    changelog / history
        27.12.2016 12:10 created (based on MPR121_Simple_wMenu.ino)
        27.12.2016 12:10 renamed/structured
        see git commits for additional information.

    TO DO:
        ~ enjoy your life ;-)


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

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Includes
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// use "file.h" for files in same directory as .ino
// #include "file.h"
// use <file.h> for files in library directory
// #include <file.h>

#include <LiquidCrystal.h>

#include <slight_DebugMenu.h>

#include <Wire.h>
#include <slight_FDC1004.h>

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Info
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void sketchinfo_print(Print &out) {
    out.println();
    //             "|~~~~~~~~~|~~~~~~~~~|~~~..~~~|~~~~~~~~~|~~~~~~~~~|"
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|                       ^ ^                      |"));
    out.println(F("|                      (0,0)                     |"));
    out.println(F("|                      ( _ )                     |"));
    out.println(F("|                       \" \"                      |"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("| FDC1004_wSerialMenu_wLCD.ino"));
    out.println(F("|   sketch for simple test with the"));
    out.println(F("|   FDC1004 sensor breakout."));
    out.println(F("|"));
    out.println(F("| This Sketch has a debug-menu:"));
    out.println(F("| send '?'+Return for help"));
    out.println(F("|"));
    out.println(F("| dream on & have fun :-)"));
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println(F("|"));
    //out.println(F("| compiled: Nov 11 2013  20:35:04"));
    out.print(F("| compiled: "));
    out.print(F(__DATE__));
    out.print(F("  "));
    out.print(F(__TIME__));
    out.println();
    out.print(F("| last changed: "));
    out.print(F(__TIMESTAMP__));
    out.println();
    //
    // out.println(F("|"));
    // out.print(F("| __FILE__: "));
    // out.print(F(__FILE__));
    // out.println();
    // out.print(F("| __BASE_FILE__"));
    // out.print(F(__BASE_FILE__));
    // out.println();
    // out.println(F("|"));
    //
    out.println(F("|"));
    out.println(F("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"));
    out.println();

    // __DATE__ Nov 11 2013
    // __TIME__ 20:35:04
    // __TIMESTAMP__ Tue Dec 27 14:14:04 2016
    // __FILE__  /home/stefan/_LocalData/projects/_arduino_sketchbook/libraries/slight_FDC1004/examples/FDC1004_wSerialMenu_wLCD/FDC1004_wSerialMenu_wLCD.ino
    // __BASE_FILE__ /tmp/arduino_build_330237/sketch/FDC1004_wSerialMenu_wLCD.ino.cpp

}


// Serial.print to Flash: Notepad++ Replace RegEx
//     Find what:        Serial.print(.*)\("(.*)"\);
//     Replace with:    Serial.print\1\(F\("\2"\)\);



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// definitions (global)
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Debug Output

boolean infoled_state = 0;
const byte infoled_pin = 13;

unsigned long debugOut_LiveSign_TimeStamp_LastAction = 0;
const uint16_t debugOut_LiveSign_UpdateInterval = 1000; //ms

boolean debugOut_LiveSign_Serial_Enabled = 0;
boolean debugOut_LiveSign_LED_Enabled = 1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu

// slight_DebugMenu(Stream &in_ref, Print &out_ref, uint8_t input_length_new);
slight_DebugMenu myDebugMenu(Serial, Serial, 15);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// init LCD
LiquidCrystal lcd(A5, A4, A3, A2, A1, A0);

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// FDC1004

// FDC1004 TWI Addresses is fixed to 0x50
// slight_FDC1004 mySensor(
//     0x50
// );
slight_FDC1004 mySensor;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// functions
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// debug things

// freeRam found at
// http://forum.arduino.cc/index.php?topic=183790.msg1362282#msg1362282
// posted by mrburnette
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// LCD
void lcd_init(Print &out) {
    out.println(F("setup lcd:")); {
        out.println(F("\t init as 16x2 lcd."));
        lcd.begin(16, 2);
        out.println(F("\t print welcome message"));
        lcd.setCursor(0, 0);
        lcd.print(F("FDC1004 "));
        lcd.print(F(__TIME__));
        // lcd.setCursor(0, 1);
        // lcd.print(F("test "));
        lcd.setCursor(0, 1);
        lcd.print(__DATE__);
        // out.println(__DATE__); Nov 11 2013
        // out.println(__TIME__); 20:35:04
    }
    out.println(F("\t finished."));
}

void lcd_update() {
    lcd.setCursor(0, 0);
    // lcd.print(F("FDC1004 "));
    slight_DebugMenu::print_uint32_align_right(
        lcd,
        mySensor.measurement_get(slight_FDC1004::MESA_1)
    );
    // lcd.print(
    //     mySensor.capacitance_get(slight_FDC1004::MESA_1)
    // );

    // lcd.setCursor(0, 1);
    // slight_DebugMenu::print_uint32_align_right(
    //     lcd,
    //     mySensor.measurement_get(slight_FDC1004::MESA_2)
    // );
    // lcd.print(F("="));
    // lcd.print(
    //     mySensor.capacitance_get(slight_FDC1004::MESA_2)
    // );

    lcd.setCursor(0, 1);
    lcd.print(F("       "));
    lcd.setCursor(0, 1);
    lcd.print(
        mySensor.capacitance_get(slight_FDC1004::MESA_1)
    );

    // lcd.print(temp);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Menu System

// Main Menu
void handleMenu_Main(slight_DebugMenu *pInstance) {
    Print &out = pInstance->get_stream_out_ref();
    char *command = pInstance->get_command_current_pointer();
    // out.print("command: '");
    // out.print(command);
    // out.println("'");
    switch (command[0]) {
        case 'h':
        case 'H':
        case '?': {
            // help
            out.println(F("____________________________________________________________"));
            out.println();
            out.println(F("Help for Commands:"));
            out.println();
            out.println(F("\t '?': this help"));
            out.println(F("\t '!': sketch info"));
            out.println(F("\t 'y': toggle DebugOut livesign print"));
            out.println(F("\t 'Y': toggle DebugOut livesign LED"));
            out.println(F("\t 'x': tests"));
            out.println();
            out.println(F("\t 'd': dump sensor config"));
            out.println(F("\t 'i': init measurements"));
            out.println(F("\t 'r': read measurements"));
            out.println(F("\t 't': test read & write"));
            out.println(F("\t 'c': set config 'c'"));
            out.println(F("\t 'R': reset sensor"));
            out.println();
            out.println(F("\t 'D': raw dump sensor data"));
            // out.println(F("\t 'r': raw read measurements"));
            out.println(F("\t 'C': raw read config"));
            // out.println(F("\t 'c': raw write config"));
            // out.println();
            // out.println(F("\t 's': sensitivity set [1, 2, .., 64, 128] 's128'"));
            // out.println(F("\t 'S': sensitivity get "));
            // out.println(F("\t 'a': auto config - load recommend configuration"));
            // out.println(F("\t 'p': print things "));
            // out.println(F("\t 't': touch status get "));
            // out.println(F("\t 'c': electrode config get "));
            // out.println(F("\t 'e': electrode config electrode enable set 'e12'"));
            // out.println(F("\t 'E': electrode config electrode enable get "));
            out.println();
            out.println(F("____________________________________________________________"));
        } break;
        case '!': {
            sketchinfo_print(out);
        } break;
        case 'y': {
            out.println(F("\t toggle DebugOut livesign Serial:"));
            debugOut_LiveSign_Serial_Enabled = !debugOut_LiveSign_Serial_Enabled;
            out.print(F("\t debugOut_LiveSign_Serial_Enabled:"));
            out.println(debugOut_LiveSign_Serial_Enabled);
        } break;
        case 'Y': {
            out.println(F("\t toggle DebugOut livesign LED:"));
            debugOut_LiveSign_LED_Enabled = !debugOut_LiveSign_LED_Enabled;
            out.print(F("\t debugOut_LiveSign_LED_Enabled:"));
            out.println(debugOut_LiveSign_LED_Enabled);
        } break;
        case 'x': {
            // get state
            out.println(F("__________"));
            out.println(F("Tests:"));

            out.println(F("nothing to do."));

            // uint16_t wTest = 65535;
            // uint16_t wTest = atoi(&command[1]);
            // out.print(F("wTest: "));
            // out.print(wTest);
            // out.println();
            //
            // out.print(F("1: "));
            // out.print((byte)wTest);
            // out.println();
            //
            // out.print(F("2: "));
            // out.print((byte)(wTest>>8));
            // out.println();
            //
            // out.println();

            // out.println(F("    bit test"));
            // out.print(F("      8 (1 << 3) "));
            // slight_DebugMenu::print_Binary_8(
            //     out,
            //     (0b1 << 3)
            // );
            // out.println();
            // out.print(F("      8 (1 << 7) "));
            // slight_DebugMenu::print_Binary_8(
            //     out,
            //     (0b1 << 7)
            // );
            // out.println();
            // out.print(F("     16 (1 << 3) "));
            // slight_DebugMenu::print_Binary_16(
            //     out,
            //     (0b1 << 3)
            // );
            // out.println();
            // out.print(F("     16 (1 << 15) "));
            // slight_DebugMenu::print_Binary_16(
            //     out,
            //     (0b1 << 15)
            // );
            // out.println();
            //
            // uint32_t u32 = 0;
            // out.print(F("     32 ((uint32_t)1 << 3) "));
            // u32 = ((uint32_t)1 << 3);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();
            // out.print(F("     32 (1 << 17) "));
            // u32 = ((uint32_t)1 << 17);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();
            // out.print(F("     32 (1 << 31) "));
            // u32 = ((uint32_t)1 << 31);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();
            // out.print(F("     32 (4,294,967,295) "));
            // u32 = (4294967295);
            // out.print(F(" : "));
            // out.print(u32, BIN);
            // out.print(F(" : "));
            // slight_DebugMenu::print_Binary_32(
            //     out,
            //     u32
            // );
            // out.println();

            // out.println(F("test set_register16bit_part: "));
            //
            //
            // uint16_t temp = 0b0001110000001111;
            //
            // out.println(F("original value: "));
            // slight_DebugMenu::print_Binary_16(out, temp);
            // // out.print(F("; "));
            // // out.print(temp, HEX);
            // // out.print(F("; "));
            // // out.print(temp, BIN);
            // // out.print(F("; "));
            // // out.print(temp);
            // out.println();
            //
            // // static uint16_t set_register16bit_part(
            // //     uint16_t reg_value,
            // //     uint16_t reg_mask,
            // //     uint8_t reg_shift,
            // //     uint8_t value
            // // );
            // out.println(F("set part"));
            // temp = slight_FDC1004::set_register16bit_part(
            //     temp,
            //     0b0001110000000000,
            //     10,
            //     0b001
            // );
            // out.println(F("result:"));
            // slight_DebugMenu::print_Binary_16(out, temp);
            // out.println();


            // temp = 0b0000000000111000;
            // out.print(F("0b0000000000111000: "));
            // slight_DebugMenu::print_Binary_16(out, temp);
            // out.print(F("; "));
            // out.print(temp, HEX);
            // out.print(F("; "));
            // out.print(temp, BIN);
            // out.print(F("; "));
            // out.print(temp);
            // out.println();



            out.println(F("__________"));
        } break;
        //---------------------------------------------------------------------
        case 'd': {
            mySensor_dump(out);
        } break;
        case 'i': {
            mySensor_measurement_init(out);
        } break;
        case 'r': {
            mySensor_measurement_read(out);
        } break;
        case 't': {
            mySensor_readwritetest(out);
        } break;
        case 'c': {
            mySensor_config(out);
            lcd.clear();
        } break;
        case 'R': {
            out.println(F("\t soft_reset_write()."));
            mySensor.soft_reset_write();
            out.print(F("\t soft_reset_read():"));
            boolean state = mySensor.soft_reset_read();
            out.print(state);
            out.println();
        } break;
        //---------------------------------------------------------------------
        case 'D': {
            mySensor_raw_dump(out);
        } break;
        // case 'r': {
        //     mySensor_raw_read_measurements(out);
        // } break;
        case 'C': {
            mySensor_raw_read_config(out);
        } break;
        // case 'c': {
        //     mySensor_raw_write_config(out);
        // } break;
        //---------------------------------------------------------------------
        // case 's': {
        //     out.print(F("\t sensitivity set "));
        //     // convert part of string to int
        //     // (up to first char that is not a number)
        //     uint8_t value = atoi(&command[1]);
        //     out.print(value);
        //     out.print(F(" = "));
        //     slight_FDC1004::sensitivity_t sens = mySensor.sensitivity_convert(value);
        //     mySensor.sensitivity_set(sens);
        //     mySensor.sensitivity_print(Serial, sens);
        //     Serial.println();
        // } break;
        // case 'S': {
        //     out.print(F("\t sensitivity get "));
        //     mySensor.sensitivity_print(Serial, mySensor.sensitivity_get());
        //     Serial.println();
        // } break;
        // case 'a': {
        //     out.print(F("\t auto_config_load_recommend_config: "));
        //     mySensor.auto_config_load_recommend_config(&out);
        //     out.println();
        // } break;
        // case 't': {
        //     out.print(F("\t touch status get: "));
        //     slight_DebugMenu::print_Binary_16(
        //         out,
        //         mySensor.touch_status_read()
        //     );
        //     out.println();
        // } break;
        // case 'p': {
        //     out.print(F("\t touch status get: "));
        //     slight_DebugMenu::print_Binary_16(
        //         out,
        //         mySensor.touch_status_read()
        //     );
        //     out.println();
        //     out.print(F("\t electrode config: "));
        //     slight_DebugMenu::print_Binary_8(
        //         out,
        //         mySensor.electrode_config_get()
        //     );
        //     out.println();
        //     out.print(F("\t\t electrode enable: "));
        //     mySensor.electrode_config_electrode_enable_print(out);
        //     out.println();
        //     out.print(F("\t\t proximity enable: "));
        //     mySensor.electrode_config_proximity_enable_print(out);
        //     out.println();
        //     out.print(F("\t\t baseline tracking: "));
        //     mySensor.electrode_config_baseline_tracking_print(out);
        //     out.println();
        // } break;
        // case 'c': {
        //     out.print(F("\t electrode config get: "));
        //     slight_DebugMenu::print_Binary_8(
        //         out,
        //         mySensor.electrode_config_get()
        //     );
        //     out.println();
        // } break;
        // case 'e': {
        //     out.print(F("\t electrode config electrode enable set: "));
        //     uint8_t value_raw = atoi(&command[1]);
        //     out.print(value_raw);
        //     out.print(F(" = "));
        //     // convert value
        //     slight_FDC1004::electrode_config_electrode_enable_t value =
        //         mySensor.electrode_config_electrode_enable_convert(
        //             value_raw
        //         );
        //     mySensor.electrode_config_electrode_enable_print(
        //         out,
        //         value
        //     );
        //     mySensor.electrode_config_electrode_enable_set(
        //         value
        //     );
        //     // slight_DebugMenu::print_Binary_8(
        //     //     out,
        //     //     mySensor.electrode_config_electrode_enable_get()
        //     // );
        //     out.println();
        // } break;
        // case 'E': {
        //     out.print(F("\t electrode config electrode enable get: "));
        //     slight_FDC1004::electrode_config_electrode_enable_t value =
        //         mySensor.electrode_config_electrode_enable_get();
        //     slight_DebugMenu::print_Binary_8(
        //         out,
        //         value
        //     );
        //     out.print(F(" = "));
        //     mySensor.electrode_config_electrode_enable_print(
        //         out,
        //         value
        //     );
        //     out.println();
        // } break;
        //---------------------------------------------------------------------
        default: {
            if(strlen(command) > 0) {
                out.print(F("command '"));
                out.print(command);
                out.println(F("' not recognized. try again."));
            }
            pInstance->get_command_input_pointer()[0] = '?';
            pInstance->set_flag_EOC(true);
        }
    } // end switch

    // end Command Parser
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// FDC1004

void mySensor_init(Print &out) {
    out.println(F("setup FDC1004:")); {
        out.println(F("\t connect to sensor ..."));
        bool ready = mySensor.begin();
        out.print("\t ");
        if (!ready) {
            out.print("not ");
        }
        out.print("found!");
        out.println();

        mySensor.update_interval_set(500);
        mySensor.sensor_event_set_callback(sensor_event);

    }
    out.println(F("\t finished."));
}

void sensor_event(slight_FDC1004 *instance) {
    // Serial.print(F("sensor_event: "));
    // for (size_t i=0; i<12; i++) {
    //     if (instance->touch_status_get() & (1 << i)) {
    //         Serial.print("1");
    //     } else {
    //         Serial.print("0");
    //     }
    // }
    // Serial.println();

    Print &out = Serial;

    lcd_update();
    out.println();

    // uint32_t temp = 0;
    //
    // out.print(F("1: "));
    // temp = instance->measurement_read(slight_FDC1004::MESA_1);
    // slight_DebugMenu::print_Binary_32(
    //     out,
    //     temp
    // );
    // out.print(F("; "));
    // slight_DebugMenu::print_uint32_align_right(
    //     out,
    //     temp
    // );
    // out.print(F("; "));
    //     // out.print(
    //     //     temp
    //     // );
    // // out.print(F("; "));
    // // out.print(
    // //     instance->capacitance_get(slight_FDC1004::MESA_1)
    // // );
    //
    //
    // lcd_update();
    //
    //
    // out.print(F("; "));
    // // out.print(
    // //     slight_FDC1004::convert_measurement_to_capacitance(temp, 0)
    // // );
    // out.println();

    // out.print(F("2: "));
    // temp = instance->measurement_read(slight_FDC1004::MESA_2);
    // slight_DebugMenu::print_Binary_32(
    //     out,
    //     temp
    // );
    // out.print(F("; "));
    // slight_DebugMenu::print_uint32_align_right(
    //     out,
    //     temp
    // );
    // out.print(F("; "));
    // out.print(
    //     temp
    // );
    // // out.print(F("; "));
    // // out.print(
    // //     instance->capacitance_get(slight_FDC1004::MESA_1)
    // // );
    // out.print(F("; "));
    // out.print(
    //     slight_FDC1004::convert_measurement_to_capacitance(temp, 0)
    // );
    // out.println();
}


void mySensor_dump(Print &out) {
    out.println(F("~~ dump all sensor data ~~"));

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    out.println(F("    read config register."));
    mySensor.fdc_config_read();

    out.print(F("      done: "));
    for (size_t i = 0; i < 4; i++) {
        out.print(mySensor.measurement_done_get(i));
        out.print(F(", "));
    }
    out.println();

    out.print(F("      init: "));
    for (size_t i = 0; i < 4; i++) {
        out.print(mySensor.measurement_done_get(i));
        out.print(F(", "));
    }
    out.println();

    out.print(F("      repeate: "));
    out.print(mySensor.measurement_repeate_get());
    out.println();
    out.print(F("      rate: "));
    out.print(mySensor.measurement_rate_get());
    out.print(F(" = "));
    mySensor.measurement_rate_print(out);
    out.println();
    out.print(F("      reset: "));
    out.print(mySensor.soft_reset_read());
    out.println();

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    out.println(F("    read measurement config (1..4)"));
    for (size_t i = 0; i < 4; i++) {
        mySensor.measurement_config_read(i);
        // uint16_t temp = mySensor.measurement_config_read(i);
        // out.print(F("    "));
        // out.print(i);
        // out.print(F(": "));
        // slight_DebugMenu::print_Binary_16(out, temp);
        // out.print(F("; "));
        // out.print(temp, HEX);
        // out.println();
    }
    out.println(  F("      NR      chA        chB  CAPDAC  uF"));
    //                     MESA_1  CIN1  DISABLED    255  16.00
    for (size_t i = 0; i < 4; i++) {
        out.print(F("      MESA_"));
        out.print(i+1);
        out.print(F("  "));
        mySensor.measurement_config_chA_print(out, i);
        out.print(F("  "));
        mySensor.measurement_config_chB_print(out, i, true);
        out.print(F("     "));
        slight_DebugMenu::print_uint8_align_right(
            out,
            mySensor.measurement_config_CAPDAC_get(i)
        );
        out.print(F("  "));
        out.print(
            mySensor.measurement_config_CAPDAC_get_capacitance(i)
        );
        // out.print(F("uF"));
        out.println();
    }
    out.println();

    // out.println(F("\t check if measurements are ready ..."));
    // uint8_t done = mySensor.done();
    //     out.print("\t ");
    //     if (!ready) {
    //         out.print("not ");
    //     }
    //     out.print("found!");
    //     out.println();

    // mySensor.sensor_event_set_callback(sensor_event);

    // out.println(F("\t finished."));
}

void mySensor_measurement_read(Print &out) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    out.println(F("    read measurement 1..4"));
    for (size_t i = 0; i < 4; i++) {
        uint32_t temp = mySensor.measurement_read(i);
        out.print(F("    "));
        out.print(i);
        out.print(F(": "));
        slight_DebugMenu::print_Binary_32(
            out,
            temp
        );
        out.print(F("; "));
        slight_DebugMenu::print_uint32_align_right(
            out,
            temp
        );
        out.print(F("; "));
        out.print(
            slight_FDC1004::convert_measurement_to_capacitance(temp, 0)
        );
        out.println();
    }
}

void mySensor_measurement_init(Print &out) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    out.println(F("\t init measurements: "));
    out.println(F("\t   MESA_1: "));

    // out.print(F("\t   o "));
    // slight_DebugMenu::print_Binary_16(
    //     out,
    //     mySensor.fdc_config_get()
    // );
    // Serial.println();

    mySensor.measurement_rate_set(slight_FDC1004::repeate_rate_100Ss);
    mySensor.measurement_repeate_set(false);
    mySensor.measurement_init(slight_FDC1004::MESA_2);

    // out.print(F("\t   s "));
    // slight_DebugMenu::print_Binary_16(
    //     out,
    //     mySensor.fdc_config_get()
    // );
    // Serial.println();

    mySensor.fdc_config_write();

    // out.print(F("\t   r "));
    // slight_DebugMenu::print_Binary_16(
    //     out,
    //     mySensor.fdc_config_read()
    // );
    // Serial.println();

    // out.println(F("    init measurement 1..4"));
    // for (size_t i = 0; i < 4; i++) {
    //     uint32_t temp = mySensor.measurement_read(i);
    //     out.print(F("    "));
    //     out.print(i);
    //     out.print(F(": "));
    //     slight_DebugMenu::print_uint32_align_right(
    //         out,
    //         temp
    //     );
    //     out.print(F("; "));
    //     out.print(
    //         slight_FDC1004::convert_measurement_to_capacitance(temp)
    //     );
    //     out.println();
    // }
}

void mySensor_config(Print &out) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    out.println(F("\t set config: "));

    // measurement configurations
    // MESA_1
    out.println(F("\t   MESA_1: (chA = CIN1; chB = DISABLED;) "));
    mySensor.measurement_config_chA_set(
        slight_FDC1004::MESA_1,
        slight_FDC1004::config_chA_CIN1
    );
    mySensor.measurement_config_chB_set(
        slight_FDC1004::MESA_1,
        // slight_FDC1004::config_chB_CIN2
        slight_FDC1004::config_chB_DISABLED
    );

    // out.println(F("\t   MESA_2: (chA = CIN2; chB = DISABLED;) "));
    // mySensor.measurement_config_chA_set(
    //     slight_FDC1004::MESA_1,
    //     slight_FDC1004::config_chA_CIN2
    // );
    // mySensor.measurement_config_chB_set(
    //     slight_FDC1004::MESA_1,
    //     slight_FDC1004::config_chB_DISABLED
    // );
    // mySensor.measurement_config_CAPDAC_set(
    //     slight_FDC1004::MESA_1,
    //     0
    // );

    // write config to device
    out.println(F("\t   write config to device."));
    mySensor.measurement_config_write(slight_FDC1004::MESA_1);
    // mySensor.measurement_config_write(slight_FDC1004::MESA_2);

    // fdc config
    // enable measurement 1
    out.println(F("\t   fdc config: (MESA_1 init=enabled; rate=100S/s; repeate=true;) "));
    mySensor.measurement_init_set(slight_FDC1004::MESA_1, true);
    // mySensor.measurement_init_set(slight_FDC1004::MESA_2, true);
    // set speed to 100S/s
    mySensor.measurement_rate_set(slight_FDC1004::repeate_rate_100Ss);
    // enable auto repeate
    mySensor.measurement_repeate_set(true);

    // write config to device
    mySensor.fdc_config_write();
    out.println(F("\t   write config to device."));
}

void mySensor_readwritetest(Print &out) {
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    out.println(F("\t read write test: "));
    out.println(F("\t   read MESA_1 config:"));
    out.println(
        mySensor.measurement_config_read(slight_FDC1004::MESA_1),
        HEX
    );
    out.println(F("\t   get MESA_1 config:"));
    out.println(
        mySensor.measurement_config_get(slight_FDC1004::MESA_1),
        HEX
    );
    out.println(F("\t   set MESA_1 config chA CIN2:"));
    mySensor.measurement_config_chA_set(
        slight_FDC1004::MESA_1,
        slight_FDC1004::config_chA_CIN2
    );
    out.println(F("\t   get MESA_1 config:"));
    out.println(
        mySensor.measurement_config_get(slight_FDC1004::MESA_1),
        HEX
    );
    out.println(F("\t   write MESA_1 config:"));
    mySensor.measurement_config_write(slight_FDC1004::MESA_1);
    out.println(F("\t   read MESA_1 config:"));
    out.println(
        mySensor.measurement_config_read(slight_FDC1004::MESA_1),
        HEX
    );

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// FDC1004 DEBUG
// direct I2C commands for communication

void mySensor_raw_dump(Print &out) {
    out.println();
    out.println(F("~~ dump all sensor data ~~"));

    uint16_t reg_value = 0;

    out.println(F("8.6.1.1 Capacitive Measurement Registers"));
    out.print(F(" REG_MEAS1_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS1_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS1_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS1_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS2_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS2_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS2_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS2_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS3_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS3_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS3_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS3_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS4_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS4_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS4_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS4_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();


    out.println(F("8.6.2 Measurement Configuration Registers"));
    out.print(F(" REG_MEAS1_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS1_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

    out.print(F(" REG_MEAS2_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS2_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

    out.print(F(" REG_MEAS3_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS3_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

    out.print(F(" REG_MEAS4_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS4_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();


    out.println(F("8.6.3 FDC Configuration Register"));
    out.print(F(" REG_FDC_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_FDC_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();


    out.println(F("8.6.4 Offset Calibration Registers"));
    out.print(F(" REG_CIN1_OFFSET_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN1_OFFSET_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();
    out.print(F(" REG_CIN2_OFFSET_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN2_OFFSET_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();
    out.print(F(" REG_CIN3_OFFSET_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN3_OFFSET_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();
    out.print(F(" REG_CIN4_OFFSET_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN4_OFFSET_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();


    out.println(F("8.6.5 Gain Calibration Registers"));
    out.print(F(" REG_CIN1_GAIN_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN1_GAIN_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();
    out.print(F(" REG_CIN2_GAIN_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN2_GAIN_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();
    out.print(F(" REG_CIN3_GAIN_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN3_GAIN_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();
    out.print(F(" REG_CIN4_GAIN_CALIBRATION "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_CIN4_GAIN_CALIBRATION);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();


    out.println(F("8.6.6 Manufacturer ID Register"));
    out.print(F(" REG_MANUFACTURER_ID "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MANUFACTURER_ID);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();


    out.println(F("8.6.7 Device ID Register"));
    out.print(F(" REG_DEVICE_ID "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_DEVICE_ID);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();
}

void mySensor_raw_read_config(Print &out) {
    out.println();
    out.println(F("~~ read config data ~~"));

    uint16_t reg_value = 0;

    out.println(F("8.6.2 Measurement Configuration Registers"));
    out.print(F(" REG_MEAS1_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS1_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

    out.print(F(" REG_MEAS2_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS2_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

    out.print(F(" REG_MEAS3_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS3_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

    out.print(F(" REG_MEAS4_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS4_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();


    out.println(F("8.6.3 FDC Configuration Register"));
    out.print(F(" REG_FDC_CONFIG "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_FDC_CONFIG);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

}

void mySensor_raw_read_measurements(Print &out) {
    out.println();
    out.println(F("~~ read measurements ~~"));

    uint16_t reg_value = 0;

    out.println(F("8.6.1.1 Capacitive Measurement Registers"));
    out.print(F(" REG_MEAS1_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS1_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS1_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS1_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS2_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS2_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS2_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS2_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS3_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS3_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS3_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS3_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS4_MSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS4_MSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();
    out.print(F(" REG_MEAS4_LSB "));
    reg_value = mySensor.read_register16bit(slight_FDC1004::REG_MEAS4_LSB);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.println();

}

void mySensor_raw_write_config(Print &out) {
    out.println();
    out.println(F("~~ configure sensor for simple test ~~"));

    uint16_t reg_value = 0;

    out.println(F("8.6.2 Measurement Configuration Registers"));
    out.print(F(" REG_MEAS1_CONFIG "));
    reg_value = 0b0001110000000000;
    mySensor.write_register16bit(slight_FDC1004::REG_MEAS1_CONFIG, reg_value);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();


    out.println(F("8.6.3 FDC Configuration Register"));
    out.print(F(" REG_FDC_CONFIG "));
    reg_value = 0b0000010010000000;
    mySensor.write_register16bit(slight_FDC1004::REG_FDC_CONFIG, reg_value);
    slight_DebugMenu::print_Binary_16(out, reg_value);
    out.print(F(" : "));
    out.print(reg_value, HEX);
    out.println();

}



//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// setup
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void setup() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise PINs

        //LiveSign
        pinMode(infoled_pin, OUTPUT);
        digitalWrite(infoled_pin, HIGH);

        // as of arduino 1.0.1 you can use INPUT_PULLUP

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // initialise serial

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // wait for arduino IDE to release all serial ports after upload.
            delay(2000);
        #endif

        Serial.begin(115200);

        // for ATmega32U4 devices:
        #if defined (__AVR_ATmega32U4__)
            // Wait for Serial Connection to be Opend from Host or
            // timeout after 6second
            uint32_t timeStamp_Start = millis();
            while( (! Serial) && ( (millis() - timeStamp_Start) < 6000 ) ) {
                // nothing to do
            }
        #endif

        Serial.println();

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // print welcome

        sketchinfo_print(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup lcd

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

        lcd_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // setup FDC1004

        Serial.print(F("# Free RAM = "));
        Serial.println(freeRam());

        mySensor_init(Serial);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // show serial commands

        myDebugMenu.set_callback(handleMenu_Main);
        myDebugMenu.begin(true);

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // go

        Serial.println(F("wait 1s."));
        delay(1000);
        Serial.println(F("Loop:"));

} /** setup **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// main loop
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void loop() {
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // menu input
        myDebugMenu.update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // TouchSensor things

        mySensor.update();

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // debug output

        if (
            (millis() - debugOut_LiveSign_TimeStamp_LastAction) >
            debugOut_LiveSign_UpdateInterval
        ) {
            debugOut_LiveSign_TimeStamp_LastAction = millis();

            if ( debugOut_LiveSign_Serial_Enabled ) {
                Serial.print(millis());
                Serial.print(F("ms;"));
                Serial.print(F("  free RAM = "));
                Serial.println(freeRam());
            }

            if ( debugOut_LiveSign_LED_Enabled ) {
                infoled_state = ! infoled_state;
                if (infoled_state) {
                    //set LED to HIGH
                    digitalWrite(infoled_pin, HIGH);
                } else {
                    //set LED to LOW
                    digitalWrite(infoled_pin, LOW);
                }
            }

        }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // other things

} /** loop **/

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// THE END
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
