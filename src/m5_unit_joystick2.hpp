/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __M5_UNIT_JOYSTICK2_HPP
#define __M5_UNIT_JOYSTICK2_HPP

#include "Arduino.h"
#include "Wire.h"

#define JOYSTICK2_ADDR                        0x63
#define JOYSTICK2_ADC_VALUE_12BITS_REG        0x00
#define JOYSTICK2_ADC_VALUE_8BITS_REG         0x10
#define JOYSTICK2_BUTTON_REG                  0x20
#define JOYSTICK2_RGB_REG                     0x30
#define JOYSTICK2_ADC_VALUE_CAL_REG           0x40
#define JOYSTICK2_OFFSET_ADC_VALUE_12BITS_REG 0x50
#define JOYSTICK2_OFFSET_ADC_VALUE_8BITS_REG  0x60
#define JOYSTICK2_FIRMWARE_VERSION_REG        0xFE
#define JOYSTICK2_BOOTLOADER_VERSION_REG      0xFC
#define JOYSTICK2_I2C_ADDRESS_REG             0xFF

typedef enum { _8bit = 0, _12bit } adc_mode_t;

class M5UnitJoystick2 {
public:
    bool begin(TwoWire *wire = &Wire, uint8_t addr = JOYSTICK2_ADDR, uint8_t sda = 21, uint8_t scl = 22,
               uint32_t speed = 400000UL);
    uint8_t set_i2c_address(uint8_t addr);
    uint8_t get_i2c_address(void);
    uint8_t get_firmware_version(void);
    uint8_t get_bootloader_version(void);
    uint16_t get_joy_adc_value_x(adc_mode_t adc_bits);
    uint16_t get_joy_adc_value_y(adc_mode_t adc_bits);
    uint8_t get_button_value(void);
    void set_rgb_color(uint32_t color);
    uint32_t get_rgb_color(void);
    void get_joy_adc_12bits_value_cal(uint16_t *x_neg_min, uint16_t *x_neg_max, uint16_t *x_pos_min,
                                      uint16_t *x_pos_max, uint16_t *y_neg_min, uint16_t *y_neg_max,
                                      uint16_t *y_pos_min, uint16_t *y_pos_max);
    void set_joy_adc_value_cal(uint16_t x_neg_min, uint16_t x_neg_max, uint16_t x_pos_min, uint16_t x_pos_max,
                               uint16_t y_neg_min, uint16_t y_neg_max, uint16_t y_pos_min, uint16_t y_pos_max);
    int16_t get_joy_adc_12bits_offset_value_x(void);
    int16_t get_joy_adc_12bits_offset_value_y(void);
    int8_t get_joy_adc_8bits_offset_value_x(void);
    int8_t get_joy_adc_8bits_offset_value_y(void);
    void get_joy_adc_16bits_value_xy(uint16_t *adc_x, uint16_t *adc_y);
    void get_joy_adc_8bits_value_xy(uint8_t *adc_x, uint8_t *adc_y);

private:
    uint8_t _addr;
    TwoWire *_wire;
    uint8_t _scl;
    uint8_t _sda;
    uint32_t _speed;
    void write_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);
    void read_bytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);
};

#endif
