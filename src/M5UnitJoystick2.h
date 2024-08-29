/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __M5UNITJOYSTICK2_H
#define __M5UNITJOYSTICK2_H

#include "Arduino.h"
#include "Wire.h"

#define JOYSTICK_ADDR                   0x63
#define JOY_ADC_VALUE_12BITS_REG        0x00
#define JOY_ADC_VALUE_8BITS_REG         0x10
#define BUTTON_REG                      0x20
#define RGB_REG                         0x30
#define JOY_ADC_VALUE_CAL_REG           0x40
#define JOY_OFFSET_ADC_VALUE_12BITS_REG 0x50
#define JOY_OFFSET_ADC_VALUE_8BITS_REG  0x60
#define FIRMWARE_VERSION_REG            0xFE
#define BOOTLOADER_VERSION_REG          0xFC
#define I2C_ADDRESS_REG                 0xFF
#define JUMP_TO_BOOTLOADER_REG          0xFD

typedef enum { _8bit = 0, _12bit } adc_mode_t;

class M5UnitJoystick2 {
   private:
    uint8_t _addr;
    TwoWire *_wire;
    uint8_t _scl;
    uint8_t _sda;
    uint32_t _speed;
    void writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);
    void readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length);

   public:
    bool begin(TwoWire *wire = &Wire, uint8_t addr = JOYSTICK_ADDR, uint8_t sda = 21, uint8_t scl = 22,
               uint32_t speed = 400000UL);
    uint8_t setI2CAddress(uint8_t addr);
    uint8_t getI2CAddress(void);
    uint8_t getFirmwareVersion(void);
    uint8_t getBootloaderVersion(void);
    uint16_t getJoyADCValueX(adc_mode_t adc_bits);
    uint16_t getJoyADCValueY(adc_mode_t adc_bits);
    uint8_t getButtonValue(void);
    void setRgbColor(uint32_t color);
    uint32_t getRgbColor(void);
    void getJoyADC12BitsValueCal(uint16_t *x_neg_min, uint16_t *x_neg_max, uint16_t *x_pos_min, uint16_t *x_pos_max,
                                 uint16_t *y_neg_min, uint16_t *y_neg_max, uint16_t *y_pos_min, uint16_t *y_pos_max);
    void setJoyADCValueCal(uint16_t x_neg_min, uint16_t x_neg_max, uint16_t x_pos_min, uint16_t x_pos_max,
                           uint16_t y_neg_min, uint16_t y_neg_max, uint16_t y_pos_min, uint16_t y_pos_max);
    int16_t getJoyADC12bitsOffsetValueX(void);
    int16_t getJoyADC12bitsOffsetValueY(void);
    int8_t getJoyADC8bitsOffsetValueX(void);
    int8_t getJoyADC8bitsOffsetValueY(void);
    void getJoyADCValueXY16Bits(uint16_t *adc_x, uint16_t *adc_y);
    void getJoyADCValueXY8Bits(uint8_t *adc_x, uint8_t *adc_y);
};

#endif
