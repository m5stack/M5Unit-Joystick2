/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "M5UnitJoystick2.h"

void M5UnitJoystick2::writeBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
    _wire->beginTransmission(addr);
    _wire->write(reg);
    for (int i = 0; i < length; i++) {
        _wire->write(*(buffer + i));
    }
    _wire->endTransmission();
}

void M5UnitJoystick2::readBytes(uint8_t addr, uint8_t reg, uint8_t *buffer, uint8_t length) {
    uint8_t index = 0;
    _wire->beginTransmission(addr);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(addr, length);
    for (int i = 0; i < length; i++) {
        buffer[index++] = _wire->read();
    }
}

bool M5UnitJoystick2::begin(TwoWire *wire, uint8_t addr, uint8_t sda, uint8_t scl, uint32_t speed) {
    _wire  = wire;
    _addr  = addr;
    _sda   = sda;
    _scl   = scl;
    _speed = speed;
    _wire->begin(_sda, _scl);
    _wire->setClock(speed);
    delay(10);
    _wire->beginTransmission(_addr);
    uint8_t error = _wire->endTransmission();
    if (error == 0) {
        return true;
    } else {
        return false;
    }
}

uint16_t M5UnitJoystick2::getJoyADCValueX(adc_mode_t adc_bits) {
    uint8_t data[4];
    uint32_t value;

    if (adc_bits == _12bit) {
        uint8_t reg = JOY_ADC_VALUE_12BITS_REG;
        readBytes(_addr, reg, data, 2);
        value = data[0] | (data[1] << 8);
    } else if (adc_bits == _8bit) {
        uint8_t reg = JOY_ADC_VALUE_8BITS_REG;
        readBytes(_addr, reg, data, 1);
        value = data[0];
    }

    return value;
}

void M5UnitJoystick2::getJoyADCValueXY16Bits(uint16_t *adc_x, uint16_t *adc_y) {
    uint8_t data[4];

    uint8_t reg = JOY_ADC_VALUE_12BITS_REG;

    readBytes(_addr, reg, data, 4);

    memcpy((uint8_t *)adc_x, &data[0], 2);
    memcpy((uint8_t *)adc_y, &data[2], 2);
}

void M5UnitJoystick2::getJoyADCValueXY8Bits(uint8_t *adc_x, uint8_t *adc_y) {
    uint8_t data[4];

    uint8_t reg = JOY_ADC_VALUE_8BITS_REG;

    readBytes(_addr, reg, data, 2);

    *adc_x = data[0];
    *adc_y = data[1];
}

uint16_t M5UnitJoystick2::getJoyADCValueY(adc_mode_t adc_bits) {
    uint8_t data[4];
    uint32_t value;

    if (adc_bits == _12bit) {
        uint8_t reg = JOY_ADC_VALUE_12BITS_REG + 2;
        readBytes(_addr, reg, data, 2);
        value = data[0] | (data[1] << 8);
    } else if (adc_bits == _8bit) {
        uint8_t reg = JOY_ADC_VALUE_8BITS_REG + 1;
        readBytes(_addr, reg, data, 1);
        value = data[0];
    }

    return value;
}

int16_t M5UnitJoystick2::getJoyADC12bitsOffsetValueX(void) {
    int16_t value;

    readBytes(_addr, JOY_OFFSET_ADC_VALUE_12BITS_REG, (uint8_t *)&value, 2);

    return value;
}

int16_t M5UnitJoystick2::getJoyADC12bitsOffsetValueY(void) {
    int16_t value;

    readBytes(_addr, JOY_OFFSET_ADC_VALUE_12BITS_REG + 2, (uint8_t *)&value, 2);

    return value;
}

int8_t M5UnitJoystick2::getJoyADC8bitsOffsetValueX(void) {
    int8_t value;

    readBytes(_addr, JOY_OFFSET_ADC_VALUE_8BITS_REG, (uint8_t *)&value, 1);

    return value;
}

int8_t M5UnitJoystick2::getJoyADC8bitsOffsetValueY(void) {
    int8_t value;

    readBytes(_addr, JOY_OFFSET_ADC_VALUE_8BITS_REG + 1, (uint8_t *)&value, 1);

    return value;
}

void M5UnitJoystick2::setJoyADCValueCal(uint16_t x_neg_min, uint16_t x_neg_max, uint16_t x_pos_min, uint16_t x_pos_max,
                                        uint16_t y_neg_min, uint16_t y_neg_max, uint16_t y_pos_min,
                                        uint16_t y_pos_max) {
    uint8_t data[16];

    memcpy(&data[0], (uint8_t *)&x_neg_min, 2);
    memcpy(&data[2], (uint8_t *)&x_neg_max, 2);
    memcpy(&data[4], (uint8_t *)&x_pos_min, 2);
    memcpy(&data[6], (uint8_t *)&x_pos_max, 2);
    memcpy(&data[8], (uint8_t *)&y_neg_min, 2);
    memcpy(&data[10], (uint8_t *)&y_neg_max, 2);
    memcpy(&data[12], (uint8_t *)&y_pos_min, 2);
    memcpy(&data[14], (uint8_t *)&y_pos_max, 2);

    writeBytes(_addr, JOY_ADC_VALUE_CAL_REG, (uint8_t *)&data[0], 16);
}

void M5UnitJoystick2::getJoyADC12BitsValueCal(uint16_t *x_neg_min, uint16_t *x_neg_max, uint16_t *x_pos_min,
                                              uint16_t *x_pos_max, uint16_t *y_neg_min, uint16_t *y_neg_max,
                                              uint16_t *y_pos_min, uint16_t *y_pos_max) {
    uint8_t data[16];

    readBytes(_addr, JOY_ADC_VALUE_CAL_REG, data, 16);

    memcpy((uint8_t *)x_neg_min, &data[0], 2);
    memcpy((uint8_t *)x_neg_max, &data[2], 2);
    memcpy((uint8_t *)x_pos_min, &data[4], 2);
    memcpy((uint8_t *)x_pos_max, &data[6], 2);
    memcpy((uint8_t *)y_neg_min, &data[8], 2);
    memcpy((uint8_t *)y_neg_max, &data[10], 2);
    memcpy((uint8_t *)y_pos_min, &data[12], 2);
    memcpy((uint8_t *)y_pos_max, &data[14], 2);
}

uint8_t M5UnitJoystick2::getButtonValue(void) {
    uint8_t data[4];

    uint8_t reg = BUTTON_REG;
    readBytes(_addr, reg, data, 1);

    return data[0];
}

void M5UnitJoystick2::setRgbColor(uint32_t color) {
    writeBytes(_addr, RGB_REG, (uint8_t *)&color, 4);
}

uint32_t M5UnitJoystick2::getRgbColor(void) {
    uint32_t rgb_read_buff = 0;

    readBytes(_addr, RGB_REG, (uint8_t *)&rgb_read_buff, 4);

    return rgb_read_buff;
}

uint8_t M5UnitJoystick2::getBootloaderVersion(void) {
    _wire->beginTransmission(_addr);
    _wire->write(BOOTLOADER_VERSION_REG);
    _wire->endTransmission(false);

    uint8_t RegValue;

    _wire->requestFrom(_addr, 1);
    RegValue = Wire.read();
    return RegValue;
}

uint8_t M5UnitJoystick2::getI2CAddress(void) {
    _wire->beginTransmission(_addr);
    _wire->write(I2C_ADDRESS_REG);
    _wire->endTransmission(false);

    uint8_t RegValue;

    _wire->requestFrom(_addr, 1);
    RegValue = Wire.read();
    return RegValue;
}

uint8_t M5UnitJoystick2::setI2CAddress(uint8_t addr) {
    _wire->beginTransmission(_addr);
    _wire->write(I2C_ADDRESS_REG);
    _wire->write(addr);
    _addr = addr;
    if (_wire->endTransmission() == 0)
        return true;
    else
        return false;
}

uint8_t M5UnitJoystick2::getFirmwareVersion(void) {
    _wire->beginTransmission(_addr);
    _wire->write(FIRMWARE_VERSION_REG);
    _wire->endTransmission(false);

    uint8_t RegValue;

    _wire->requestFrom(_addr, 1);
    RegValue = Wire.read();
    return RegValue;
}
