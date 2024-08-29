/*
 * SPDX-FileCopyrightText: 2024 M5Stack Technology CO LTD
 *
 * SPDX-License-Identifier: MIT
 */
#include "Arduino.h"
#include "Wire.h"
#include "M5UnitJoystick2.h"

M5UnitJoystick2 joystick2;

void setup() {
    Serial.begin(115200);
    joystick2.begin(&Wire, JOYSTICK_ADDR, 21, 22);
    joystick2.setRgbColor(0x00ff00);
}

void loop() {
    uint8_t bootloader_ver, firmware_ver;
    uint16_t adc_x, adc_y;
    bootloader_ver = joystick2.getBootloaderVersion();
    firmware_ver   = joystick2.getFirmwareVersion();
    joystick2.getJoyADCValueXY16Bits(&adc_x, &adc_y);
    Serial.printf("bootloader ver:%d, firmware ver:%d, x adc:%d, y adc:%d\n", bootloader_ver, firmware_ver, adc_x,
                  adc_y);
    delay(100);
}