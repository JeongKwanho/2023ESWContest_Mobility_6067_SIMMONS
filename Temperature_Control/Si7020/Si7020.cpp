/*******************************************************************************
 * Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

#include "Si7020.h"

/***** Definitions *****/
#define I2C_ADDR    (0x80)  // 1000000x
#define POLYVAL     (0x131)

/***** File Scope Data *****/
static const char cmd_meas_humid[] = { 0xE5 };
static const char cmd_meas_humid_no_hold[] = { 0xF5 };
static const char cmd_meas_temp[] = { 0xE3 };
static const char cmd_meas_temp_no_hold[] = { 0xF3 };
static const char cmd_meas_prev_temp[] = { 0xE0 };
static const char cmd_rst[] = { 0xFE };
static const char cmd_write_user1[] = { 0xE6 };
static const char cmd_read_user1[] = { 0xE7 };
static const char cmd_id_1[] = { 0xFA, 0x0F };
static const char cmd_id_2[] = { 0xFC, 0xC9 };
static const char cmd_fw_ver[] = { 0x84, 0xB8 };

//******************************************************************************
Si7020::Si7020(PinName sda, PinName scl)
{
    i2c_ = new I2C(sda, scl);
    i2c_owner = true;

    // 400KHz, as specified by the datasheet.
    i2c_->frequency(400000);
}

//******************************************************************************
Si7020::Si7020(I2C *i2c) :
    i2c_(i2c)
{
    i2c_owner = false;
}

//******************************************************************************
Si7020::~Si7020()
{
    if(i2c_owner) {
        delete i2c_;
    }
}

//******************************************************************************
int Si7020::reset(void)
{
    if (i2c_->write(I2C_ADDR, cmd_rst, sizeof(cmd_rst)) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int Si7020::getElectronicId(char* id)
{
    // Send cmd with repeated start
    if (i2c_->write(I2C_ADDR, cmd_id_1, sizeof(cmd_id_1), true) != 0) {
        return -1;
    }

    // Read first portion of ID
    char temp[8];
    if (i2c_->read(I2C_ADDR, temp, 8, false) != 0) {
        return -1;
    }

    // Check the CRC
    char crc = 0;
    int i;
    for(i = 0; i < 4; i++) {
        crc = crc8(temp[2*i], crc);
        if(crc != temp[2*i +1]) {
            return -1;
        }
    }

    // Save top portion of ID
    id[7] = temp [0];
    id[6] = temp [2];
    id[5] = temp [4];
    id[4] = temp [6];

    // Send cmd with repeated start
    if (i2c_->write(I2C_ADDR, cmd_id_2, sizeof(cmd_id_2), true) != 0) {
        return -1;
    }

    // Read rest of ID
    if (i2c_->read(I2C_ADDR, temp, 6, false) != 0) {
        return -1;
    }

    // Check the CRC
    crc = 0;
    for(i = 0; i < 2; i++) {
        crc = crc8(temp[3*i], crc);
        crc = crc8(temp[3*i + 1], crc);
        if(crc != temp[3*i + 2]) {
            return -1;
        }
    }

    // Save bottom portion of ID
    id[3] = temp [0];
    id[2] = temp [1];
    id[1] = temp [3];
    id[0] = temp [4];

    return 0;
}

//******************************************************************************
int Si7020::configResolution(Si7020::resolution_t resolution)
{
    char data[2];

    if (i2c_->write(I2C_ADDR, cmd_read_user1, sizeof(cmd_read_user1)) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, &data[1], 1) != 0) {
        return -1;
    }

    switch (resolution) {
        case RH_12b_TEMP_14b:
            data[1] &= ~0x81;
            break;
        case RH_8b_TEMP_12b:
            data[1] = (data[1] & ~0x80) | 0x01;
            break;
        case RH_10b_TEMP_13b:
            data[1] = (data[1] & ~0x01) | 0x80;
            break;
        case RH_11b_TEMP_11b:
            data[1] |= 0x81;
            break;
        default:
            return -1;
    }

    data[0] = cmd_write_user1[0];

    if (i2c_->write(I2C_ADDR, data, 2) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int Si7020::getTemperature(int16_t *tempCx10)
{
    if (i2c_->write(I2C_ADDR, cmd_meas_temp, sizeof(cmd_meas_temp)) != 0) {
        return -1;
    }

    return checkTemperature(tempCx10);
}

//******************************************************************************
int Si7020::getTemperature(float *tempC)
{
    if (i2c_->write(I2C_ADDR, cmd_meas_temp, sizeof(cmd_meas_temp)) != 0) {
        return -1;
    }

    return checkTemperature(tempC);
}

//******************************************************************************
int Si7020::startTemperature(void)
{
    if (i2c_->write(I2C_ADDR, cmd_meas_temp_no_hold, sizeof(cmd_meas_temp_no_hold)) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int Si7020::checkTemperature(int16_t *tempCx10)
{
    char data[3];
    uint16_t code;
    int temp;

    if (i2c_->read(I2C_ADDR, data, 3) != 0) {
        return -1;
    }

    // Get 16-bit value from bytes read
    code = ((uint16_t)data[0] << 8) + data[1];

    // Calculate the temperature using the formula from the datasheet
    // Scaled by 100
    temp = ((((int)17572 * code) + 0x8000) >> 16) - 4685;

    // Return value is to be scaled by 10
    *tempCx10 = (temp + 5) / 10;
    
    return 0;
}

//******************************************************************************
int Si7020::checkTemperature(float *tempC)
{
    char data[3];
    uint16_t code;

    if (i2c_->read(I2C_ADDR, data, 3) != 0) {
        return -1;
    }

    // Get 16-bit value from bytes read
    code = ((uint16_t)data[0] << 8) + data[1];

    // Calculate the temperature using the formula from the datasheet
    // Scaled by 100
    *tempC = ((175.72f * (float)code) / 65536.0f) - 46.85f;

    return 0;
}

//******************************************************************************
int Si7020::getHumidity(int16_t *humidx10)
{
    if (i2c_->write(I2C_ADDR, cmd_meas_humid, sizeof(cmd_meas_humid), true) != 0) {
        return -1;
    }

    return checkHumidity(humidx10);
}

//******************************************************************************
int Si7020::getHumidity(float *humid)
{
    if (i2c_->write(I2C_ADDR, cmd_meas_humid, sizeof(cmd_meas_humid)) != 0) {
        return -1;
    }

    return checkHumidity(humid);
}

//******************************************************************************
int Si7020::startHumidity(void)
{
    if (i2c_->write(I2C_ADDR, cmd_meas_humid_no_hold, sizeof(cmd_meas_humid_no_hold)) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int Si7020::checkHumidity(int16_t *humidx10)
{
    char data[3];
    uint16_t code;

    if (i2c_->read(I2C_ADDR, data, 3) != 0) {
        return -1;
    }

    // Get 16-bit value from bytes read
    code = ((uint16_t)data[0] << 8) + data[1];

    // Calculate the humidity using the formula from the datasheet
    // Scaled by 10
    *humidx10 = ((((int)1250 * code) + 0x8000) >> 16) - 60;

    // Check the crc
    char crc = crc8(data[0], 0x00);
    crc = crc8(data[1], crc);
    if(crc != data[2]) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int Si7020::checkHumidity(float *humid)
{
    char data[3];
    uint16_t code;

    if (i2c_->read(I2C_ADDR, data, 3) != 0) {
        return -1;
    }

    // Get 16-bit value from bytes read
    code = ((uint16_t)data[0] << 8) + data[1];

    // Calculate the humidity using the formula from the datasheet
    *humid = ((125.0f * (float)code) / 65536.0f) - 6.0f;

    return 0;
}

//******************************************************************************
int Si7020::getPrevTemperature(float *tempC) {

    if (i2c_->write(I2C_ADDR, cmd_meas_prev_temp, sizeof(cmd_meas_prev_temp)) != 0) {
        return -1;
    }

    if(checkTemperature(tempC) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int Si7020::getPrevTemperature(int16_t *tempCx10) {

    if (i2c_->write(I2C_ADDR, cmd_meas_prev_temp, sizeof(cmd_meas_prev_temp)) != 0) {
        return -1;
    }

    if(checkTemperature(tempCx10) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
int Si7020::getRev(char *rev)
{
    if (i2c_->write(I2C_ADDR, cmd_fw_ver, sizeof(cmd_fw_ver)) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, rev, 1) != 0) {
        return -1;
    }
    
    return 0;
}

//******************************************************************************
int Si7020::heater(bool enable)
{
    char data[2];

    if (i2c_->write(I2C_ADDR, cmd_read_user1, sizeof(cmd_read_user1)) != 0) {
        return -1;
    }

    if (i2c_->read(I2C_ADDR, &data[1], 1) != 0) {
        return -1;
    }

    if (enable) {
        data[1] |= 0x04;
    } else {
        data[1] &= ~0x04;
    }

    data[0] = cmd_write_user1[0];

    if (i2c_->write(I2C_ADDR, data, 2) != 0) {
        return -1;
    }

    return 0;
}

//******************************************************************************
char Si7020::crc8(char value, char seed)
{
    int i;

    for (i = 0; i < 8; i++) {

        if ((value ^ seed) & 0x80) {
            seed <<= 1;
            seed ^= POLYVAL;
        } else {
            seed <<= 1;
        }

        value <<= 1;
    }

    return seed;
}
