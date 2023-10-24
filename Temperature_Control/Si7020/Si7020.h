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

#ifndef _SI7020_H_
#define _SI7020_H_

#include "mbed.h"

/**
 * Silicon Labs Si7020 Humidity and Temperature Sensor
 *
 * @code
 * #include <stdio.h>
 * #include "mbed.h"
 * #include "Si7020.h"
 * 
 * I2C i2c(I2C_SDA, I2C_SCL);
 * Si7020 si(&i2c);
 * 
 * int main()
 * {
 *     while(1) {
 * 
 *         float humid;
 *         if(si.getHumidity(&humid) != 0) {
 *             printf("Error getting humidity\n");
 *             humid = -1;
 *         }
 * 
 *         float temp;
 *         if(si.getTemperature(&temp) != 0) {
 *             printf("Error getting temperature");
 *             temp = -1;
 *         } 
 *         printf("Humidity = %f%% Temperature = %fC\n", humid, temp);
 * 
 *         wait(1);
 *     }
 * }
 * @endcode
 */
class Si7020
{

public:

    /**
     * @brief   Measurement resolution.
     * @details Controls the resolution of the humidity and temperarure readings.
     */
    typedef enum {
        RH_12b_TEMP_14b = 0x0, ///< 12 bits for RH, 14 bits for Temp
        RH_8b_TEMP_12b = 0x1,  ///< 8 bits for RH, 12 bits for Temp
        RH_10b_TEMP_13b = 0x2, ///< 10 bits for RH, 13 bits for Temp
        RH_11b_TEMP_11b = 0x3, ///< 11 bits for RH, 11 bits for Temp
    } resolution_t;

    /**
     * Si7020 constructor.
     *
     * @param sda mbed pin to use for SDA line of I2C interface.
     * @param scl mbed pin to use for SCL line of I2C interface.
     */
    Si7020(PinName sda, PinName scl);

    /**
     * Si7020 constructor.
     *
     * @param i2c I2C object to use
     */
    Si7020(I2C *i2c);

    /**
     * Si7020 destructor.
     */
    ~Si7020();

    /**
     * @brief   Reset.
     * @details Sends the rest command.
     * @returns 0 if no errors, -1 if error.
     */
    int reset(void);

    /**
     * @brief   Get Electronic ID.
     * @details Gets the Electronic ID of the connected device. Verifies the 
     *          ID with an 8-bit CRC.
     *
     * @param   Character buffer to store the id. Needs to be at least 8 bytes.
     * @returns 0 if no errors, -1 if error.
     */
    int getElectronicId(char *id);

    /**
     * @brief   Configure sample resolution.
     * @details Sets the number of bits used for humidity and temperature readings.
     * @param   resolution Enum for the resolution setting.
     * @returns 0 if no errors, -1 if error.
     */
    int configResolution(Si7020::resolution_t resolution);

    /**
     * @brief   Get temperature reading.
     * @details Initiates a temperature reading and blocks until reading has
     *          been calculated. 
     * 
     * @note    Will hold the I2C bus until reading is complete. Refer to datasheet
     *          for timing specifications.
     * 
     * @param   tempCx10 Pointer for temperature reading. Unit is 1/10th degree Celcius.
     * @returns 0 if no errors, -1 if error.
     */
    int getTemperature(int16_t *tempCx10);

    /**
     * @brief   Get temperature reading.
     * @details Initiates a temperature reading and blocks until reading has
     *          been calculated. 
     * 
     * @note    Will hold the I2C bus until reading is complete. Refer to datasheet
     *          for timing specifications.
     * 
     * @param   tempC Pointer for temperature reading. Unit is degree Celcius.
     * @returns 0 if no errors, -1 if error.
     */
    int getTemperature(float *tempC);

    /**
     * @brief   Start temperature reading.
     * @details Initiates a temperature reading. Will not hold the bus or block.
     * @returns 0 if no errors, -1 if error.
     */
    int startTemperature(void);

    /**
     * @brief   Check temperature reading.
     * @details Checks to see if the temperature reading has been completed.
                Returns temperature if reading complete.
     * @note    Must call startTemperature() prior to calling this function. 
     * @param   tempCx10 Pointer for temperature reading. Unit is 1/10th degree Celcius.
     * @returns 0 if reading taken, -1 if reading pending.
     */
    int checkTemperature(int16_t *tempCx10);

    /**
     * @brief   Check temperature reading.
     * @details Checks to see if the temperature reading has been completed.
                Returns temperature if reading complete.
     * @note    Must call startTemperature() prior to calling this function. 
     * @param   tempC Pointer for temperature reading. Unit is degree Celcius.
     * @returns 0 if reading taken, -1 if reading pending.
     */
    int checkTemperature(float *tempC);

    /**
     * @brief   Get humidity reading.
     * @details Initiates a humidity reading and blocks until reading has
     *          been calculated.
     *
     * @note    Will hold the I2C bus until reading is complete. Refer to datasheet
     *          for timing specifications.
     * 
     * @param   humidx10 Pointer for humidity reading. Unit is 1/10th percent.
     * @returns 0 if no errors, -1 if error.
     */
    int getHumidity(int16_t *humidx10);

    /**
     * @brief   Get humidity reading.
     * @details Initiates a humidity reading and blocks until reading has
     *          been calculated.
     *
     * @note    Will hold the I2C bus until reading is complete. Refer to datasheet
     *          for timing specifications.
     * 
     * @param   humid Pointer for humidity reading. Unit is percent.
     * @returns 0 if no errors, -1 if error.
     */
    int getHumidity(float *humid);

    /**
     * @brief   Start humidity reading.
     * @details Initiates a humidity reading. Will not hold the bus or block.
     * @returns 0 if no errors, -1 if error.
     */
    int startHumidity(void);

    /**
     * @brief   Check humidity reading.
     * @details Checks to see if the humidity reading has been completed.
                Returns humidity if reading complete.

     * @note    Must call startHumidity() prior to calling this function. 
     * @param   humidCx10 Pointer for humidity reading. Unit is 1/10th percent.
     * @returns 0 if reading taken, -1 if reading pending.
     */
    int checkHumidity(int16_t *humidx10);

    /**
     * @brief   Check humidity reading.
     * @details Checks to see if the humidity reading has been completed.
                Returns humidity if reading complete.

     * @note    Must call startHumidity() prior to calling this function. 
     * @param   humid Pointer for humidity reading. Unit is percent.
     * @returns 0 if reading taken, -1 if reading pending.
     */
    int checkHumidity(float *humid);

    /**
     * @brief   Get temperature from humidity reading.
     * @details Gets temperature reading from previous humidity reading.
     * @note    Must call startHumidity() prior to calling this function. 
     * @param   tempC Pointer for temperature reading. Unit is degree Celcius.
     * @returns 0 if reading taken, -1 if reading pending.
     */
    int getPrevTemperature(float* tempC);

    /**
     * @brief   Get temperature from humidity reading.
     * @details Gets temperature reading from previous humidity reading.
     * @note    Must call startHumidity() prior to calling this function. 
     * @param   tempCx10 Pointer for temperature reading. Unit is 1/10th degree Celcius.
     * @returns 0 if reading taken, -1 if reading pending.
     */
    int getPrevTemperature(int16_t *tempCx10);

    /**
     * @brief   Get firmware revision.
     * @details Reads the firmware revision, refer to datasheet for codes.
     * @param   rev Pointer to store firmware revision.
     * @returns 0 if no errors, -1 if error.
     */
    int getRev(char *rev);

    /**
     * @brief   Control heater.
     * @details Enable or disable the heater.
     * @param   enable True to enable heater, false to disable.
     * @returns 0 if no errors, -1 if error.
     */
    int heater(bool enable);

private:

    char crc8(char value, char seed);
    I2C *i2c_;
    bool i2c_owner;
};

#endif /* _SI7020_H_ */
