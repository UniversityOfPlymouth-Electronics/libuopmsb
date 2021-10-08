/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "MPU6050.h"



// Blinking rate in milliseconds
#define BLINKING_RATE     500ms

 MPU6050 mpu6050;

int main()
{
    //Set up I2C
    i2c.frequency(400000);  // use fast (400 kHz) I2C 
    wait_us(1000);
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    printf("I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x68\n\r");

    // Initialise the digital pin LED1 as an output
    DigitalOut led(LED1);

    while (true) {
        led = !led;
        ThisThread::sleep_for(BLINKING_RATE);
    }
}
