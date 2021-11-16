#include "uop_msb.h"
#include <iostream>
using namespace uop_msb;

//Radians to degrees
const double alpha = 180.0/3.1415926;

// Motion Sensor
MotionSensor motion;

//On board LEDs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
 
//On board switch
DigitalIn BlueButton(USER_BUTTON);

//LCD Display
LCD_16X2_DISPLAY disp;

//Buzzer
Buzzer buzz;

//Traffic Lights
DigitalOut traf1RedLED(TRAF_RED1_PIN,1);
DigitalOut traf1YelLED(TRAF_YEL1_PIN);
DigitalOut traf1GrnLED(TRAF_GRN1_PIN);
DigitalInOut traf2RedLED(TRAF_RED2_PIN, PIN_OUTPUT, OpenDrainNoPull, 0);
DigitalInOut traf2YelLED(TRAF_YEL2_PIN, PIN_OUTPUT, OpenDrainNoPull, 1);
DigitalInOut traf2GrnLED(TRAF_GRN2_PIN, PIN_OUTPUT, OpenDrainNoPull, 1);

//Light Levels
AnalogIn ldr(AN_LDR_PIN);

//LCD Backlight
DigitalOut backLight(LCD_BKL_PIN);

//DIP Switches
DIPSwitches dipSwitches;

//Push Buttons
Buttons button;

//Environmental Sensor
EnvSensor env;

int main()
{
    // Interrogate Environmental Sensor Driver
    switch (env.getSensorType())
    {
        case EnvSensor::BMP280:
        printf("BMP280 (MSB v2)\n");
        break;
        case uop_msb::EnvSensor::SPL06_001:
        printf("SPL06_001 (MSB v4)\n");
        break;
        default:
        printf("Sensor not used?\n");
    }

    //Read DIP Switches
    int u = dipSwitches;
    printf("DIP as hex: %X\n", u);

    printf("DIP_A: %d\n", dipSwitches[0]);
    printf("DIP_B: %d\n", dipSwitches[1]);
    printf("DIP_C: %d\n", dipSwitches[2]);
    printf("DIP_D: %d\n", dipSwitches[3]);
 
    wait_us(3000000);

    //LCD Backlight ON
    backLight = 1;

    // Initial display
    disp.cls();
    disp.locate(0,0);
    disp.printf("LDR: %0.2f", ldr.read());    
    disp.locate(1, 0);
    disp.printf("%.2f %.1f", env.getTemperature(), env.getPressure());
    wait_us(200000); 



    while (true) {
        
        Motion_t acc   = motion.getAcceleration();   
        Motion_t gyr   = motion.getGyro();          
            
        //Temperature of sensor
        float tempMems = motion.getTemperatureC();  

        //Display sensor values
        printf("%8.3f,\t%8.3f,\t%8.3f,\t", acc.x, acc.y, acc.z); 
        printf("%8.3f,\t%8.3f,\t%8.3f,\t", gyr.x, gyr.y, gyr.z);         
        printf("%8.3f,\t\n",           tempMems); 
    
        //Loop time is influenced by the following
        wait_us(25000); 
    }
}

