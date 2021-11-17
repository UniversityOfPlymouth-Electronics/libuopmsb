#include "uop_msb.h"
#include <iostream>
using namespace uop_msb;

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
    //LCD Backlight ON
    backLight = 1;

    // Initial display
    disp.cls();
    disp.locate(0,0);
    disp.printf("MSB v%d", MSB_VER);    
    disp.locate(1, 0);

    // Interrogate Environmental Sensor Driver
    switch (env.getSensorType())
    {
        case EnvSensor::BMP280:
        disp.printf("BMP280\n");
        break;
        case uop_msb::EnvSensor::SPL06_001:
        disp.printf("SPL06_001\n");
        break;
        default:
        disp.printf("ERROR");
    }

    wait_us(2000000); 

    while (true) {
        

        // TEST MEMS SENSORS
        disp.cls();
        disp.locate(0,0);
        disp.printf("Testing MEMS");

        for (uint16_t n = 0; n<20; n++) {
            Motion_t acc   = motion.getAcceleration();   
            Motion_t gyr   = motion.getGyro();             

            //Temperature of sensor
            float tempMems = motion.getTemperatureC();  

            //Display sensor values
            printf("%8.3f,\t%8.3f,\t%8.3f,\t", acc.x, acc.y, acc.z); 
            printf("%8.3f,\t%8.3f,\t%8.3f,\t", gyr.x, gyr.y, gyr.z);         
            printf("%8.3f\n",             tempMems); 
        
            //Loop time is influenced by the following
            wait_us(500000);             
        }

        // TEST ENV SENSOR
        disp.cls();
        disp.locate(0,0);
        disp.printf("Testing %s", (MSB_VER == 2) ? "BMP280" : "SPL06_001");
        for (uint16_t n = 0; n < 20; n++) {
            float temp = env.getTemperature();
            float pres = env.getPressure();
            float hum = env.getHumidity();
            float lux = ldr.read();
            printf("T=%.2f, P=%.2f, H=%.2f, L=%.2f\n", temp, pres, hum, lux);   
            wait_us(500000);         
        }

        //Read DIP Switches (if fitted)
        #if MSB_VER != 2
        int u = dipSwitches;
        printf("DIP as hex: %X\n", u);

        printf("DIP_A: %d\n", dipSwitches[0]);
        printf("DIP_B: %d\n", dipSwitches[1]);
        printf("DIP_C: %d\n", dipSwitches[2]);
        printf("DIP_D: %d\n", dipSwitches[3]);
    
        wait_us(3000000);
        #endif        

    }
}

