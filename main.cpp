//#include "mbed.h"
#include "uop_msb.h"
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

    while (true) {

        //Environmental sensor
        led1 = 1; led2 = 1; led3 = 1;

        disp.cls();
        disp.locate(0,0);
        disp.printf("LDR: %0.3f", ldr.read());    
        disp.locate(1, 0);
        disp.printf("%4.2f %5.1f", env.getTemperature(), env.getPressure());
        wait_us(1000000); 

        //MEMS
        led1 = 0; led2 = 0; led3 = 0;  

        Motion_t acc   = motion.getAcceleration();
        Motion_t gyr   = motion.getGyro();
        float tempMems = motion.getTemperatureC();

        printf("ax = %f\tay = %f\taz = %f\n",       acc.x, acc.y, acc.z); 
        printf("gx = %f\tgy = %f\tgz = %f deg/s\n", gyr.x, gyr.y, gyr.z);         
        printf("Temp (MPU6050) = %4.1f degC\n\r",   tempMems); 
    
        wait_us(1000000); 
    }
}

