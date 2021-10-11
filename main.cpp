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

    //Mean Gyro Values
    Motion_t gyrMean   = motion.getGyro();
    Motion_t accMean   = motion.getAcceleration();
    bool isFirstRun = true;

    while (true) {
        
        if (isFirstRun) {
            printf("CALIBRATING\n\r");
            led1 = 1; led2 = 1; led3 = 1;
            gyrMean = {0.0, 0.0, 0.0};
            for (uint32_t n=0; n<1000; n++) {
                Motion_t gyr   = motion.getGyro();
                gyrMean.x += gyr.x;
                gyrMean.y += gyr.y;
                gyrMean.z += gyr.z;
                wait_us(1000);
            }
            gyrMean.x /= 1000.0;
            gyrMean.y /= 1000.0;
            gyrMean.z /= 1000.0;
            isFirstRun = false;
            
            //Terminal header
            printf("%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s\n\r", "acc_x","acc_y","acc_z","gyr_x","gyr_y","gyr_z","temp");
            continue;
        }

        //Environmental sensor
        led1 = 1; led2 = 1; led3 = 1;

        disp.cls();
        disp.locate(0,0);
        disp.printf("LDR: %0.3f", ldr.read());    
        disp.locate(1, 0);
        disp.printf("%4.2f %5.1f", env.getTemperature(), env.getPressure());
        wait_us(200000); 

        //MEMS
        led1 = 0; led2 = 0; led3 = 0;  

        Motion_t acc   = motion.getAcceleration();
        Motion_t gyr   = motion.getGyro(); gyr.x -= gyrMean.x; gyr.y -= gyrMean.y; gyr.z -= gyrMean.z; 
        float tempMems = motion.getTemperatureC();

        printf("%8.3f,\t%8.3f,\t%8.3f,\t", acc.x, acc.y, acc.z); 
        printf("%8.3f,\t%8.3f,\t%8.3f,\t", gyr.x, gyr.y, gyr.z);         
        printf("%8.3f\n\r",             tempMems); 
    
        wait_us(200000); 
    }
}

