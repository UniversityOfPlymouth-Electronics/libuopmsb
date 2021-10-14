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

    //Mean Gyro Values
    Motion_t gyrMean   = motion.getGyro();
    bool isFirstRun = true;

    //Integrated Gyro Values
    Motion_t gyrAngle = {.x=0.0, .y=0.0, .z = 0.0};
    Motion_t gyrPrev  = {.x=0.0, .y=0.0, .z = 0.0};    

    // Initial display
    disp.cls();
    disp.locate(0,0);
    disp.printf("LDR: %0.2f", ldr.read());    
    disp.locate(1, 0);
    disp.printf("%.2f %.1f", env.getTemperature(), env.getPressure());
    wait_us(200000); 

    // Timer
    Timer tmr;
    tmr.start();
    std::chrono::microseconds now = 0us;
    std::chrono::microseconds prev = 0us;


    while (true) {
        
        //On first execution, calibrate the gyros
        if (isFirstRun) {
            printf("CALIBRATING\n\r");
            led1 = 1; led2 = 1; led3 = 1;
            gyrMean  = {0.0, 0.0, 0.0};
            gyrAngle = gyrMean;
            gyrPrev  = gyrMean;

            for (uint32_t n=0; n<100; n++) {
                Motion_t gyr   = motion.getGyro();
                gyrMean.x += gyr.x; gyrMean.y += gyr.y; gyrMean.z += gyr.z;
                wait_us(10000);
            }
            gyrMean.x /= 100.0; gyrMean.y /= 100.0; gyrMean.z /= 100.0;
            isFirstRun = false;
            
            //Terminal header
            printf("%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s,\t%8s\n\r", "acc_x","acc_y","acc_z","gyr_x","gyr_y","gyr_z","temp", "intX", "intY", "intZ", "T");
            disp.cls();

            continue;
        }

        //Environmental sensor
        led1 = 1; led2 = 1; led3 = 1;

        //MEMS
        led1 = 0; led2 = 0; led3 = 0;  
 
        Motion_t acc   = motion.getAcceleration();   
        Motion_t gyr   = motion.getGyro();          gyr.x -= gyrMean.x; gyr.y -= gyrMean.y; gyr.z -= gyrMean.z; 
        
        //Measure sampling time
        now = tmr.elapsed_time();
        double deltaT = (double)1.0e-6*(now-prev).count();
        prev = now;        

        //Temperature of sensor
        float tempMems = motion.getTemperatureC();  

        //Display sensor values
        printf("%8.3f,\t%8.3f,\t%8.3f,\t", acc.x, acc.y, acc.z); 
        printf("%8.3f,\t%8.3f,\t%8.3f,\t", gyr.x, gyr.y, gyr.z);         
        printf("%8.3f,\t",             tempMems); 
    
        
        //Calculate accelerometer angles
        double angle_y = alpha * atan( acc.y / sqrt(acc.x*acc.x + acc.z*acc.z) );
        double angle_x = alpha * atan( acc.x / sqrt(acc.y*acc.y + acc.z*acc.z) );
        double angle_z = alpha * atan( sqrt( acc.x*acc.x + acc.y*acc.y) / acc.z  );
        disp.cls();
        disp.locate(0,0);
        disp.printf("X:%4.1f", angle_x);    
        disp.locate(0, 8);
        disp.printf("Y:%4.1f", angle_y);
        disp.locate(1, 0);
        disp.printf("Z:%4.1f", angle_z);        

        // Integrate the Gyro Values
        gyrAngle.x += deltaT * (gyr.x + gyrPrev.x) * 0.5;
        gyrAngle.y += deltaT * (gyr.y + gyrPrev.y) * 0.5;
        gyrAngle.z += deltaT * (gyr.z + gyrPrev.z) * 0.5;
        gyrPrev = gyr;

        printf("%8.3f,\t%8.3f,\t%8.3f,\t%8.6f\n\r", gyrAngle.x, gyrAngle.y, gyrAngle.z, deltaT);

        //Option to reset the angle
        if (BlueButton == 1) {
            buzz.playTone("A");
            wait_us(500000);
            gyrAngle = {0.0, 0.0, 0.0};
            buzz.rest();
        };

        //Loop time is influenced by the following
        wait_us(25000); 
    }
}

