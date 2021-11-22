#include "uop_msb.h"
using namespace uop_msb;

#include <iostream>
using namespace std;


void UOP_MSB::test() 
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

    // Test Buzzer
    buzz.playTone("C", Buzzer::LOWER_OCTAVE);
    wait_us(100000);
    buzz.playTone("C");
    wait_us(100000);
    buzz.playTone("C", Buzzer::HIGHER_OCTAVE);
    wait_us(100000);
    buzz.rest();

    while (true) {
        //Buttons
        printf("Button A: %d\n", button.Button1.read());
        printf("Button B: %d\n", button.Button2.read());
        printf("Button C: %d\n", button.Button3.read());
        printf("Button D: %d\n", button.Button4.read());

        //Traffic Lights
        traf1RedLED = 1; 
        wait_us(250000);
        traf1YelLED = 1; 
        wait_us(250000);
        traf1GrnLED = 1; 
        wait_us(250000);
        traf2RedLED = 0; 
        wait_us(250000);
        traf2YelLED = 0; 
        wait_us(250000);
        traf2GrnLED = 0; 
        wait_us(250000);
        traf1RedLED = 0;
        wait_us(250000);
        traf1YelLED = 0; 
        wait_us(250000);
        traf1GrnLED = 0; 
        wait_us(250000);
        traf2RedLED = 1; 
        wait_us(250000);
        traf2YelLED = 1; 
        wait_us(250000);
        traf2GrnLED = 1; 
        wait_us(250000);        

        // LDR
        for (unsigned int n=0; n<10; n++) {
            float lightLevel = ldr;
            disp.cls(); disp.locate(0, 0);
            disp.printf("LIGHT: %4.2f", lightLevel);
            wait_us(250000);
        }

        // POT
        for (unsigned int n=0; n<10; n++) {
            float potV = pot;
            disp.cls(); disp.locate(0, 0);
            disp.printf("POT: %4.2f", potV);
            wait_us(250000);
        }

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
