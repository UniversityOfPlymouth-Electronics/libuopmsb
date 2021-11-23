#include "uop_msb.h"
using namespace uop_msb;

#include <iostream>
using namespace std;

// MATRIX DISPLAY SPI WITH OUTPUT ENABLE
#define SCAN_RATE_MS   50

// Clear the LED matrix (code by Martin Simpson)
void UOP_MSB_TEST::clearMatrix(void)
{
    matrix_spi_cs=0;           //CLEAR Matrix
    matrix_spi.write(0x00);//COL RHS
    matrix_spi.write(0x00);//COL LHS
    matrix_spi.write(0x00);//ROX RHS
    matrix_spi_cs=1;
}

// Test the LED matrix (code by Martin Simpson)
void UOP_MSB_TEST::matrix_scan(void)
{
    for (int j=0;j<=7;j++)  //NB ROW has 3 to 8 decoder so 0 to 7 to Row Select
                            //COLUMNS(COL) are 0 to 255 to select each Byte
    {
        for (int i=1;i<255;i*=2) //Increment LEFT Hand Block of 8
        {
            matrix_spi_cs=0;           //Send Data to Matrix
            matrix_spi.write(0x00);//COL RHS
            matrix_spi.write(i);   //COL LHS
            matrix_spi.write(j);   //ROW RHS
            matrix_spi_cs=1;           //low to high will effectivelly LATCH the Shift register to output
            thread_sleep_for(SCAN_RATE_MS);
        }
        for (int i=1;i<255;i*=2) //Increment RIGHT Hand Block of 8
        {
            matrix_spi_cs=0;           //Send Data to Matrix
            matrix_spi.write(i);   //COL RHS
            matrix_spi.write(0x00);//COL LHS
            matrix_spi.write(j);   //ROW RHS
            matrix_spi_cs=1;           //low to high will effectivelly LATCH the Shift register to output
            thread_sleep_for(SCAN_RATE_MS);
        }
    }
for (int j=0;j<=7;j++)  //NB ROW has 3 to 8 decoder so 0 to 7 to Row Select
                            //COLUMNS(COL) are 0 to 255 to select each Byte
    {
        {
            matrix_spi_cs=0;           //Send Data to Matrix
            matrix_spi.write(0xFF);//COL RHS
            matrix_spi.write(0xFF);   //COL LHS
            matrix_spi.write(j);   //ROW RHS
            matrix_spi_cs=1;           //low to high will effectivelly LATCH the Shift register to output
            thread_sleep_for(SCAN_RATE_MS);
        }
    }
    for (int j=7;j>=0;j--)  //NB ROW has 3 to 8 decoder so 0 to 7 to Row Select
                            //COLUMNS(COL) are 0 to 255 to select each Byte
    {
        {
            matrix_spi_cs=0;           //Send Data to Matrix
            matrix_spi.write(0xFF);//COL RHS
            matrix_spi.write(0xFF);   //COL LHS
            matrix_spi.write(j);   //ROW RHS
            matrix_spi_cs=1;           //low to high will effectivelly LATCH the Shift register to output
            thread_sleep_for(SCAN_RATE_MS);
        }
    }
}


// SD Card (RTOS ONLY)
#ifdef USE_SD_CARD
#include "SDBlockDevice.h"
#include "FATFileSystem.h"
// Instantiate the SDBlockDevice by specifying the SPI pins connected to the SDCard
// socket. The PINS are: (This can also be done in the JSON file see mbed.org Docs API Storage SDBlockDevice)
// PB_5    MOSI (Master Out Slave In)
// PB_4    MISO (Master In Slave Out)
// PB_3    SCLK (Serial Clock)
// PF_3    CS (Chip Select)
//
// and there is a Card Detect CD on PF_4 ! (NB this is an Interrupt capable pin..)
SDBlockDevice sd(PB_5, PB_4, PB_3, PF_3);

int write_sdcard()
{
    printf("Initialise and write to a file\n");
 int err;
    // call the SDBlockDevice instance initialisation method.

    err=sd.init();
    if ( 0 != err) {
        printf("Init failed %d\n",err);
        return -1;
    }
    
    FATFileSystem fs("sd", &sd);
    FILE *fp = fopen("/sd/test.txt","w");
    if(fp == NULL) {
        error("Could not open file for write\n");
        sd.deinit();
        return -1;
    } else {
        //Put some text in the file...
        fprintf(fp, "Martin Says Hi!\n");
        //Tidy up here
        fclose(fp);
        printf("SD Write done...\n");
        sd.deinit();
        return 0;
    }
    
}

int read_sdcard()
{
    printf("Initialise and read from a file\n");

    // call the SDBlockDevice instance initialisation method.
    if ( 0 != sd.init()) {
        printf("Init failed \n");
        return -1;
    }
    
    FATFileSystem fs("sd", &sd);
    FILE *fp = fopen("/sd/test.txt","r");
    if(fp == NULL) {
        error("Could not open or find file for read\n");
        sd.deinit();
        return -1;
    } else {
        //Put some text in the file...
        char buff[64]; buff[63] = 0;
        while (!feof(fp)) {
            fgets(buff, 63, fp);
            printf("%s\n", buff);
        }
        //Tidy up here
        fclose(fp);
        printf("SD Write done...\n");
        sd.deinit();
        return 0;
    }
}
#endif


void UOP_MSB_TEST::test() 
{
    //LCD Backlight ON
    backLight = 1;

    //Test SD Card
    #ifdef USE_SD_CARD
    write_sdcard();
    read_sdcard();
    #endif

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

    // LED Matrix
    clearMatrix();
    matrix_scan();
    clearMatrix();

    // LED Strips (3x8)
    sevenSeg.setMode(LatchedLED::LEDMODE::STRIP);
    sevenSeg.enable(true);
    auto kr = [&]() {
        for (unsigned n=1; n<=128; n<<=1) {
            sevenSeg = n;
            wait_us(100000);
        }
    };
    sevenSeg.setGroup(LatchedLED::LEDGROUP::RED);
    kr();
    sevenSeg.setGroup(LatchedLED::LEDGROUP::GREEN);
    kr();
    sevenSeg.setGroup(LatchedLED::LEDGROUP::BLUE);
    kr();

    wait_us(250000);
    sevenSeg.enable(false);

    // 7-segment
    sevenSeg.setMode(LatchedLED::LEDMODE::SEVEN_SEG);
    sevenSeg.enable(true);
    for (unsigned n=0; n<=99; n++) {
        sevenSeg = n;
        wait_us(100000);
    }
    wait_us(250000);
    sevenSeg.enable(false);

    //Main loop
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
