//#include "mbed.h"
#include "uop_msb.h"


//BEGIN
#include "MPU6050.h"
using namespace MPU6050_DRIVER;

MPU6050 mpu6050;

using namespace uop_msb;

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
    i2c.frequency(400000);  // use fast (400 kHz) I2C 
    wait_us(1000);
    uint8_t whoami = mpu6050.readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    printf("MEMS check: I AM 0x%x\n\r", whoami); printf("I SHOULD BE 0x68\n\r");
    wait_us(1000);


    mpu6050.MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
    printf("x-axis self test: acceleration trim within : "); printf("%f", SelfTest[0]); printf("%% of factory value \n\r");
    printf("y-axis self test: acceleration trim within : "); printf("%f", SelfTest[1]); printf("%% of factory value \n\r");
    printf("z-axis self test: acceleration trim within : "); printf("%f", SelfTest[2]); printf("%% of factory value \n\r");
    printf("x-axis self test: gyration trim within : "); printf("%f", SelfTest[3]); printf("%% of factory value \n\r");
    printf("y-axis self test: gyration trim within : "); printf("%f", SelfTest[4]); printf("%% of factory value \n\r");
    printf("z-axis self test: gyration trim within : "); printf("%f", SelfTest[5]); printf("%% of factory value \n\r");    

    if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) 
    {
        mpu6050.resetMPU6050(); // Reset registers to default in preparation for device calibration
        mpu6050.calibrateMPU6050(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
        mpu6050.initMPU6050();
        printf("PASSED\n");
    } else {
        printf("MPU6050 FAILED SELF TEST\n");
        while (1);
    }

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

        // Wait for data ready bit set, all data registers have new data
        while ( (mpu6050.readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) == 0 );

        mpu6050.readAccelData(accelCount);  // Read the x/y/z adc values
        mpu6050.getAres();
        
        // Now we'll calculate the accleration value into actual g's
        ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
        ay = (float)accelCount[1]*aRes - accelBias[1];   
        az = (float)accelCount[2]*aRes - accelBias[2];  

        mpu6050.readGyroData(gyroCount);  // Read the x/y/z adc values
        mpu6050.getGres();

        // Calculate the gyro value into actual degrees per second
        gx = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
        gy = (float)gyroCount[1]*gRes; // - gyroBias[1];  
        gz = (float)gyroCount[2]*gRes; // - gyroBias[2];   

        tempCount = mpu6050.readTempData();  // Read the x/y/z adc values
        temperature = (tempCount) / 340. + 36.53; // Temperature in degrees Centigrade  
    
        printf("ax = %f\tay = %f\taz = %f\n", ax, ay, az); 
        printf("gx = %f\tgy = %f\tgz = %f deg/s\n", gx,gy,gz);         
        printf("Temp (MPU6050) = %4.1f degC\n\r", temperature); 
    
        wait_us(1000000); 
    }
}

