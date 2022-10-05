// (c) 2020 University of Plymouth
// Nicholas Outram
// nicholas.outram@plymouth.ac.uk
//
//
// Added support for SPL06-001 environmental sensor
// updated 14 09 2021
// Added support for MPU6050 MEMS sensor
// updated 17 09 2021
//
// Added class for LEDMatrix and further definitions for the associated pins
// updated 05 10 2022

/* mbed_app.json

You need RTOS and SD card support added to get full functionality

{
    "target_overrides": {
        "*": {
            "target.printf_lib": "std",
            "target.c_lib": "std",   
            "target.components_add": ["SD"],
            "platform.stdio-baud-rate": 115200,
            "platform.stdio-buffered-serial": 1
        }      
    }
}

For baremetal, you cannot use the SD card, so remove the component

{
    "requires": ["bare-metal"],
    "target_overrides": {
        "*": {
            "target.printf_lib": "std",
            "target.c_lib": "std",   
            "platform.stdio-baud-rate": 115200,
            "platform.stdio-buffered-serial": 1
        }      
    }
}

*/


#ifndef __UOP_MSB__
#define __UOP_MSB__

// ******************************************** IMPORTANT ********************************************
// *             Set the value below to match the board version you are using                        *
// ******************************************** IMPORTANT ********************************************
#include "mbed.h"
//#define MSB_VER 2
#define MSB_VER 4


#include "Stream.h"

// SD Support?
#if (MBED_CONF_RTOS_PRESENT == 1)
#if (MBED_CONF_SD_FSFAT_SDCARD_INSTALLED == 1)
#define USE_SD_CARD
#endif
#endif
//            "target.components_add": ["SD"],  

// Use the sensor for the appropriate board version
#if MSB_VER == 2
#include "BMP280_SPI.h"
#include "MPU6050.hpp"  //unpopulated by default - some are retro fitted
#define ENV_SENSOR_T BMP280_SPI
#elif MSB_VER == 4
#include "SPL06-001.h"
#include "MPU6050.hpp"
#define ENV_SENSOR_T SPL06_001_SPI
#else
#error Valid Module Support Board Version is Needed
#endif

//#include <iostream>
#include <chrono>
#include <cstdint>
#include <map>
using namespace std::chrono;

namespace uop_msb {

    // *****************************
    // Traffic Lights (individual) *
    // *****************************
    
    //PUSH PULL
    #define TRAF_GRN1_PIN PC_6
    #define TRAF_YEL1_PIN PC_3
    #define TRAF_RED1_PIN PC_2

    //OPEN DRAIN
    #define TRAF_GRN2_PIN PC_9
    #define TRAF_YEL2_PIN PC_8
    #define TRAF_RED2_PIN PC_7
    #define TRAF_WHITE_PIN PF_10

    // ****************************************
    // LED strip and 7-segment (All on PORTE) *
    // ****************************************
    //All lower 16-bits of PORTE pins are outputs
    #define LED_MASK 0x0000FFFF
    //For initialisation, OE=0, LE=1, DATA=0
    #define LED_INIT 0x00007C00 //0b0111 1100 0000 0000
    //For OFF condition, OE = 1, LE=0 and DATA=0 
    #define LED_OFF 0x00008001

    #define LED_BAR_OE_PIN PE_0
    #define LED_D0_PIN PE_2
    #define LED_D1_PIN PE_3
    #define LED_D2_PIN PE_4
    #define LED_D3_PIN PE_5
    #define LED_D4_PIN PE_6
    #define LED_D5_PIN PE_7
    #define LED_D6_PIN PE_8
    #define LED_D7_PIN PE_9
    #define LED_D1_LE_PIN PE_10
    #define LED_D2_LE_PIN PE_11
    #define LED_RED_LE_PIN PE_12
    #define LED_GRN_LE_PIN PE_13
    #define LED_BLUE_LE_PIN PE_14
    #define LED_DIGIT_OE_PIN PE_15

    // ANALOGUE
    #define AN_POT_PIN  PA_0
    #define MIC_AN_PIN  PA_3
    #define DAC1_AN_PIN PA_4 
    #define DAC2_AN_PIN PA_5
    #define BNC_AN_PIN  PA_6
    #define AN_LDR_PIN  PC_0
    #define STEREO_LEFT_AN_PIN      PB_0
    #define STEREO_RIGHT_AN_PIN     PB_1

    // Environmental Sensor
    #define ENV_MOSI_PIN PB_5
    #define ENV_MISO_PIN PB_4
    #define ENV_SCLK_PIN PB_3
    #define ENV_CS_PIN   PB_2

    // MATRIX
    #define MATRIX_LATCH_PIN PB_6
    #define MATRIX_OE_PIN    PB_12
    #define SPI3_SCK_PIN     PC_10
    #define SPI3_MISO_PIN    PC_11
    #define SPI3_MOSI_PIN    PC_12

    // DIP SWITCHES
    #define DIP_SW_0 PF_12
    #define DIP_SW_1 PF_13
    #define DIP_SW_2 PF_14
    #define DIP_SW_3 PF_15

    // LCD
    #define LCD_D0_PIN PD_0
    #define LCD_D1_PIN PD_1
    #define LCD_D2_PIN PD_2
    #define LCD_D3_PIN PD_3
    #define LCD_D4_PIN PD_4
    #define LCD_D5_PIN PD_5
    #define LCD_D6_PIN PD_6
    #define LCD_D7_PIN PD_7
    #define LCD_RS_PIN PD_11
    #define LCD_RW_PIN PD_12
    #define LCD_E_PIN PD_13
    #define LCD_BKL_PIN PD_14    

    // *********
    // BUTTONS *
    // *********
    #define BTN1_PIN PG_0
    #define BTN2_PIN PG_1
    #define BTN3_PIN PG_2
    #define BTN4_PIN PG_3

    class Buttons {
    public:
        Buttons(PinName b1 = BTN1_PIN, PinName b2 = BTN2_PIN, PinName b3 = BTN3_PIN, PinName b4 = BTN4_PIN, PinName buser = USER_BUTTON) :
        Button1(b1), Button2(b2), Button3(b3, PullDown), Button4(b4, PullDown), BlueButton(buser) {
            
        }
        DigitalIn Button1, Button2, Button3, Button4, BlueButton;
    };

    class DIPSwitches {
    private:
        BusIn DIP;
    public:
        DIPSwitches(PinName d0=DIP_SW_0, PinName d1=DIP_SW_1, PinName d2=DIP_SW_2, PinName d3=DIP_SW_3) : DIP(d0,d1,d2,d3) {
            DIP.mode(PullDown);
        }
        operator int() {
            return DIP;
        }
        int operator [] (uint8_t idx) {
            return DIP[idx & 0b11];
        }
    };

    class LatchedLED {
        public:
            typedef enum {STRIP, SEVEN_SEG} LEDMODE;   
            typedef enum {TENS, UNITS, RED, GREEN, BLUE} LEDGROUP;     
        private:
            LEDMODE _mode;
            LEDGROUP _grp;

            DigitalOut LED_BAR_OE;
            DigitalOut LED_DIGIT_OE;
            DigitalOut LED_D1_LE;
            DigitalOut LED_D2_LE;
            DigitalOut LED_RED_LE;
            DigitalOut LED_GRN_LE;
            DigitalOut LED_BLUE_LE;     
            BusOut dataBus;

            // Returns the currently selected latch enable pin
            DigitalOut& LE(LEDGROUP grp)
            {
                switch (grp) {
                    case TENS:
                    return LED_D1_LE;
                    break;

                    case UNITS:
                    return LED_D2_LE;
                    break;

                    case RED:
                    return LED_RED_LE;
                    break;

                    case GREEN:
                    return LED_GRN_LE;
                    break;

                    case BLUE:
                    return LED_BLUE_LE;
                    break;
                }
            }


            // Convert decimal to a 7-segment pattern
            #define A  0x10
            #define B  0x20
            #define C  0x40
            #define D  0x08
            #define E  0x02
            #define F  0x01
            #define G  0x04
            #define DP 0x80

            uint8_t dec_to_7seg(uint8_t d)
            {
                switch(d){
                    case 0: return(A+B+C+D+E+F);    break;
                    case 1: return(B+C);            break;
                    case 2: return(A+B+D+E+G);      break;
                    case 3: return(A+B+C+D+G);      break;
                    case 4: return(B+C+F+G);        break;
                    case 5: return(A+C+D+F+G);      break;
                    case 6: return(C+D+E+F+G);      break;
                    case 7: return(A+B+C);          break;
                    case 8: return(A+B+C+D+E+F+G);  break;
                    case 9: return(A+B+C+D+F+G);    break;
                    default: return DP;              break;
                }            
            }

        public:
            LatchedLED(LEDMODE mode, LEDGROUP grp=RED) :  _mode(mode), 
                                        _grp(grp),
                                        LED_BAR_OE(LED_BAR_OE_PIN,1),
                                        LED_DIGIT_OE(LED_DIGIT_OE_PIN,1),
                                        LED_D1_LE(LED_D1_LE_PIN,0),
                                        LED_D2_LE(LED_D2_LE_PIN,0),
                                        LED_RED_LE(LED_RED_LE_PIN),
                                        LED_GRN_LE(LED_GRN_LE_PIN),
                                        LED_BLUE_LE(LED_BLUE_LE_PIN),
                                        dataBus(LED_D0_PIN, LED_D1_PIN, LED_D2_PIN, LED_D3_PIN, LED_D4_PIN, LED_D5_PIN, LED_D6_PIN, LED_D7_PIN)                             
            {                                
                //Further initialisation here 
                dataBus = 0;
                LED_RED_LE  = 1;
                LED_GRN_LE  = 1;
                LED_BLUE_LE = 1;
                wait_us(5);   
                LED_RED_LE  = 0;
                LED_GRN_LE  = 0;
                LED_BLUE_LE = 0; 
            }

            ~LatchedLED() {
                LED_BAR_OE = 1;
                LED_DIGIT_OE = 1;
                LED_D1_LE = 0;
                LED_D2_LE = 0;
            }

            void enable(bool en)
            {
                switch (_mode) {
                    case STRIP:
                        LED_BAR_OE = en ? 0 : 1;
                    break;
                    case SEVEN_SEG:
                        LED_DIGIT_OE = en ? 0  : 1;
                        break;
                }
            }

            void setGroup(LEDGROUP grp) {
                if (_grp != grp) {
                    _grp = grp;
                }
            }

            void setMode(LEDMODE mode) {
                if (_mode != mode) {
                    LED_D1_LE = 0;
                    LED_D2_LE = 0;
                    LED_RED_LE = 0;
                    LED_GRN_LE = 0;
                    LED_BLUE_LE = 0;                     
                    _mode = mode;
                }
                
                
            }

            void write(uint8_t dat) {
                switch (_mode) {
                    case STRIP:
                        write(dat, _grp);
                        break;
                    case SEVEN_SEG:
                        uint8_t units = dec_to_7seg(dat % 10);
                        uint8_t tens  = dec_to_7seg(dat / 10);
                        write(tens, TENS);
                        write(units,UNITS);
                        break;
                }
            }
            void write(uint8_t dat, LEDGROUP grp) 
            {
                //Set data pins
                dataBus = dat;
                //Latch enable
                wait_us(1);
                LE(grp)=1;
                wait_us(1);
                LE(grp)=0;
                wait_us(1);
            }

            void operator=(const uint8_t dat) {
                write(dat);
            }

    };

    // BUZZER
    #define BUZZER_PIN PB_13
    class Buzzer {
        public:
        typedef enum {LOWER_OCTAVE, MIDDLE_OCTAVE, HIGHER_OCTAVE} OCTAVE_REGISTER;

        Buzzer(PinName p = BUZZER_PIN) : _buzzer(p) {
            volatile double T = periodForNote_us("C");
            _buzzer.period_us(T);
            rest();      
        }

        void playTone(const char* note, OCTAVE_REGISTER octave=MIDDLE_OCTAVE)
        {
            _buzzer.period_us(periodForNote_us(note, octave));
            _buzzer.write(0.5f);
        }

        void rest() {
            _buzzer.write(0.0);
        }

        protected:
        
        PwmOut _buzzer;
        double periodForNote_us(const char* note, OCTAVE_REGISTER octave=MIDDLE_OCTAVE)
        {
            volatile uint8_t idx = offsetForNote(note); 
            volatile double f = note_freq[idx];

            switch (octave) {
                case LOWER_OCTAVE:
                    f *= 0.5;
                    break;
                case MIDDLE_OCTAVE:
                    break;
                case HIGHER_OCTAVE:
                    f *= 2.0;
                    break;
                default:
                    break;
            }
            return 1000000.0 / f;
        }

        private:
        uint8_t offsetForNote(const char *noteStr)
        {
            uint8_t res;
            switch (noteStr[0])
            { 
                case 'A':
                    res = (noteStr[1]=='#') ? 1 : 0;
                    break;
                case 'B':
                    res = 2;
                    break;
                case 'C':
                    res = (noteStr[1]=='#') ? 4 : 3; 
                    break;
                case 'D':
                    res = (noteStr[1]=='#') ? 6 : 5;                    
                    break;
                case 'E':
                    res = 7;
                    break;
                case 'F':
                    res = (noteStr[1]=='#') ? 9 : 8;                                           
                    break;
                case 'G':
                    res = (noteStr[1]=='#') ? 11 : 10;  
                    break;
                default:
                    res = 0;                   
                    break;
            }

            return res;
        }

        double note_freq[12] = {
            220.0, 233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 329.63, 349.23, 369.99, 392, 415.3
        };

    };

    class LCD_16X2_DISPLAY : public Stream {
    public:
        typedef enum {INSTRUCTION=0, DATA=1} REGISTER_SELECT;   // RS
        typedef enum {WRITE=0, READ=1} READWRITE;               // R/W
        typedef enum {DISABLE=0, ENABLE=1} ENABLE_STATE;        // E

        typedef enum {DECREMENT=0, INCREMENT=1} ENTRY_MODE_DIR;                             // I/D
        typedef enum {CURSOR_MOVE_ON_ENTRY=0, DISPLAY_SHIFT_ON_ENTRY=1} ENTRY_MODE_SHIFT;   // S
        typedef enum {DISP_OFF=0, DISP_ON=1} DISPLAY_STATUS;                        // D
        typedef enum {CURSOR_VISIBLE_OFF=0, CURSOR_VISIBLE_ON=1} CURSOR_VISIBLE;    // C
        typedef enum {BLINK_OFF=0, BLINK_ON=1} BLINK_STATUS;                        // B
        typedef enum {CURSOR_MOVE=0, DISPLAY_SHIFT=1} CURSOR_OR_DISPLAY_SHIFT;      // S/C
        typedef enum {SHIFT_LEFT=0, SHIFT_RIGHT} CURSOR_OR_DISPLAY_SHIFT_DIR;       // R/L
        typedef enum { FOURBIT=0, EIGHTBIT=1 } INTERFACE_DATA_LENGTH;               // DL
        typedef enum { ONELINE=0, TWOLINES=1 } INTERFACE_NUM_LINES;                 // N
        typedef enum { FONT_5X8=0, FONT_5X10=1} INTERFACE_FONT_SIZE;                // F

        enum LCDType {
            LCD16x2     /**< 16x2 LCD panel (default) */
            , LCD16x2B  /**< 16x2 LCD panel alternate addressing */
            , LCD20x2   /**< 20x2 LCD panel */
            , LCD20x4   /**< 20x4 LCD panel */
        };

        // 8-bit constructor
        LCD_16X2_DISPLAY(PinName rs=LCD_RS_PIN, PinName rw=LCD_RW_PIN, PinName e=LCD_E_PIN, PinName bkl=LCD_BKL_PIN, 
        PinName d0=LCD_D0_PIN, PinName d1=LCD_D1_PIN, PinName d2=LCD_D2_PIN, PinName d3=LCD_D3_PIN, 
        PinName d4=LCD_D4_PIN, PinName d5=LCD_D5_PIN, PinName d6=LCD_D6_PIN, PinName d7=LCD_D7_PIN,
        LCDType type=LCD16x2) : _rs(rs), _e(e), _rw(rw), _bkl(bkl), _data(d0, d1, d2, d3, d4, d5, d6, d7), _type(type) 
        {
            //Power on wait
            wait_us(100000);

            //Function set
            functionSet(EIGHTBIT, TWOLINES, FONT_5X8);
            
            //Display ON
            displayOnOffControl(DISP_ON, CURSOR_VISIBLE_OFF, BLINK_OFF);

            //CLS
            cls();

            //Entry Mode
            setEntryMode(INCREMENT, CURSOR_MOVE_ON_ENTRY);

            //Cursor moves right
            //cursorOrDisplayShift(CURSOR_MOVE, SHIFT_RIGHT);        
        }

    protected:  
        DigitalOut _rs;
        DigitalOut _e;
        DigitalOut _rw;
        DigitalOut _bkl;
        BusOut _data;
        LCDType _type;
        uint8_t _row;
        uint8_t _column;

        uint8_t address(int row, int column) {
            switch (_type) {
                case LCD20x4:
                    switch (row) {
                        case 0:
                            return 0x80 + column;
                        case 1:
                            return 0xc0 + column;
                        case 2:
                            return 0x94 + column;
                        case 3:
                            return 0xd4 + column;
                        default:
                            return 0x80 + column;   //Should never happen
                    }
                case LCD16x2B:
                    return 0x80 + (row * 40) + column;
                case LCD16x2:
                case LCD20x2:
                default:
                    return 0x80 + (row * 0x40) + column;
            }
        }


    public:
        void character(int row, int column, uint8_t c) {
            volatile int a = address(row, column);
            set_DDRAM_Address(a);
            write(DATA, c);
        }

        void write(REGISTER_SELECT rs, uint8_t b)
        {
            _rs = rs;
            _rw = WRITE;
            wait_ns(5);
            _e = ENABLE;
            wait_ns(25);    //Rising edge
            wait_ns(100);   //Setup
            _data = b;
            wait_ns(40);    //Data setup time
            _e = DISABLE;
            wait_ns(25);    //Falling edge
            wait_ns(10);    //Hold time

            //Prevent premature write
            wait_us(50);
        }

        uint8_t read(REGISTER_SELECT rs=INSTRUCTION)
        {
            _rs = rs;
            _rw = 1;
            wait_ns(5);
            _e = 1;
            wait_ns(25);    //Rise time
            wait_ns(100);   //T_DDR
            uint8_t data = _data;    //Read
            wait_ns(40);
            _e = 0;
            wait_ns(25);    //Fall time
            wait_ns(10);    //t_H
            _rw = 0;
            wait_us(1);     //Final wait to prevent early trans
            return data;
        }

        void cls() {
            write(INSTRUCTION, 0b00000001);
            wait_us(1600); 
            home();
        }

        void home() {
            write(INSTRUCTION, 0b00000010);
            wait_us(1600); 
            _column = 0;
            _row = 0;       
        }

        void setEntryMode(ENTRY_MODE_DIR id=INCREMENT, ENTRY_MODE_SHIFT sh=CURSOR_MOVE_ON_ENTRY)
        {
            uint8_t dat = 0b00000100;
            dat |= (id ? 2 : 0);
            dat |= (sh ? 1 : 0);
            write(INSTRUCTION, dat);          
        }

        void displayOnOffControl(DISPLAY_STATUS d=DISP_ON, CURSOR_VISIBLE c=CURSOR_VISIBLE_OFF, BLINK_STATUS b=BLINK_OFF)
        {
            uint8_t dat = 0b00001000;
            dat |= (d ? 4 : 0);
            dat |= (c ? 2 : 0);
            dat |= (b ? 1 : 0);
            write(INSTRUCTION, dat);      
        }

        void cursorOrDisplayShift(CURSOR_OR_DISPLAY_SHIFT sc=CURSOR_MOVE, CURSOR_OR_DISPLAY_SHIFT_DIR rl=SHIFT_RIGHT)
        {
            uint8_t dat = 0b00010000;
            dat |= (sc ? 8 : 0);
            dat |= (rl ? 4 : 0);
            write(INSTRUCTION, dat); 
        }

        void functionSet(INTERFACE_DATA_LENGTH dl=EIGHTBIT, INTERFACE_NUM_LINES n=TWOLINES, INTERFACE_FONT_SIZE f=FONT_5X8)
        {
            uint8_t dat = 0b00100000;
            dat |= (dl ? 16 : 0);   //Data lines (0: 4 bit; 1: 8 bit)
            dat |= (n ? 8 : 0);     //Number of display lines (0: 1 line; 1: 2 lines)
            dat |= (f ? 4 : 0);     //Font: (0: 5x8; 1: 5x10)
            write(INSTRUCTION, dat);     
        }    

        void set_CGRAM_Address(uint8_t Addr)
        {
            uint8_t dat =  0b01000000;
            dat |= (Addr & 0b00111111);
            write(INSTRUCTION, dat);
        }

        void set_DDRAM_Address(uint8_t Addr)
        {
            uint8_t dat =  Addr & 0b01111111;
            dat |= 0b10000000;
            write(INSTRUCTION, dat);
        }

        bool isBusy()
        {
            return (read(INSTRUCTION) & 0b10000000) ? true : false;
        }

        void locate(uint8_t row, uint8_t column) {
            _column = column;
            _row = row;
        }

        uint8_t columns() {
            switch (_type) {
                case LCD20x4:
                case LCD20x2:
                    return 20;
                case LCD16x2:
                case LCD16x2B:
                default:
                    return 16;
            }
        }

        uint8_t rows() {
            switch (_type) {
                case LCD20x4:
                    return 4;
                case LCD16x2:
                case LCD16x2B:
                case LCD20x2:
                default:
                    return 2;
            }
        }

        // STDIO
        virtual int _putc(int value) {
            if (value == '\n') {
                _column = 0;
                _row++;
                if (_row >= rows()) {
                    _row = 0;
                }
            } else {
                character(_row, _column, value);
                _column++;
                if (_column >= columns()) {
                    _column = 0;
                    _row++;
                    if (_row >= rows()) {
                        _row = 0;
                    }
                }
            }
            return value;
        }

        virtual int _getc() {
            return -1;
        }
    };

    // Mbed os 5 like Timer
    class TimerCompat : public Timer {

        public:
        long long read_ms() {
            return duration_cast<milliseconds>(elapsed_time()).count();
        }

        long long read_us() {
            return duration_cast<microseconds>(elapsed_time()).count();
        }
    };    

    class EnvSensor {
    private:
        ENV_SENSOR_T sensor;

        float hum, hum0, delta; //Humidity and % persecond
        time_t prevTime, currTime;

        float fRand() {
            return 0.02f*(float)(rand() % 100) - 1.0f;
        } 

    public:
        typedef enum {NONE, BMP280, SPL06_001} ENV_SENSOR_TYPE;

        //PB_5, PB_4, PB_3, PB_2

        EnvSensor(PinName mosi=ENV_MOSI_PIN, PinName miso=ENV_MISO_PIN, PinName sclk=ENV_SCLK_PIN, PinName cs=ENV_CS_PIN) : sensor(mosi, miso, sclk, cs)
        {
            //Initialise the mocked humidity algorithm
            hum = hum0 = 50.0f + 30.0f*fRand(); //20.0% .. 80.0%
            delta = 0.1f*fRand();
            set_time(0);
            prevTime = currTime = time(NULL);
        }
        ~EnvSensor() {
            
        }

        float getTemperature() {
            return sensor.getTemperature();
        }
        float getPressure() {
            return sensor.getPressure();
        }
        // Mocked Humidity Reading
        float getHumidity() 
        {
            prevTime = currTime;
            currTime = time(NULL);
            hum = fmax(fmin(hum+(float)(currTime-prevTime)*delta, 100.0f),0.0f);

            if (fabs(hum-hum0)>=1.0f) {
                //Reset initial values
                set_time(0);
                hum0 = hum;
                prevTime = currTime = time(NULL);
                //New profile
                delta = 0.1f*fRand();  
            }
            return hum;
        }

        ENV_SENSOR_TYPE getSensorType()
        {
            #if MSB_VER == 2
            return BMP280;
            #elif MSB_VER == 4
            return SPL06_001;
            #else
            return NONE;
            #endif
        }

    };    //end class

    typedef struct {
        float x;
        float y;
        float z;
    } Motion_t;

    class MotionSensor : MPU6050_DRIVER::MPU6050 {
        public:
        MotionSensor(PinName SDA=PB_11, PinName SDC=PB_10, int i2cFreq = 400000) : MPU6050_DRIVER::MPU6050(SDA, SDC, i2cFreq)
        {
            if (_whoami != 0x68) {
                DEBUG_PRINT("ERROR: MotionSensor Constructor: Wrong whoami value. Is the board populated?\n");
            }
        }

        // Read x,y and z values from the accelerometer
        Motion_t getAcceleration()
        {
            if (_whoami != 0x68) {
                return {0.0f, 0.0f, 0.0f};
            }

            Motion_t acc;
            // Wait for data ready bit set, all data registers have new data
            while ( (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) == 0 );

            int16_t accelCount[3];
            readAccelData(accelCount);  // Read the x/y/z adc values
            getAres();

            // Now we'll calculate the accleration value into actual g's
            acc.x = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
            acc.y = (float)accelCount[1]*aRes - accelBias[1];   
            acc.z = (float)accelCount[2]*aRes - accelBias[2];  

            return acc;
        }

        // Read x,y and z values from the gyroscope
        Motion_t getGyro()
        {
            if (_whoami != 0x68) {
                return {0.0f, 0.0f, 0.0f};
            }

            Motion_t gyro;
            // Wait for data ready bit set, all data registers have new data
            while ( (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) == 0 );

            int16_t gyroCount[3];  
            readGyroData(gyroCount);  // Read the x/y/z adc values
            getGres();

            // Calculate the gyro value into actual degrees per second
            gyro.x = (float)gyroCount[0]*gRes; // - gyroBias[0];  // get actual gyro value, this depends on scale being set
            gyro.y = (float)gyroCount[1]*gRes; // - gyroBias[1];  
            gyro.z = (float)gyroCount[2]*gRes; // - gyroBias[2];  

            return gyro;
        }

        // Read temperature using the MPU6050
        float getTemperatureC()
        {
            if (_whoami != 0x68) {
                return 0.0f;
            }
            
            // Wait for data ready bit set, all data registers have new data
            while ( (readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) == 0 );

            float tempC = (float)readTempData();    // 
            temperature = tempC / 340. + 36.53;     // Temperature in degrees Centigrade             
            return temperature;
        }

        // Get the device ID
        uint8_t whoAmI() {
            return _whoami;
        }

    };

    // ********************************************** BOARD TEST CLASS **********************************************
    class UOP_MSB_TEST {
    private:
        // Motion Sensor
        MotionSensor motion;

        //On board LEDs
        DigitalOut led1;
        DigitalOut led2;
        DigitalOut led3;
        
        //On board switch
        DigitalIn BlueButton;

        //LCD Display
        LCD_16X2_DISPLAY disp;

        //Buzzer
        Buzzer buzz;

        //Traffic Lights
        DigitalOut traf1RedLED;
        DigitalOut traf1YelLED;
        DigitalOut traf1GrnLED;
        DigitalInOut traf2RedLED;
        DigitalInOut traf2YelLED;
        DigitalInOut traf2GrnLED;

        //Light Levels
        AnalogIn ldr;

        //Pot
        AnalogIn pot;

        //LCD Backlight
        DigitalOut backLight;

        //DIP Switches
        DIPSwitches dipSwitches;

        //Push Buttons
        Buttons button;

        //Environmental Sensor
        EnvSensor env;   

        // 3x8 LED Strips + 7-seg display
        LatchedLED sevenSeg;

        // LED Matrix 
        SPI matrix_spi;   // MOSI, MISO, SCLK
        DigitalOut matrix_spi_cs;            //Chip Select ACTIVE LOW
        DigitalOut matrix_spi_oe;           //Output Enable ACTIVE LOW        

        public:
        UOP_MSB_TEST() : led1(LED1), led2(LED2), led3(LED3), BlueButton(USER_BUTTON), 
                    traf1RedLED(TRAF_RED1_PIN,1), traf1YelLED(TRAF_YEL1_PIN), traf1GrnLED(TRAF_GRN1_PIN),
                    traf2RedLED(TRAF_RED2_PIN, PIN_OUTPUT, OpenDrainNoPull, 0), traf2YelLED(TRAF_YEL2_PIN, PIN_OUTPUT, OpenDrainNoPull, 1), traf2GrnLED(TRAF_GRN2_PIN, PIN_OUTPUT, OpenDrainNoPull, 1),
                    ldr(AN_LDR_PIN), pot(AN_POT_PIN), backLight(LCD_BKL_PIN), sevenSeg(LatchedLED::LEDMODE::SEVEN_SEG),
                    matrix_spi(SPI3_MOSI_PIN, SPI3_MISO_PIN, SPI3_SCK_PIN), matrix_spi_cs(MATRIX_LATCH_PIN), matrix_spi_oe(MATRIX_OE_PIN)
        {
            //Constructor
        }

        void test();

        private:
        void clearMatrix(void);
        void matrix_scan(void);

    };    

}

#endif