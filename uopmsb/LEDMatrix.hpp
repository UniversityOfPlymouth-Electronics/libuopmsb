#pragma once
#include "uop_msb.h"
using namespace uop_msb;

// Base Class for the 16x8 LED Matrix
class LEDMatrix {
protected:
    SPI matrix_spi;             // Digital Interface for the LED Matrix
    DigitalOut matrix_spi_cs;   // Chip Select
    DigitalOut matrix_spi_oe;   // Output Enable

private:
    // Plot a point at row (0..7),col (0..15)
    // Each addressable row is a 16-bit pattern held in a latch
    void rcplot(const uint8_t row, const uint8_t col)
    {
        //Set a 1 in the column position
        uint16_t u = 1 << col;          //Col is a 16-bit pattern

        // Start the SPI transaction by pulling Chip Select LOW
        matrix_spi_cs=0;                //Send Data to Matrix

        // Write the 16-bit bit pattern for the row
        matrix_spi.write(u >> 8);       //COL RHS
        matrix_spi.write(u & 0xFF);     //COL LHS

        // Select the row
        matrix_spi.write(row & 0x07);   //ROW RHS

        // End the SPI transaction by pulling Chip Select HIGH
        matrix_spi_cs=1;                //low to high will effectively LATCH the Shift register to output    
    } 

public:
    // Parameterless Constructor 
    LEDMatrix() : matrix_spi(SPI3_MOSI_PIN, SPI3_MISO_PIN, SPI3_SCK_PIN), matrix_spi_cs(MATRIX_LATCH_PIN), matrix_spi_oe(MATRIX_OE_PIN)
    {
        //Nothing to do yet
    }

    void xyplot(const uint8_t x, const uint8_t y) {
        this->rcplot(y, x);
    }

    void test()
    {
        for (uint8_t row = 0; row<8; row++) {
            for (uint8_t col = 0; col<16; col++) {
                this->rcplot(row, col);
                wait_us(25000);
            }
        }    
    }
};

class BalanceTable : public LEDMatrix
{
private:

    volatile int16_t deltaX=0;
    volatile int8_t  deltaY=0;
    volatile uint8_t phase = 0;
    Ticker xyTicker;  
    #if (MBED_CONF_RTOS_PRESENT == 1)
    Thread t1;
    EventQueue queue;
    #endif
    //Draw dot on table
    void centreDot() 
    {
        uint16_t x = (deltaX >= 0) ? (0b0000000110000000 << deltaX) : (0b0000000110000000 >> abs(deltaX));
        uint8_t  y = 3+phase+deltaY;

        matrix_spi_cs=0;                //Send Data to Matrix
        matrix_spi.write(x >> 8);       //COL RHS
        matrix_spi.write(x & 0x00FF);   //COL LHS
        matrix_spi.write(y & 0x07);     //ROW RHS
        matrix_spi_cs=1;                //low to high will effectively LATCH the Shift register to output    

        //Flip phase
        phase = 1-phase;   
    }    
  

public:
    BalanceTable() : LEDMatrix()
    {
        #if (MBED_CONF_RTOS_PRESENT == 1)
        //For RTOS, use a thread
        auto go = [&](){
            queue.dispatch_forever();
        };
        t1.start(go);
        queue.call_every(10ms, callback(this, &BalanceTable::centreDot));
        #else
        //Simple version - no RTOS
        xyTicker.attach(callback(this, &BalanceTable::centreDot), 10ms);
        #endif
    }

    void setXYoffset(int8_t dX, int8_t dY)
    {
        CriticalSectionLock lock;
        deltaX = dX;
        deltaY = dY;
    }

    void test()
    {
        for (int8_t dx = -7; dx<=7; dx++) {
            for (int8_t dy=-3; dy<=3; dy++) {
                this->setXYoffset(dx, dy);
                wait_us(50000);   
            }
        }
    }

};