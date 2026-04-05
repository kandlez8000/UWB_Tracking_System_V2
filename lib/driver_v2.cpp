#include "driver.h"
#include <SPI.h>
#include <math.h>

/*
 * DWM3000 SPI Driver Architecture
 * 1. CS pin toggling is strictly handled inside the lowest-level SPI transfer function for atomicity.
 * 2. Data buffers are strictly uint8_t arrays.
 * 3. Dynamic header sizing is handled internally by readWriteRegister; user code only passes reg/sub_reg.
 */

DWM3000Class DWM3000;

// Internal variables for the driver
static int ANTENNA_DELAY = 16350;
static int destination = 0x0;
static int sender = 0x0;

DWM3000_Config_t DWM3000Class::config = {
    CHANNEL_5,         // channel
    PREAMBLE_1024,     // preamble_length
    9,                 // preamble_code
    PAC32,             // pac_size
    DATARATE_6_8MB,    // datarate
    PHR_MODE_STANDARD, // phr_mode
    PHR_RATE_850KB     // phr_rate
};

void DWM3000Class::begin() //Made by @kendagaz
{

    delay(5);
    pinMode(CS_PIN, OUTPUT);

    SPI.begin();
    digitalWrite(CS_PIN, HIGH);

    if (DEBUG_OUTPUT == 1) {
        Serial.println("[INFO] SPI initialized");
    }
}


void DWM3000Class::spiTransaction(uint8_t txBuffer[] , size_t txLength, size_t rxBuffer[], uint32_t rxLength) { //Made by @kendagaz
    // Passing txBuffer[] is a pointer to &txBuffer[0]


    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

    digitalWrite(CS_PIN, LOW); // Pull CS low to start transaction
    
    // Put txBuffer data on the SPI bus
    // MOSI wire
    for (size_t i = 0; i < txLength; i++) { 
        SPI.transfer(txBuffer[i]);
    }

    SPI.endTransaction(); 
    digitalWrite(CS_PIN, HIGH); // Pull CS high to end transaction
}

void DWM3000Class::readWriteRegister() { //Made by @kendagaz
    // Passing txBuffer[] is a pointer to &txBuffer[0]


    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));

    digitalWrite(CS_PIN, LOW); // Pull CS low to start transaction


    SPI.endTransaction(); 
    digitalWrite(CS_PIN, HIGH); // Pull CS high to end transaction
}
