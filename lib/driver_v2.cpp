#include "driver.h"
#include <SPI.h>
#include <math.h>

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

void DWM3000Class::begin() 
{

    delay(5);
    pinMode(CS_PIN, OUTPUT);


    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); // 2 MHz clock speed, max speed of DW3000 is 38MHz, but ESP32
                                                                     // base clock is 80MHz; SPI hardware must be 80/(integer)
                                                                     // MODE0 is sample HIGH on rising edge
                                                                     // MSBFIRST defined by DW3000 User Manual
    SPI.endTransaction();
    digitalWrite(CS_PIN, HIGH);

    if (DEBUG_OUTPUT == 1) {
        Serial.println("[INFO] SPI initialized");
    }
}


void DWM3000Class::spiTransaction(uint8_t txBuffer[] , size_t txLength, size_t rxBuffer[], uint32_t rxLength) {
    // Passing txBuffer[] is a pointer to &txBuffer[0]









    digitalWrite(CS_PIN, LOW); // Pull CS low to start transaction

    // Put txBuffer data on the SPI bus
    // MOSI wire
    for (size_t i = 0; i < txLength; i++) { 
        SPI.transfer(txBuffer[i]);
    }





   





    
    










}
