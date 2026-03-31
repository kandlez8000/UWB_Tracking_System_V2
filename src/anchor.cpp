#include <SPI.h>
#include "driver.h"
#include <Arduino.h>

// Set to 1 for Anchor 1, 2 for Anchor 2
#define ANCHOR_ID 1
#define RESPONSE_TIMEOUT_MS 20 
#define MAX_RETRIES 3

unsigned long last_ranging_time = 0;
int retry_count = 0;

static int rx_status;
static int tx_status;
static int curr_stage = 0;
static int t_roundB = 0;
static int t_replyB = 0;
static long long rx = 0;
static long long tx = 0;

void resetRadio()
{
    DWM3000.softReset();
    delay(200);

    // Re-initialize the complex register settings
    DWM3000.init();

    // Re-apply the specific Anchor configurations
    DWM3000.setTXAntennaDelay(16350);
    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();
    DWM3000.standardRX();
}

void setup()
{
   Serial.begin(1152000);
   DWM3000.begin();
   DWM3000.hardReset();
   delay(200);

   if (!DWM3000.checkSPI())
   {
       Serial.println("[ERROR] Could not establish SPI Connection to DWM3000!");
       while (1);
   }

   while (!DWM3000.checkForIDLE())
   {
       Serial.println("[ERROR] IDLE1 FAILED\r");
       delay(1000);
   }

   DWM3000.softReset();
   delay(200);

   if (!DWM3000.checkForIDLE())
   {
       Serial.println("[ERROR] IDLE2 FAILED\r");
       while (1);
   }
   DWM3000.init();
   DWM3000.setupGPIO();
   DWM3000.setTXAntennaDelay(16350);
   DWM3000.setSenderID(ANCHOR_ID);

   DWM3000.configureAsTX();
   DWM3000.clearSystemStatus();
   DWM3000.standardRX();
}

void loop()
{
   if (DWM3000.receivedFrameSucc() == 1 && DWM3000.ds_getStage() == 1 && DWM3000.getDestinationID() == ANCHOR_ID)
   {
       if (curr_stage != 0)
       {
           curr_stage = 0;
           t_roundB = 0;
           t_replyB = 0;
       }
   }
   
   switch (curr_stage)
   {
   case 0: // Await ranging
       t_roundB = 0;
       t_replyB = 0;

       if (rx_status = DWM3000.receivedFrameSucc())
       {
           DWM3000.clearSystemStatus();
           if (rx_status == 1)
           { 
               if (DWM3000.getDestinationID() == ANCHOR_ID)
               {
                   if (DWM3000.ds_isErrorFrame())
                   {
                       Serial.println("[WARNING] Received error frame!");
                       curr_stage = 0;
                       DWM3000.standardRX();
                   }
                   else if (DWM3000.ds_getStage() != 1)
                   {
                       Serial.print("[WARNING] Unexpected stage: ");
                       Serial.println(DWM3000.ds_getStage());
                       DWM3000.ds_sendErrorFrame();
                       DWM3000.standardRX();
                       curr_stage = 0;
                   }
                   else
                   {
                       curr_stage = 1;
                   }
               }
               else
               {
                   DWM3000.standardRX();
               }
           }
           else
           {
               Serial.println("[ERROR] Receiver Error occurred!");
               DWM3000.clearSystemStatus();
           }
       }
       else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS)
       {
           Serial.println("[WARNING] Timeout waiting for ranging request");
           if (++retry_count > MAX_RETRIES)
           {
               Serial.println("[ERROR] Max retries reached, resetting radio");
               resetRadio();
               retry_count = 0;
           }
           DWM3000.standardRX(); 
       }
       break;

   case 1: // Ranging received. Sending response
       DWM3000.ds_sendFrame(2);

       rx = DWM3000.readRXTimestamp();
       tx = DWM3000.readTXTimestamp();

       t_replyB = tx - rx;
       curr_stage = 2;
       last_ranging_time = millis(); 
       break;

   case 2: // Awaiting response
       if (rx_status = DWM3000.receivedFrameSucc())
       {
           retry_count = 0; 
           DWM3000.clearSystemStatus();
           if (rx_status == 1)
           { 
               if (DWM3000.ds_isErrorFrame())
               {
                   Serial.println("[WARNING] Received error frame!");
                   curr_stage = 0;
                   DWM3000.standardRX();
               }
               else if (DWM3000.ds_getStage() != 3)
               {
                   Serial.print("[WARNING] Unexpected stage: ");
                   Serial.println(DWM3000.ds_getStage());
                   DWM3000.ds_sendErrorFrame();
                   DWM3000.standardRX();
                   curr_stage = 0;
               }
               else
               {
                   curr_stage = 3;
               }
           }
           else
           {
               Serial.println("[ERROR] Receiver Error occurred!");
               DWM3000.clearSystemStatus();
           }
       }
       else if (millis() - last_ranging_time > RESPONSE_TIMEOUT_MS)
       {
           Serial.println("[WARNING] Timeout waiting for second response");
           if (++retry_count > MAX_RETRIES)
           {
               Serial.println("[ERROR] Max retries reached, resetting radio");
               resetRadio();
               retry_count = 0;
           }
           curr_stage = 0;
           DWM3000.standardRX();
       }
       break;

   case 3: // Second response received. Sending information frame
       rx = DWM3000.readRXTimestamp();
       t_roundB = rx - tx;
       DWM3000.ds_sendRTInfo(t_roundB, t_replyB);

       curr_stage = 0;
       DWM3000.standardRX();
       break;

   default:
       Serial.print("[ERROR] Entered unknown stage (");
       Serial.print(curr_stage);
       Serial.println("). Reverting back to stage 0");

       curr_stage = 0;
       DWM3000.standardRX();
       break;
   }
}