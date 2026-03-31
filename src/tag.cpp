#include <SPI.h>
#include "driver.h"
#include <Arduino.h>

// Scalable Anchor Configuration
#define NUM_ANCHORS 1
#define TAG_ID 10
#define FIRST_ANCHOR_ID 1 

// Ranging Configuration
#define FILTER_SIZE 50 
#define MIN_DISTANCE 0
#define MAX_DISTANCE 10000.0

// Watchdog Timer for Circumventing Dropped Packets
#define TAG_TIMEOUT_MS 20
unsigned long last_ranging_time = 0;

// Global variables
static int rx_status;
static int tx_status;
static int current_anchor_index = 0; 
static int curr_stage = 0;

// Anchor data structure
struct AnchorData
{
   int anchor_id; 

   // Timing measurements
   int t_roundA = 0;
   int t_replyA = 0;
   long long rx = 0;
   long long tx = 0;
   int clock_offset = 0;

   // Distance measurements
   float distance = 0;
   float distance_history[FILTER_SIZE] = {0};
   int history_index = 0;
   float filtered_distance = 0;

   // Signal quality metrics
   float signal_strength = 0;    
   float fp_signal_strength = 0; 
};

// Dynamic array of anchor data
AnchorData anchors[NUM_ANCHORS];

void initializeAnchors()
{
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
   }
}

AnchorData *getCurrentAnchor()
{
   return &anchors[current_anchor_index];
}

int getCurrentAnchorId()
{
   return anchors[current_anchor_index].anchor_id;
}

void switchToNextAnchor()
{
   current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS;
}

bool allAnchorsHaveValidData()
{
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       if (anchors[i].filtered_distance <= 0)
       {
           return false;
       }
   }
   return true;
}

bool isValidDistance(float distance)
{
   return (distance >= MIN_DISTANCE && distance <= MAX_DISTANCE);
}

float calculateMedian(float arr[], int size)
{
   float temp[size];
   for (int i = 0; i < size; i++) temp[i] = arr[i];

   for (int i = 0; i < size - 1; i++)
   {
       for (int j = i + 1; j < size; j++)
       {
           if (temp[j] < temp[i])
           {
               float t = temp[i];
               temp[i] = temp[j];
               temp[j] = t;
           }
       }
   }

   if (size % 2 == 0)
       return (temp[size / 2 - 1] + temp[size / 2]) / 2.0;
   else
       return temp[size / 2];
}

void updateFilteredDistance(AnchorData &data)
{
   data.distance_history[data.history_index] = data.distance;
   data.history_index = (data.history_index + 1) % FILTER_SIZE;

   float valid_distances[FILTER_SIZE];
   int valid_count = 0;

   for (int i = 0; i < FILTER_SIZE; i++)
   {
       if (isValidDistance(data.distance_history[i]))
       {
           valid_distances[valid_count++] = data.distance_history[i];
       }
   }

   if (valid_count > 0)
   {
       data.filtered_distance = calculateMedian(valid_distances, valid_count);
   }
   else
   {
       data.filtered_distance = 0;
   }
}

void printAllDistances()
{
   Serial.print("Distances - ");
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       Serial.print("A");
       Serial.print(anchors[i].anchor_id);
       Serial.print(": ");
       if (anchors[i].filtered_distance > 0)
       {
           DWM3000.printDouble(anchors[i].filtered_distance, 100, false);
           Serial.print(" cm");
       }
       else
       {
           Serial.print("INVALID");
       }

       if (i < NUM_ANCHORS - 1)
       {
           Serial.print(" | ");
       }
   }
   Serial.println();
}

void setup()
{
   Serial.begin(115200);
   initializeAnchors();

   Serial.print("Initialized ");
   Serial.print(NUM_ANCHORS);
   Serial.println(" anchors:");
   for (int i = 0; i < NUM_ANCHORS; i++)
   {
       Serial.print("  Anchor ");
       Serial.print(i);
       Serial.print(" - ID: ");
       Serial.println(anchors[i].anchor_id);
   }

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
   DWM3000.setSenderID(TAG_ID);

   Serial.println("> TAG - Three Anchor Ranging System <");
   Serial.println("> Wi-Fi Removed - Awaiting ESP-NOW Integration <\n");
   Serial.println("[INFO] Setup is finished.");
   Serial.print("Antenna delay set to: ");
   Serial.println(DWM3000.getTXAntennaDelay());

   DWM3000.configureAsTX();
   DWM3000.clearSystemStatus();
}

void loop()
{
   AnchorData *currentAnchor = getCurrentAnchor();
   int currentAnchorId = getCurrentAnchorId();

   switch (curr_stage)
   {
   case 0: // Start ranging with current target
       currentAnchor->t_roundA = 0;
       currentAnchor->t_replyA = 0;

       DWM3000.setDestinationID(currentAnchorId);
       DWM3000.ds_sendFrame(1);
       currentAnchor->tx = DWM3000.readTXTimestamp();
       curr_stage = 1;
       last_ranging_time = millis();
       break;

   case 1: // Await first response
       if (rx_status = DWM3000.receivedFrameSucc())
       {
           DWM3000.clearSystemStatus();
           if (rx_status == 1)
           {
               if (DWM3000.ds_isErrorFrame())
               {
                   Serial.print("[WARNING] Error frame from Anchor ");
                   curr_stage = 0;
               }
               else if (DWM3000.ds_getStage() != 2)
               {
                   Serial.print("[WARNING] Unexpected stage from Anchor ");
                   DWM3000.ds_sendErrorFrame();
                   curr_stage = 0;
               }
               else
               {
                   curr_stage = 2;
               }
           }
           else
           {
               Serial.print("[ERROR] Receiver Error from Anchor ");
               DWM3000.clearSystemStatus();
               curr_stage = 0;
           }
       }
       else if (millis() - last_ranging_time > TAG_TIMEOUT_MS) 
       {
            Serial.print("[WARNING] Timeout waiting for Anchor ");
            Serial.println(currentAnchorId);
            DWM3000.standardRX();
            switchToNextAnchor();
            curr_stage = 0;
       }
       break;

   case 2: // Response received. Send second ranging
       currentAnchor->rx = DWM3000.readRXTimestamp();
       DWM3000.ds_sendFrame(3);

       currentAnchor->t_roundA = currentAnchor->rx - currentAnchor->tx;
       currentAnchor->tx = DWM3000.readTXTimestamp();
       currentAnchor->t_replyA = currentAnchor->tx - currentAnchor->rx;

       curr_stage = 3;
       last_ranging_time = millis();
       break;

   case 3: // Await second response
       if (rx_status = DWM3000.receivedFrameSucc())
       {
           DWM3000.clearSystemStatus();
           if (rx_status == 1)
           {
               if (DWM3000.ds_isErrorFrame())
               {
                   Serial.print("[WARNING] Error frame from Anchor ");
                   curr_stage = 0;
               }
               else
               {
                   currentAnchor->clock_offset = DWM3000.getRawClockOffset();
                   curr_stage = 4;
               }
           }
           else
           {
               Serial.print("[ERROR] Receiver Error from Anchor ");
               DWM3000.clearSystemStatus();
               curr_stage = 0;
           }
       }
       else if (millis() - last_ranging_time > TAG_TIMEOUT_MS) 
       {
            Serial.print("[WARNING] Timeout waiting for Anchor ");
            Serial.println(currentAnchorId);
            DWM3000.standardRX();
            switchToNextAnchor();
            curr_stage = 0;
       }
       break;

   case 4: // Response received. Calculating results
   {
       int ranging_time = DWM3000.ds_processRTInfo(
           currentAnchor->t_roundA,
           currentAnchor->t_replyA,
           DWM3000.read(0x12, 0x04),
           DWM3000.read(0x12, 0x08),
           currentAnchor->clock_offset);

       currentAnchor->distance = DWM3000.convertToCM(ranging_time);
       currentAnchor->signal_strength = DWM3000.getSignalStrength();
       currentAnchor->fp_signal_strength = DWM3000.getFirstPathSignalStrength();
       updateFilteredDistance(*currentAnchor);
   }

       printAllDistances();

       if (allAnchorsHaveValidData())
       {
           // TODO: Implement ESP-NOW broadcast here
           // sendDataViaESPNow();
       }

       switchToNextAnchor();
       curr_stage = 0;
       break;

   default:
       Serial.print("Entered stage (");
       Serial.print(curr_stage);
       Serial.println("). Reverting back to stage 0");
       curr_stage = 0;
       break;
   }
}