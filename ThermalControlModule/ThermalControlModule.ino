/*
  TesLorean Thermal Control Module
  2018
  Jeff Cooke

  Based on Gen2 Charger Control Program by T de Bree, D.Maguire 2017-2018
  Additional work by C. Kidder
*/
#include <mcp2515.h>
#include <SPI.h>
#include "config.h"

// Create two CAN objects, # is the chip select
MCP2515 can0(10);        // TesLorean CAN
MCP2515 can1(9);         // Battery CAN bus

// Create some reusable can frames


//****** DEBUG VARIABLES
int debug = 1;

//********* STATUS DEFINITIONS ***********
// Relay Statuses
#define RELAY_ON 0
#define RELAY_OFF 1

// Relay Connections
#define Relay_X358_7  3  // X358-7 K3: Main -VE [G]
#define Relay_X358_8  4  // X358-8 Multifunction contactor [E]
#define Relay_X358_9  5  // X358-9 K1: Precharge contactor for Main [L]
#define Relay_X358_10  6  // X358-10 K2: Main +VE contactor [F]
#define Relay_X357_4  7  // X357-3 to 12v accessory wake up
#define Relay_X357_5  8  // X357-14 to 12v HV energy mgt communication enable

//********* DELAY SETTINGS ******************
#define DELAY_NAME 1000    // Time (in ms)

//********* TIME SINCE VARIABLES *************
unsigned long timeStatus = 0;

//********* STATE VARIABLES ******************
int NAMEstate;      // Current state of the contactors

//********* CAN IDs ******************
int NAMEFrameID = 0x140;

//********* HELPER ARRAYS ************
byte group4mods[8][3] = {{0, 0, 1}, {1, 2, 2}, {3, 3, 4}, {4, 5, 5}, {6, 6, 7}, {7, 9, 9}, {9, 9, 9}, {9, 9, 9}}; // Modules for Group 4 voltages
byte startcellnum[5] = {0, 3, 6, 9, 12}; // cell voltage frames, starting triple voltage cell num

//********* DATA FROM BMS ******************
uint16_t moduletemps[8] = {0, 0, 0, 0, 0, 0, 0, 0};       // 1 = 0.5F
uint16_t totalpackvolts = 0;                              // 1 = 1V
uint16_t priorpackvolts = 0;                             // Total voltage at prior reporting
uint16_t packtemps[2] = {0, 0};                           // 1 = 1F
uint16_t cellvolts[14][8];                                // 14 cells in each of 8 modules, 1 = 1/100th V
uint16_t mincellvolts = 0;                                // lowest individual cell voltage, 1 = 1/100th V
uint16_t maxcellvolts = 0;                                // highest individual cell voltage, 1 = 1/100th V
byte frame260[5][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame262[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame270[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame272[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
byte frame274[2][3] = {{0, 0, 0}, {0, 0, 0}};

void setup()
{
  //// INITIALIZE RELAY PINS
  digitalWrite(Relay_X358_7, RELAY_OFF); // X358-7 K3: Main -VE [G]

  //// SETUP OUTPUT LINES
  pinMode(Relay_X358_7, OUTPUT);

  // init the serial port - for status/debug
  while (!Serial);
  Serial.begin(115200);

  // init the SPI communications
  SPI.begin();
  
  //// TIMERS AND INTERRUPTS
  //  Timer3.attachInterrupt(Controller_CAN_Messages).start(90000); // charger messages every 100ms

  //// CAN PORTS
  can0.reset();
  can0.setBitrate(CAN_500KBPS);
  can0.setNormalMode();
  can1.reset();
  can1.setBitrate(CAN_500KBPS);
  can1.setNormalMode();
  Serial.println("CAN Initialized");
}

void loop()
{
  // Define CAN Frame variable structure
  CAN_FRAME incoming;

  // Check for instructions on the TesLorean CAN bus
  if (Can0.available())
  {
    Can0.read(incoming);
    can0decode(incoming);
  }

  // Selectively send out status and informational messages
  Controller_CAN_Messages;

  // Check the Serial Bus for commands
  if (Serial.available())
  {
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 'd'://d for display
        Serial.println();
        Serial.print("Status : ");
        Serial.println(incomingByte);
        break;

      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

  // Check for Request to change to a different Battery Contactors Status
  if (contactorsreqdstate != contactorsstate)
  {

    // Request to CLOSE (activate) the battery contactors
    if (contactorsreqdstate == CONTACTORSCLOSED)
    {
      switch (contactorsstate)
      {
        case CONTACTORSOPEN:
          // Switch on the -VE and Multifunction Contactors
          digitalWrite(Relay_X358_7, RELAY_ON);  // Enable Main -VE Contactor
          digitalWrite(Relay_X358_8, RELAY_ON);  // Enable Multifunction Contactor

          // Immediately change to state CONTACTORNEGMUL
          contactorsstate = CONTACTORSNEGMUL;

          // Start the contactorsstatetimer
          timeContactorStatus = millis();

          break;

        case CONTACTORSNEGMUL:
          // Check that the require time has passed in this state
          if (timeContactorStatus <  (millis() - DELAY_INTERRELAY))
          {
            // Switch on the PreCharge Contactor
            digitalWrite(Relay_X358_9, RELAY_ON);  // Enable Pre-charger +VE Contactor

            // Change to state CONTACTORSPRECRG
            contactorsstate = CONTACTORSPRECRG;

            // Restart the contactorsstatetimer
            timeContactorStatus = millis();
          }
          break;

        case CONTACTORSPRECRG:
          // Check that the require time has passed in this state
          if (timeContactorStatus <  (millis() - DELAY_PRECHARGE))
          {
            // Switch on the Positive Contactor
            digitalWrite(Relay_X358_10, RELAY_ON);  // Enable Main +VE Contactor

            // Change to state CONTACTORSPREPOS
            contactorsstate = CONTACTORSPREPOS;

            // Restart the contactorsstatetimer
            timeContactorStatus = millis();
          }
          break;

        case CONTACTORSPREPOS:
          // Check that the require time has passed in this state
          if (timeContactorStatus <  (millis() - DELAY_INTERRELAY))
          {
            // Switch off the PreCharge Contactor
            digitalWrite(Relay_X358_9, RELAY_OFF);  // Disable Pre-charger +VE Contactor

            // Change to state CONTACTORSCLOSED
            contactorsstate = CONTACTORSCLOSED;

            // Restart the contactorsstatetimer
            timeContactorStatus = millis();
          }
          break;

        default:
          // if nothing else matches, do the default
          break;
      }
    }

    // Request to OPEN (deactivate) the battery contactors
    if (contactorsreqdstate == CONTACTORSOPEN)
    {
      // Immediately deactivate all the contactors (open)
      digitalWrite(Relay_X358_10, RELAY_OFF);  // Disable Main +VE Contactor
      digitalWrite(Relay_X358_7, RELAY_OFF);  // Disable Main -VE Contactor
      digitalWrite(Relay_X358_8, RELAY_OFF);  // Disable Multifunction Contactor

      // Change to state CONTACTORSOPEN
      contactorsstate = CONTACTORSOPEN;
    }

  }  // Required state != Current state

  // Check for Request to change to a different BMS Communication Status
  if (bmsreqdstate != bmsstate)
  {

    // Request to turn on the BMS communications
    if (bmsreqdstate == BMSCOMMSON)
    {
      // Enable the signal lines
      digitalWrite(Relay_X357_4, RELAY_ON); // X357-3 Accessory wake-up
      digitalWrite(Relay_X357_5, RELAY_ON); // X357-14 HV communications enable

      // Update the status
      bmsstate = BMSCOMMSON;
    }

    // Request to turn off the BMS communications
    if (bmsreqdstate == BMSCOMMSOFF)
    {
      // Disable the signal lines
      digitalWrite(Relay_X357_5, RELAY_OFF); // X357-14 HV communications enable
      digitalWrite(Relay_X357_4, RELAY_OFF); // X357-3 Accessory wake-up

      // Update the status
      bmsstate = BMSCOMMSOFF;
    }
  }

  // Output a debug status message on regular frequency
  if (debug != 0)
  {
    if (timeLastStatusMsg <  (millis() - DELAY_STATUSMSG))
    {
      timeLastStatusMsg = millis();

      Serial.println();
      Serial.print("Time [");
      Serial.print(millis());
      Serial.print("] Contactor State: ");
      Serial.println(contactorsstate);
    }
  }
}

// Check for Instructions on the TesLorean CAN bus
void can0decode(CAN_FRAME & frame)
{
  switch (frame.id)
  {
    case 0x140: // BattConnFrameID : Battery HV Activation/Deactivation
      // 1 = Connect, 0 = Disconnect
      if (frame.data.bytes[0] == 0)
      {
        // Set the contactors required state to deactivated
        contactorsreqdstate = CONTACTORSOPEN;
      }
      if (frame.data.bytes[0] == 1)
      {
        // Set the contactors required state to activated
        contactorsreqdstate = CONTACTORSCLOSED;
      }
      break;

    case 0x142: // BMSConnFrameID : BMS Activation/Deactivation
      // 1 = Start, 0 = Stop
      if (frame.data.bytes[0] == 0)
      {
        // Activate the battery data communications
        bmsreqdstate = BMSCOMMSON;
      }
      if (frame.data.bytes[0] == 1)
      {
        // Deactivate the battery data communications
        bmsreqdstate = BMSCOMMSOFF;
      }
      break;

    case 0x151: // BattVoltReqFrameID : Start transmmission of individual voltages
      // Set the counters that signal transmission underway
      cellvoltmodule = 1;
      cellvolttriplet = 1;
      break;

    default:
      // if nothing else matches, do the default
      break;
  }
}

void Controller_CAN_Messages()
{
  CAN_FRAME outframe;  //A structured variable according to due_can library for transmitting CAN data.

  // Send out a Battery Contactor/BMS status message every 1sec
  if (timeBatteryStatus <  (millis() - DELAY_STATUSMSG))
  {
    // Send Battery Status Update message
    outframe.id = BattStatFrameID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request
    outframe.data.bytes[0] = contactorsstate;   // Status of the HV contactors
    outframe.data.bytes[1] = bmsstate;          // Status of the BMS communications
    outframe.data.bytes[2] = lowByte(totalpackvolts);
    outframe.data.bytes[3] = highByte(totalpackvolts);
    outframe.data.bytes[4] = lowByte(mincellvolts);
    outframe.data.bytes[5] = highByte(mincellvolts);
    outframe.data.bytes[6] = lowByte(maxcellvolts);
    outframe.data.bytes[7] = highByte(maxcellvolts);
    Can0.sendFrame(outframe);
  }

  // Send out a Battery module temperature message
  if (timeModTempsStatus <  (millis() - DELAY_MODFMSG))
  {
    // Send Battery Module Temps message
    outframe.id = BattModFFrameID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request
    outframe.data.bytes[0] = contactorsstate;   // Status of the HV contactors
    outframe.data.bytes[1] = bmsstate;          // Status of the BMS communications
    outframe.data.bytes[2] = lowByte(totalpackvolts);
    outframe.data.bytes[3] = highByte(totalpackvolts);
    outframe.data.bytes[4] = lowByte(mincellvolts);
    outframe.data.bytes[5] = highByte(mincellvolts);
    outframe.data.bytes[6] = lowByte(maxcellvolts);
    outframe.data.bytes[7] = highByte(maxcellvolts);
    Can0.sendFrame(outframe);
  }

  // Send out a Battery pack temperature message
  if (timePckTempsStatus <  (millis() - DELAY_PCKFMSG))
  {
    // Calculate the rate of voltage decline
    uint16_t deltatotalvolts = (totalpackvolts - priorpackvolts) * (60000/DELAY_PCKFMSG);
    
    // Send Battery Pack Temps message
    outframe.id = BattPckFFrameID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request
    outframe.data.bytes[0] = lowByte(packtemps[0]);
    outframe.data.bytes[1] = highByte(packtemps[0]);
    outframe.data.bytes[2] = lowByte(packtemps[1]);
    outframe.data.bytes[3] = highByte(packtemps[1]);
    outframe.data.bytes[4] = lowByte(deltatotalvolts);
    outframe.data.bytes[5] = highByte(deltatotalvolts);
    outframe.data.bytes[6] = 0;
    outframe.data.bytes[7] = 0;
    Can0.sendFrame(outframe);
  }

  // Check if individual voltages are being transmitted (two per cycle) and if so send the next one
  if (cellvoltmodule != 0 && cellvolttriplet !=0){
    // Send Battery Individual Cell Voltage
    outframe.id = BattVoltFrameID;
    outframe.length = 8;            // Data payload 8 bytes
    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
    outframe.rtr = 0;                 //No request
    outframe.data.bytes[0] = cellvoltmodule;
    outframe.data.bytes[1] = cellvolttriplet;
    outframe.data.bytes[2] = lowByte(cellvolts[cellvolttriplet-1][cellvoltmodule-1]);
    outframe.data.bytes[3] = highByte(cellvolts[cellvolttriplet-1][cellvoltmodule-1]);
    outframe.data.bytes[4] = lowByte(cellvolts[cellvolttriplet][cellvoltmodule-1]);
    outframe.data.bytes[5] = highByte(cellvolts[cellvolttriplet][cellvoltmodule-1]);
    outframe.data.bytes[6] = 0;
    outframe.data.bytes[7] = 0;
    Can0.sendFrame(outframe);

    // Progress to next one OR finish transmission
    cellvolttriplet = cellvolttriplet + 2;
    if (cellvolttriplet > 12){    
      cellvolttriplet = 1;
      cellvoltmodule++;
      if (cellvoltmodule > 8){
        cellvoltmodule = 0;
        cellvolttriplet = 0; 
      }
    }
  }

}
