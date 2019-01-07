/*
  TesLorean Thermal Control Module
  2018
  Jeff Cooke

  Coding Todos
  - Receive temp reports via CAN (HSM, DCDC, Battery, Charger)
    - DONE: Battery (via Battery Control Module)
    - HSM Motor Controller (Drive Unit and Inverter)
    - DCDC Converter
    - Charger
  - Serial port commands (e.g. reset)
  - Consider use of PWM on Heater/AC Compressor/Diverters to graduate the heating/cooling
  - Set/Clear the 'accabin' flag if the HVAC Controller requests AC

  Coding Notes
  - Establishes the InnerLoop and OuterLoop heating/cooling status by examining the max device temperatures and device temperature targets
  - Sets the pumps/valves/heater/chiller/accomp depending on the heating/cooling demand
  - Requires devices to operate for a minimum time before switching off/on
  - Sends a heartbeat message (to the Trip Computer)

*/
#define DEBUG 1

#include <mcp2515.h>
#include <SPI.h>
#include "config.h"

enum opmode {CHARGE, DRIVE, PARKED, POWEROFF};
enum loopmode {MODENONE ,COOLING, HEATING};
enum looplevel {LEVELNONE, LEVELLOW, LEVELHIGH};

//********* CAN IDs ******************
int ThermalStatusFrameID = 0x134;
int BattModFFrameID = 0x0133;
int BattPckFFrameID = 0x0131;

// DEFINE
#define StoredFrames 50

// Array for preprepared CAN frames
can_frame CANFrames[StoredFrames];

// Counters tracking the add point and read point in the circular array
volatile uint8_t addPointFrames = 0;
volatile uint8_t readPointFrames = 0;
volatile uint16_t netAddReadCount = 0;   // Adds increment, Reads decrement, only Read if <> 0

// Create objects, # is the chip select
MCP2515 can0(10);        // TesLorean CAN
  
void irqBATHandler()
{
  uint8_t irq = can0.getInterrupts();

  // check channel 0
  if (irq & MCP2515::CANINTF_RX0IF)
  {
    if (can0.readMessage(MCP2515::RXB0, &CANFrames[addPointFrames]) == MCP2515::ERROR_OK)
    {
        // frame contains received from RXB0 message
        addPointFrames = (addPointFrames + 1) % StoredFrames;
        netAddReadCount++;
    }
  }
            
  // check channel 1
  if (irq & MCP2515::CANINTF_RX1IF)
  {
    if (can0.readMessage(MCP2515::RXB1, &CANFrames[addPointFrames]) == MCP2515::ERROR_OK)
    {
      // frame contains received from RXB1 message
      addPointFrames = (addPointFrames + 1) % StoredFrames;
      netAddReadCount++;
    }
  } 
}

// Relay Statuses
#define RELAY_ON 0
#define RELAY_OFF 1

// Global Temperature Variables 1 = 1F
uint16_t moduletemp = 0;
uint16_t packtemp = 0;
uint16_t driveunittemp = 0;
uint16_t invertertemp = 0;
uint16_t dcdctemp = 0;
uint16_t chargertemp = 0;
uint16_t innertemp = 0;
uint16_t outertemp = 0;

// Global status variables
bool fan1on = false;
bool fan2on = false;
uint8_t fanonnum = 0;
bool heateron = false;
bool accompon = false;
bool accabin = false;
bool chilleron = false;
bool pump1on = false;
uint8_t pump1speed = 0;
bool pump2on = false;
uint8_t pump2speed = 0;
bool pump3on = false;
uint8_t pump3speed = 0;
bool bypass1open = false;   // Radiator
bool bypass2open = false;   // Chiller
bool loopconopen = false;

// Global Operational Variables
opmode carmode = PARKED;
loopmode innermode = COOLING;
looplevel innerlevel = LEVELNONE;
loopmode outermode = COOLING;
looplevel outerlevel = LEVELNONE;

// TIME SINCE VARIABLES
unsigned long timesinceStatus = 0;    // ms since last Heatbeat message
unsigned long timesinceFanStart = 0;  // ms since last Fan switched on
unsigned long timesinceHeaterStart = 0;
unsigned long timesinceACCompStart = 0;
unsigned long timesinceRadiatorStart = 0;
unsigned long timesinceLoopStart = 0;
unsigned long timesinceFanStop = 0;  // ms since last Fan switched on
unsigned long timesinceHeaterStop = 0;
unsigned long timesinceACCompStop = 0;
unsigned long timesinceRadiatorStop = 0;
unsigned long timesinceLoopStop = 0;

void reset()
{
  carmode = PARKED;
  innermode = COOLING;
  innerlevel = LEVELNONE;
  outermode = COOLING;
  outerlevel = LEVELNONE;
  
  // Fans
  fan1on = false;
  digitalWrite(PIN_OUT_FAN_1, LOW);
  fan2on = false;
  digitalWrite(PIN_OUT_FAN_2, LOW);
  fanonnum = 0;

  // Coolant Heater PWM
  heateron = false;
  analogWrite(PIN_OUT_HEATER_PWM, 0);

  // AC Compressor PWM
  accompon =false;
  accabin = false;
  analogWrite(PIN_OUT_ACCOMP_PWM, 0);

  // Chiller on/off
  chilleron = false;
  digitalWrite(PIN_OUT_CHILLER_DIG, LOW);

  // Coolant Pumps
  pump1on = false;
  pump1speed = 0;
  digitalWrite(PIN_OUT_PUMP1_DIG, LOW);
  analogWrite(PIN_IN_PUMP1_ANA, 0);
  pump2on = false;
  pump2speed = 0;
  digitalWrite(PIN_OUT_PUMP2_DIG, LOW);
  analogWrite(PIN_IN_PUMP2_ANA, 0);
  pump3on = false;
  pump3speed = 0;
  digitalWrite(PIN_OUT_PUMP3_DIG, LOW);
  analogWrite(PIN_IN_PUMP3_ANA, 0);

  // Diverter Valves
  bypass1open = false;    // Radiator
  digitalWrite(PIN_OUT_OPEN_BYPASS_1, LOW);
  digitalWrite(PIN_OUT_CLOSE_BYPASS_1, HIGH);
  analogWrite(PIN_OUT_PWM_BYPASS_1, 0);
  bypass2open = false;    // Chiller
  digitalWrite(PIN_OUT_OPEN_BYPASS_2, LOW);
  digitalWrite(PIN_OUT_CLOSE_BYPASS_2, HIGH);
  analogWrite(PIN_OUT_PWM_BYPASS_2, 0);
  loopconopen = false;
  digitalWrite(PIN_OUT_OPEN_LOOP, LOW);
  digitalWrite(PIN_OUT_CLOSE_LOOP, HIGH);
  analogWrite(PIN_OUT_PWM_LOOP, 0);
}

void setup()
{
  // Set up Serial port for test input and outputs
  while (!Serial);
  Serial.begin(115200);
  SPI.begin();    // init the SPI communications

  // Initialize the array pointers
  addPointFrames = 0;
  readPointFrames = 0;
  netAddReadCount = 0;
  
  // Set up TESLOREAN CAN interface
  can0.reset();
  can0.setBitrate(CAN_500KBPS);
  can0.setNormalMode();

  // Set up signal lines to the Fans
  pinMode(PIN_OUT_FAN_1, OUTPUT);
  pinMode(PIN_OUT_FAN_2, OUTPUT);  

  // Set up signal lines to the Pumps
  pinMode(PIN_IN_PUMP1_ANA, INPUT);
  pinMode(PIN_OUT_PUMP1_DIG, OUTPUT);
  pinMode(PIN_IN_PUMP2_ANA, INPUT);
  pinMode(PIN_OUT_PUMP2_DIG, OUTPUT);
  pinMode(PIN_IN_PUMP3_ANA, INPUT);
  pinMode(PIN_OUT_PUMP3_DIG, OUTPUT);
        
  // Set up signal lines to the Valves
  pinMode(PIN_OUT_OPEN_BYPASS_1, OUTPUT); // Radiator
  pinMode(PIN_OUT_CLOSE_BYPASS_1, OUTPUT);
  pinMode(PIN_OUT_OPEN_BYPASS_2, OUTPUT);   // Chiller
  pinMode(PIN_OUT_CLOSE_BYPASS_2, OUTPUT);
  pinMode(PIN_OUT_OPEN_LOOP, OUTPUT);
  pinMode(PIN_OUT_CLOSE_LOOP, OUTPUT);
  
  // Set up the PWM line to the AC Compressor and Chiller
  pinMode(PIN_OUT_ACCOMP_PWM, OUTPUT);
  pinMode(PIN_OUT_CHILLER_DIG, OUTPUT);

  // Set up the line to the Coolant Heater
  pinMode(PIN_OUT_HEATER_PWM, OUTPUT);

  // Reset all devices
  reset();
  
  // CAN Interrupt
  attachInterrupt(digitalPinToInterrupt(PIN_IN_CAN_INTERRUPT), irqBATHandler, LOW);
}

void loop()
{
  // THERMAL MONITORING LOGIC

  // Inner Loop
  if (moduletemp > BATTERY_MAX_F || packtemp > BATTERY_MAX_F || dcdctemp > DCDC_MAX_F)
  {
    innermode = COOLING;
    innerlevel = LEVELLOW;
    if (moduletemp > BATTERY_HIMAX_F || packtemp > BATTERY_HIMAX_F || dcdctemp > DCDC_HIMAX_F)
    {
      innermode = COOLING;
      innerlevel = LEVELHIGH;
    }
    else
    {
      if (moduletemp < BATTERY_MIN_F || packtemp < BATTERY_MIN_F)
      {
        innermode = HEATING;
        innerlevel = LEVELHIGH;
      }
      else
      {
        innermode = MODENONE;
        innerlevel = LEVELNONE;
      }
    }
  }

  // Outer Loop
  if (driveunittemp > DRIVEUNIT_MAX_F || invertertemp > DRIVEUNIT_MAX_F || chargertemp > DCDC_MAX_F)
  {
    outermode = COOLING;
    outerlevel = LEVELLOW;
    if (driveunittemp > DRIVEUNIT_HIMAX_F || invertertemp > DRIVEUNIT_MAX_F || chargertemp > DCDC_HIMAX_F)
    {
      outermode = COOLING;
      outerlevel = LEVELHIGH;
    }
    else
    {
      outermode = MODENONE;
      outerlevel = LEVELNONE;
    }
  }

  // PUMP AND VALVE ACTIONS

  // Outer Coolant Pump (#1)
  // Default : Always running when charger or drive unit activated
  // Inner Cooland Pump (#2)
  // Default : Always running when charger or drive unit activated
  if (carmode == CHARGE || carmode == DRIVE)
  {
    // Switch on the #1 outer loop coolant pump
    if (pump1on == false)
    {
      pump1speed = 255;
      digitalWrite(PIN_OUT_PUMP1_DIG, HIGH);
      analogWrite(PIN_IN_PUMP1_ANA, pump1speed);
      pump1on = true;
    }
    // Switch on the #2 inner loop coolant pump
    if (pump2on == false)
    {
      pump2speed = 255;
      digitalWrite(PIN_OUT_PUMP2_DIG, HIGH);
      analogWrite(PIN_IN_PUMP2_ANA, pump2speed);
      pump2on = true;
    }
  }
  else
  {
    // Switch off the pumps
      pump1speed = 0;
      digitalWrite(PIN_OUT_PUMP1_DIG, LOW);
      analogWrite(PIN_IN_PUMP1_ANA, pump1speed);
      pump1on = false;    

      pump2speed = 0;
      digitalWrite(PIN_OUT_PUMP2_DIG, LOW);
      analogWrite(PIN_IN_PUMP2_ANA, pump2speed);
      pump2on = false;    
  }

  // Radiator Diverter
  // Default : Bypass; Open : when Outer = Cooling
  if (outermode = COOLING)
  {
    // Keep Radiator closed for at least the min run time...
    if(timesinceRadiatorStop > (millis() - RADIATOR_MIN_RUN_MS))
    {
      bypass1open = true;
      digitalWrite(PIN_OUT_CLOSE_BYPASS_1, LOW);
      digitalWrite(PIN_OUT_OPEN_BYPASS_1, HIGH);
      analogWrite(PIN_OUT_PWM_BYPASS_1, 255);
      timesinceRadiatorStart = millis();
    }
  }
  if (outermode = MODENONE)
  {
    // Keep Radiator open for at least the min run time...
    if(timesinceRadiatorStart > (millis() - RADIATOR_MIN_RUN_MS))
    {
      bypass1open = false;
      digitalWrite(PIN_OUT_OPEN_BYPASS_1, LOW);
      digitalWrite(PIN_OUT_CLOSE_BYPASS_1, HIGH);
      analogWrite(PIN_OUT_PWM_BYPASS_1, 0);
      timesinceRadiatorStop = millis();
    }
  }

  // AC Compressor and Chiller
  // Default : Off ; On : when Inner = Chilling and Cooling Level high or ac cabin requested ; Note : AC Compressor is PWM controlled
  if (accabin == true || (innermode == COOLING && innerlevel == LEVELHIGH))
  {
    // Keep AC compressor from running for at least the min run time...
    if(timesinceACCompStop > (millis() - ACCOMP_MIN_RUN_MS))
    {
      // AC Compressor PWM
      accompon = true;
      analogWrite(PIN_OUT_ACCOMP_PWM, 255);
      timesinceACCompStart = millis();
  
      if (innermode == COOLING && innerlevel == LEVELHIGH)
      {
        // Chiller on
        chilleron = true;
        digitalWrite(PIN_OUT_CHILLER_DIG, HIGH);
  
        // Chiller bypass open (runs past chiller)
        bypass2open = true;
        digitalWrite(PIN_OUT_CLOSE_BYPASS_2, LOW);
        digitalWrite(PIN_OUT_OPEN_BYPASS_2, HIGH);
        analogWrite(PIN_OUT_PWM_BYPASS_2, 255);
      }
      else
      {
        // Chiller off
        chilleron = false;
        digitalWrite(PIN_OUT_CHILLER_DIG, LOW);
  
        // Chiller bypass closed (does not run past chiller)
        bypass2open = false;
        digitalWrite(PIN_OUT_OPEN_BYPASS_2, LOW);
        digitalWrite(PIN_OUT_CLOSE_BYPASS_2, HIGH);
        analogWrite(PIN_OUT_PWM_BYPASS_2, 0);
      }
    }
  }
  else
  {
    // Keep AC compressor running for at least the min run time...
    if(timesinceACCompStart > (millis() - ACCOMP_MIN_RUN_MS))
    {
      // AC Compressor PWM
      accompon = false;
      analogWrite(PIN_OUT_ACCOMP_PWM, 0);
      timesinceACCompStop = millis();
      
      // Chiller off
      chilleron = false;
      digitalWrite(PIN_OUT_CHILLER_DIG, LOW);
  
      // Chiller bypass closed (does not run past chiller)
      bypass2open = false;
      digitalWrite(PIN_OUT_OPEN_BYPASS_2, LOW);
      digitalWrite(PIN_OUT_CLOSE_BYPASS_2, HIGH);
      analogWrite(PIN_OUT_PWM_BYPASS_2, 0);
    }
  }

  // Radiator Fans
  // Default : Off ; On : when Outer = Cooling and Cooling level required high, or AC Compressor on
  if (accompon == true || (outermode == COOLING && outerlevel == LEVELHIGH))
  {
    // Keep fans running for at least the min run time...
    if(timesinceFanStop > (millis() - FAN_MIN_RUN_MS))
    {
      // Default to 2 fans, in future update set 0, 1, or 2 fans depending on demand
      fanonnum = 2;
      
      // Fans
      fan1on = true;
      digitalWrite(PIN_OUT_FAN_1, HIGH);
      timesinceFanStart = millis();
    }
  }
  else
  {
    // Keep fans running for at least the min run time...
    if(timesinceFanStart > (millis() - FAN_MIN_RUN_MS))
    {
      // Fans off
      fan1on = false;
      digitalWrite(PIN_OUT_FAN_1, LOW);
      fan2on = false;
      digitalWrite(PIN_OUT_FAN_1, LOW);
      timesinceFanStop = millis();
      fanonnum = 0;
    }
  }
  // If needed switch 2nd fan on after delay
  if (fanonnum == 2 && fan1on && !fan2on && (timesinceFanStart <  (millis() - FAN_DELAY_MS)))
  {
    // Switch second fan on
    fan2on = true;
    digitalWrite(PIN_OUT_FAN_2, HIGH);
  }

  // Coolant Heater
  // Default : Off ; On : when Inner = Heating and Heating Level high ; Note : Heater is PWM controlled
  if (innermode == HEATING && innerlevel == LEVELHIGH)
  {
    // Keep heater from running for at least the min run time...
    if(timesinceHeaterStop > (millis() - HEATER_MIN_RUN_MS))
    {
      heateron = true;
      analogWrite(PIN_OUT_HEATER_PWM, 255);
      timesinceHeaterStart = millis();
    }
  }
  else
  {
    // Keep heater running for at least the min run time...
    if(timesinceHeaterStart > (millis() - HEATER_MIN_RUN_MS))
    {
      heateron = false;
      analogWrite(PIN_OUT_HEATER_PWM, 0);
      timesinceHeaterStop = millis();
    }
  }

  // Inner & Outer Loop Diverter (joins the two loops)
  // Default : Separate ; Joined : Inner Temp > Outer Temp (by X degrees?) AND Inner Temp = Cooling low or high
  if (innertemp > outertemp && innermode == COOLING)
  {
    // Keep loop disconnected for at least the min run time...
    if(timesinceLoopStop > (millis() - LOOP_MIN_RUN_MS))
    {
      // Join the loops to allow the inner to use the outer cooling loop
      loopconopen = true;
      timesinceLoopStart = millis();
      digitalWrite(PIN_OUT_CLOSE_LOOP, LOW);
      digitalWrite(PIN_OUT_OPEN_LOOP, HIGH);
      analogWrite(PIN_OUT_PWM_LOOP, 255);
    }
  }
  else
  {
    // Keep loop connected for at least the min run time...
    if(timesinceLoopStart > (millis() - LOOP_MIN_RUN_MS))
    {
      // Keep the loops separate
      loopconopen = false;
      timesinceLoopStop = millis();
      digitalWrite(PIN_OUT_OPEN_LOOP, LOW);
      digitalWrite(PIN_OUT_CLOSE_LOOP, HIGH);
      analogWrite(PIN_OUT_PWM_LOOP, 0);
    }
  }
  // Separate the loops once the inner loop has cooled by LOOP_TEMP_DIFF
  if  (innertemp < outertemp - LOOP_TEMP_DIFF)
  {
    // Keep loop connected for at least the min run time...
    if(timesinceLoopStart > (millis() - LOOP_MIN_RUN_MS))
    {
      loopconopen = false;
      timesinceLoopStop = millis();
      digitalWrite(PIN_OUT_OPEN_LOOP, LOW);
      digitalWrite(PIN_OUT_CLOSE_LOOP, HIGH);
      analogWrite(PIN_OUT_PWM_LOOP, 0);
    }
  }

  // CAN FRAME HANDLING
      
  // Check for received frames
  if (netAddReadCount > 0)
  {
    if (readPointFrames != addPointFrames)
    {
      // Process the frame received
      can0decode(CANFrames[readPointFrames]);

      // Move on to the next frame to read
      readPointFrames = (readPointFrames + 1) % StoredFrames;
      if(netAddReadCount > 0){netAddReadCount--;}
    }
  }

  // Selectively send out status and informational messages
  Controller_CAN_Messages;

  // Check the Serial Bus for commands
  if (Serial.available())
  {
    char incomingByte;
    incomingByte = Serial.read(); // read the incoming byte:

    switch (incomingByte)
    {
      case 'd'://d for display
        Serial.println();
        Serial.print("Status : ");
        Serial.println(incomingByte);
        break;
        
      case 'r'://d for reset
        reset();
        Serial.println();
        Serial.println("Reset Controller : Done ");
        break;
        
      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }
  }

};

// Process instructions on the TesLorean CAN bus
void can0decode(can_frame & frame)
{
  switch (frame.can_id)
  {
    // BattPckFFrameID : Battery Pack Temps
    case 0x131: 
      uint16_t pack1temp;
      uint16_t pack2temp;
      pack1temp = (frame.data[1] << 8) + frame.data[0];
      pack2temp = (frame.data[3] << 8) + frame.data[2];
      moduletemp = pack1temp;
      if (pack2temp > pack1temp) {moduletemp = pack2temp;}
      break;
      
    // BattModFFrameID : Battery Module Temps
    case 0x133: 
      uint8_t maxtemp;
      maxtemp = 0;
      for (int i = 0 ; i < frame.can_dlc ; i++)
      {
        if (frame.data[i] > maxtemp){maxtemp = frame.data[i];}
      }
      packtemp = maxtemp;
      break;

    // Driveunit and Inverter temperatures

    // DCDC Converter temperature

    // Charger temperature
      
    default:
      // if nothing else matches, do the default
      break;
  }
}

void Controller_CAN_Messages()
{
  can_frame outframe;  //A structured variable according to due_can library for transmitting CAN data.

  // Send out a Thermal Controller status heartbeat message every 1 sec
  if (timesinceStatus <  (millis() - DELAY_STATUSMSG))
  {
    // Send Battery Status Update message
    outframe.can_id = ThermalStatusFrameID;
    outframe.can_dlc = 8;            // Data payload 8 bytes
//    outframe.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
//    outframe.rtr = 0;                 //No request
    outframe.data[0] = 0;
    outframe.data[1] = 0;
    outframe.data[2] = 0;
    outframe.data[3] = 0;
    outframe.data[4] = 0;
    outframe.data[5] = 0;
    outframe.data[6] = 0;
    outframe.data[7] = 0;
    can0.sendMessage(&outframe);
    
    timesinceStatus = millis();
  }
}
