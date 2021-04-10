/**********************************************************************************
   Carduino 5000

   This sketch combines CAN BUS and the 4D Systems library to read parameters
   from a Megasquirt and output the values to multiple 4D displays.

   Libraries
   https://github.com/coryjfowler/MCP_CAN_lib
   https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html
   https://github.com/4dsystems/ViSi-Genie-Arduino-Library
   http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf - page 6 is the breakout of the IDs


   Default Base ID: 0x5E8 (1520)

   Example:
   ID: 0x5F2 (1520 + 2 = 1522) = Barometer, MAP, MAT, and Coolant Temp
a
   Barometer (kPa) = (message[0] << 8) + message[1]) / 10.0;
   MAP (kPa) = (message[2] << 8) + message[3]) / 10.0;
   MAT (deg F) = (message[4] << 8) + message[5]) / 10.0;
   Coolang Temp (deg F) = (message[6] << 8) + message[7]) / 10.0;


*/

// 4D Systems library for Genie designer displays
// #include <genieArduino.h>

// CAN BUS libraries
#include <mcp_can.h>
#include <SPI.h>

// Queueing Library
#include "Queue.h"

// Add library for timed delays (pretend multi tasking)
#include <millisDelay.h>

// EEPROM library for reading/writing to onboard memory (for the odometer)
#include <EEPROM.h>

// Initialize variable to run demo or not
bool runningDemo = false;

// Initialize Debug Setting 
bool debug = false;

// Variables for the CAN BUS messages
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
int processedMessages = 0;

// PIN identification for the CAN BUS shield
#define CAN0_INT 2   // Set INT to pin 2
MCP_CAN CAN0(9);     // Set CS to pin 9

// Delay timers - These are used for setting delays for pretend multi-tasking
millisDelay speedometerDelay;
millisDelay tachometerDelay;
millisDelay pressureDelay;
millisDelay temperatureDelay;
millisDelay fuelDelay;
millisDelay demoDelay;
millisDelay odometerWriteDelay;
millisDelay serialQueueDelay;
millisDelay readAnalogSensorValuesDelay;
millisDelay readDigitalSensorValuesDelay;

// Variables for display screen values
int tachometerValue = 0;
int speedometerValue = 0;
int fuelValue = 0;
int oilPressureValue = 0;
int fuelPressureValue = 0;
int batteryVoltageValue = 0;
int gearNumberValue = 0;
int32_t odometerValue = 782460; // initial mileage from bill of sale
int barometerValue = 0;
int MAPValue = 0;
int MATValue = 0;
int coolantTemperatureValue = 0;
int coolantPressureValue = 0;
int transmissionTemperatureValue = 0;
int transmissionPressureValue = 0;
int throttlePositionValue = 0;
int boostValue = 0;


// Define pins to reset the displays
#define RESETLINE1 22
#define RESETLINE2 24
#define RESETLINE3 26

// We need to force a delay after each write otherwise the displays take a dump
#define WRITE_OBJECT_DELAY 0

#define MAIN_TACHOMETER_ARROWS       2
#define MAIN_TACHOMETER_DIGITS       0
#define MAIN_SPEEDOMETER_ARROWS      1
#define MAIN_SPEEDOMETER_DIGITS      1
#define MAIN_FUEL_GAUGE              3
#define MAIN_OIL_PRESSURE_GAUGE      4
#define MAIN_COOLANT_TEMP_GAUGE      5
#define MAIN_BATTERY_VOLTAGE_GAUGE   6
#define MAIN_GEAR_SELECTOR           0
#define MAIN_ODOMETER_DIGITS         2
#define SIDE_TOP_TEXT                0
#define SIDE_TOP_UNITS               5
#define SIDE_TOP_DIGITS              1
#define SIDE_TOP_RIGHT_TEXT          1
#define SIDE_TOP_RIGHT_UNITS         6
#define SIDE_TOP_RIGHT_DIGITS        0
#define SIDE_TOP_LEFT_TEXT           2
#define SIDE_TOP_LEFT_UNITS          7
#define SIDE_TOP_LEFT_DIGITS         2
#define SIDE_BOTTOM_RIGHT_TEXT       3
#define SIDE_BOTTOM_RIGHT_UNITS      8
#define SIDE_BOTTOM_RIGHT_DIGITS     4
#define SIDE_BOTTOM_LEFT_TEXT        4
#define SIDE_BOTTOM_LEFT_UNITS       9
#define SIDE_BOTTOM_LEFT_DIGITS      3


#define GENIE_OBJ_FORM                  10
#define GENIE_OBJ_USERIMAGES            27
#define GENIE_OBJ_LED_DIGITS            15
#define GENIE_OBJ_STRINGS               17
#define GENIE_OBJ_SOUND                 22
#define GENIE_OBJ_ILED_DIGITS_H         38
#define GENIE_OBJ_ILED_DIGITS_L         47
#define GENIE_OBJ_ILED_DIGITS           47
#define GENIE_WRITE_STR                 2
#define GENIE_OBJ_SOUND_VOLUME          999

#define MAIN_DISPLAY_SCREEN Serial1
#define LEFT_DISPLAY_SCREEN Serial2
#define RIGHT_DISPLAY_SCREEN Serial3

#define FUEL_SENDER_INPUT A0
#define COOLANT_PRESSURE_INPUT A1
#define TRANSMISSION_PRESSURE_INPUT A10

#define GEAR_P_INPUT A3
#define GEAR_R_INPUT A13
#define GEAR_N_INPUT A5
#define GEAR_D_INPUT A12
#define GEAR_2_INPUT A11
#define GEAR_L_INPUT A4

int message_counter = 0;
int demoCounter = 0;

typedef struct {
  Stream *serialPort;
  uint16_t object;
  uint16_t index;
  uint16_t digitsValue;
  uint16_t imageIndex;
  char *stringsValue;

} DisplayScreenSerialMessage;

union FloatLongFrame {
  float floatValue;
  int32_t longValue;
  uint32_t ulongValue;
  int16_t wordValue[2];
};


Queue<DisplayScreenSerialMessage> displayMessageSerialQueue = Queue<DisplayScreenSerialMessage>(50);

void processSerialQueue() {
  if (serialQueueDelay.justFinished()) {
    serialQueueDelay.repeat(); // start delay again without drift

    int messageCount = displayMessageSerialQueue.count();
    
    if (debug) {
      Serial.println("Processed Messages: ");
      Serial.println(messageCount);
    }

    if (messageCount > 0) {

      DisplayScreenSerialMessage serialMessage = displayMessageSerialQueue.pop();

      switch (serialMessage.object) {
        case GENIE_OBJ_ILED_DIGITS:
          writeObjectToDisplay(*serialMessage.serialPort, GENIE_OBJ_ILED_DIGITS, serialMessage.index, serialMessage.digitsValue);
          break;
        case GENIE_OBJ_USERIMAGES:
          writeObjectToDisplay(*serialMessage.serialPort, GENIE_OBJ_USERIMAGES, serialMessage.index, serialMessage.digitsValue);
          break;
        case GENIE_WRITE_STR:
          writeStringToDisplay(*serialMessage.serialPort, serialMessage.index, serialMessage.stringsValue);
          break;
        case GENIE_OBJ_SOUND_VOLUME:
          writeObjectToDisplay(*serialMessage.serialPort, GENIE_OBJ_SOUND, 1, 25);
          break;
        case GENIE_OBJ_SOUND:
          writeObjectToDisplay(*serialMessage.serialPort, GENIE_OBJ_SOUND, 0, 0);
          break;
      }

    }





  }
}

void writeLongIntoEEPROM(int address, long number)
{
  EEPROM.write(address, (number >> 24) & 0xFF);
  EEPROM.write(address + 1, (number >> 16) & 0xFF);
  EEPROM.write(address + 2, (number >> 8) & 0xFF);
  EEPROM.write(address + 3, number & 0xFF);
}

long readLongFromEEPROM(int address)
{
  return ((long)EEPROM.read(address) << 24) +
         ((long)EEPROM.read(address + 1) << 16) +
         ((long)EEPROM.read(address + 2) << 8) +
         (long)EEPROM.read(address + 3);
}

void initializeScreenLabels() {

  Serial.println("Initiate Display Screen Labels");

  // Play a fun little 'ding'
  // NOTE: SO this makes the screens puke on startup - I imagine all memory is needed to play the file and it croaks
  //playScreenSound();

  // MAP
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_LEFT_TEXT, 0, 0, "      MAP");
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_LEFT_UNITS, 0, 0, "        psi");

  // MAT
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_LEFT_TEXT, 0, 0, "         MAT");
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_LEFT_UNITS, 0, 0, "     deg F");

  // Fuel Pressure
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_RIGHT_TEXT, 0, 0, "      FUEL");
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_RIGHT_UNITS, 0, 0, "        psi");

  // Coolant Temperature
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_LEFT_TEXT, 0, 0, " COOLANT");
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_LEFT_UNITS, 0, 0, "      deg F");

  // Coolant Pressure
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_RIGHT_TEXT, 0, 0, "  COOLANT");
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_RIGHT_UNITS, 0, 0, "        psi");

  // Oil Pressure
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_TEXT, 0, 0, "        OIL");
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_UNITS, 0, 0, "      psi");

  // Battery Voltage
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_TEXT, 0, 0, "  BATTERY");
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_UNITS, 0, 0, "     volts");

  // Battery AMPS
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_RIGHT_TEXT, 0, 0, "      BARO");
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_RIGHT_UNITS, 0, 0, "        psi");

  // Transmission Temperature
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_LEFT_TEXT, 0, 0, "      TRANS");
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_LEFT_UNITS, 0, 0, "    psi");

  // Transmission Pressure
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_RIGHT_TEXT, 0, 0, "THROTTLE");
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_RIGHT_UNITS, 0, 0, "        %");

}

// Play the default sound on both screens
void playScreenSound() {

  // Set Volume
  //addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_SOUND, 0, 0, 0, "");
  //addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_SOUND_VOLUME, 0, 0, 0, "");
  
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_SOUND, 0, 0, 0, "");
  //addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_SOUND, 0, 0, 0, "");
}

// In Debug Mode, this function outputs to serial the values of all Analog Inputs
// For the MEGA, there are 15 analog inputs
void readAllAnalogInputs()
{
  Serial.print("A0: ");
  Serial.println(analogRead(0));
  Serial.print("A1: ");
  Serial.println(analogRead(1));
  Serial.print("A2: ");
  Serial.println(analogRead(2));
  Serial.print("A3: ");
  Serial.println(analogRead(3));
  Serial.print("A4: ");
  Serial.println(analogRead(4));
  Serial.print("A5: ");
  Serial.println(analogRead(5));
  Serial.print("A6: ");
  Serial.println(analogRead(6));
  Serial.print("A7: ");
  Serial.println(analogRead(7));
  Serial.print("A8: ");
  Serial.println(analogRead(8));
  Serial.print("A9: ");
  Serial.println(analogRead(9));
  Serial.print("A10: ");
  Serial.println(analogRead(10));
  Serial.print("A11: ");
  Serial.println(analogRead(11));
  Serial.print("A12: ");
  Serial.println(analogRead(12));
  Serial.print("A13: ");
  Serial.println(analogRead(13));
  Serial.print("A14: ");
  Serial.println(analogRead(14));
  Serial.print("A15: ");
  Serial.println(analogRead(15));
}


// Setup function

void setup()
{
  // Initialize the debug Serial if needed
  Serial.begin(115200);

  // Initialize the Serial pins for the displays. 115200 should be sufficient.
  Serial1.begin(115200);  // Serial1 @ 115200 Baud
  Serial2.begin(115200);  // Serial2 @ 115200 Baud
  Serial3.begin(115200);  // Serial3 @ 115200 Baud


  // Initialize MCP2515 running a 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);

  // Configuring pin for /INT input
  pinMode(CAN0_INT, INPUT);

  // Reset the Displays
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.

  // SOOOOOO None of this is hooked up so I think the only thing needed is the delay below
//  pinMode(RESETLINE1, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
//  pinMode(RESETLINE2, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
//  pinMode(RESETLINE3, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
//  digitalWrite(RESETLINE1, 0);  // Reset the Display via D4
//  digitalWrite(RESETLINE2, 0);  // Reset the Display via D4
//  digitalWrite(RESETLINE3, 0);  // Reset the Display via D4
//  delay(200);
//  digitalWrite(RESETLINE1, 1);  // unReset the Display via D4
//  digitalWrite(RESETLINE2, 1);  // unReset the Display via D4
//  digitalWrite(RESETLINE3, 1);  // unReset the Display via D4

  delay (3000); //let the display start up after the reset (This is important)


  // Set all the default labels for the screens
  initializeScreenLabels();

  // Set the queue delay for processing serial communications to the screens
  // 5 milliseconds seems to be the minimum to not have the screens puke
  serialQueueDelay.start(7);

  if (runningDemo == true) {
    demoDelay.start(200); // 10 times per second
  }
  else
  {

    // Start Delays for Dipslay Screens (in milliseconds)
    speedometerDelay.start(100); // 10 times per second
    tachometerDelay.start(100); // 10 times per second
    pressureDelay.start(1000); // 1 time per second
    temperatureDelay.start(1000); // 1 time per second
    fuelDelay.start(5000); // // Every 5 seconds
    odometerWriteDelay.start(5000); // Every 5 seconds
    readAnalogSensorValuesDelay.start(1000); // Every second
    readDigitalSensorValuesDelay.start(250); // Twice per second

    // Read initial value of odometer from EEPROM
    odometerValue = readLongFromEEPROM(0);

  }

}

void readSensorValues() {

  // **** Read Analog Sensor Values ****
  if (readAnalogSensorValuesDelay.justFinished()) {
    readAnalogSensorValuesDelay.repeat(); // start delay again without drift
    int fuelAnalogReading = analogRead(0);
    int transmissionPressureAnalogReading = analogRead(1);
    int coolantPressureAnalogReading = analogRead(2);

    // Pressure is from 0 (0.5v) to 150psi (4.5v)
    transmissionPressureValue = (transmissionPressureAnalogReading * (5.0 / 1023.0) - 0.5) * 37.5 * 10.0;
    coolantPressureValue = (coolantPressureAnalogReading * (5.0 / 1023.0) - 0.5) * 37.5 * 10.0;
    
    //Serial.print("TRANS: ");
    //Serial.println(transmissionPressureValue);
    //Serial.print("COOLANT: ");
    //Serial.println(coolantPressureValue);

    
    //int transmissionTemperatureAnalogReading = analogRead(2);
    //if (transmissionTemperatureAnalogReading > 0) {
      //transmissionTemperatureValue = 148.0 + 65.5 * log(transmissionTemperatureAnalogReading);
    //}

    


  }

  if (readDigitalSensorValuesDelay.justFinished()) {
    readDigitalSensorValuesDelay.repeat(); // start delay again without drift
    // Transmission Gear

    // Transmission gears are read through analog inputs
    // The active gear gets 2.5v (about 480 from the analog read)
    // Inactive gears will read 0v
    // the range of active will be from 400-550
    
    int gearP = analogRead(GEAR_P_INPUT);
    int gearR = analogRead(GEAR_R_INPUT);
    int gearN = analogRead(GEAR_N_INPUT);
    int gearD = analogRead(GEAR_D_INPUT);
    int gear2 = analogRead(GEAR_2_INPUT);
    int gearL = analogRead(GEAR_L_INPUT);

    if (gearP > 400 && gearP < 550) gearNumberValue = 1;
    else if (gearR > 400 && gearR < 550) gearNumberValue = 2;
    else if (gearN > 400 && gearN < 550) gearNumberValue = 3;
    else if (gearD > 400 && gearD < 550) gearNumberValue = 4;
    else if (gear2 > 400 && gear2 < 550) gearNumberValue = 5;
    else if (gearL > 400 && gearL < 550) gearNumberValue = 6;

    // Update the Gear
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_GEAR_SELECTOR, gearNumberValue, 0, "");

  }

}

void updateOdometer() {

  if (odometerWriteDelay.justFinished()) {

    // start delay again without drift
    odometerWriteDelay.repeat();

    char cstr[12] = "";
    String odometerString = String(odometerValue/10) + " mi";

    odometerString.toCharArray(cstr, 12);

    char *odometerCharacters = cstr;

    if (debug) {
      Serial.println("Odometer Value: ");
      Serial.println(odometerCharacters);
    }

    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_WRITE_STR, MAIN_ODOMETER_DIGITS, 0, 0, odometerCharacters);

    // Write the new odometer value to eeprom
    writeLongIntoEEPROM(0, odometerValue);

  }

}

void updateDisplayScreens() {

  // **** Speedometer ****
  if (speedometerDelay.justFinished()) {
    speedometerDelay.repeat(); // start delay again without drift

     addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, MAIN_SPEEDOMETER_DIGITS, speedometerValue, 0, "");
     addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_SPEEDOMETER_ARROWS, ceil(speedometerValue / 170.0), 0, "");

  }

  // **** Tachometer ****
  if (tachometerDelay.justFinished()) {
    tachometerDelay.repeat(); // start delay again without drift

    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_RIGHT_DIGITS, throttlePositionValue, 0, "");
    
     addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, MAIN_TACHOMETER_DIGITS, tachometerValue, 0, "");
     addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_TACHOMETER_ARROWS, ceil(tachometerValue / 750.0), 0, "");
  }

  // **** Pressure Gauges ****
  if (pressureDelay.justFinished()) {
    pressureDelay.repeat(); // start delay again without drift

    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, batteryVoltageValue, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_RIGHT_DIGITS, barometerValue, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_LEFT_TEXT, MAPValue, 0, "");

    //trasn
    //coolantPressureValue
    //transmissionPressureValue
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_LEFT_DIGITS, transmissionPressureValue, 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_RIGHT_DIGITS, coolantPressureValue, 0, "");

   addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_RIGHT_DIGITS, fuelPressureValue, 0, "");
   addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, oilPressureValue, 0, "");



  // Oil pressure is 0-100psi on the gauge
  int oilPressureIndex = 0;
  if (oilPressureValue < 0) oilPressureIndex = 0;
  else if (oilPressureValue > 100) oilPressureIndex = 18;
  else {
    oilPressureIndex = oilPressureValue / 55.555;
  }
 
   addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_OIL_PRESSURE_GAUGE, oilPressureIndex, 0, "");

     int batteryVoltageIndex = 0;
    //coolantTemperatureValue = 2760;
    if (batteryVoltageValue <= 80) batteryVoltageIndex = 0;
    
    else if (batteryVoltageValue >= 160) batteryVoltageIndex = 18;
    
    else {
      batteryVoltageIndex = ((batteryVoltageValue - 80) / 80.0) * 18;

    }

    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_BATTERY_VOLTAGE_GAUGE, batteryVoltageIndex, 0, "");
  }

  // **** Temperature Gauges ****
  if (temperatureDelay.justFinished()) {
    temperatureDelay.repeat(); // start delay again without drift

    int coolantImageIndex = 0;
    //coolantTemperatureValue = 2760;
    if (coolantTemperatureValue <= 1000) coolantImageIndex = 0;
    else if (coolantTemperatureValue >= 2800) coolantImageIndex = 18;
    else {
      coolantImageIndex = ((coolantTemperatureValue - 1000) / 1800.0) * 18;

    }


    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_LEFT_DIGITS, coolantTemperatureValue, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_COOLANT_TEMP_GAUGE, coolantImageIndex, 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_LEFT_DIGITS, MATValue, 0, "");

  }

  // **** Fuel Gauges ****
  if (fuelDelay.justFinished()) {
    fuelDelay.repeat(); // start delay again without drift

    int val = analogRead(0);  // read the input pin
    
    // Tested this direct to the ground and it's 290 max 40 min
    if (val > 290) val = 290; // Tank Empty
    if (val < 40) val = 40; // Tank Full
    int fuelImageNumber = 18 - round((285 - 40)/13.68); // =Round((260-190)/5.26)

    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_FUEL_GAUGE, fuelImageNumber, 0, "");

    if (debug) {
      Serial.println(val);          // debug value
      Serial.print("Fuel Image: ");
      Serial.println(fuelImageNumber);
    }


    processedMessages = 0;
  }

}

int processCANBUSMessage() {

  // Read data: len = data length, buf = data byte(s)
  CAN0.readMsgBuf(&rxId, &len, rxBuf);

  // If the message is an extendedID, return without processing
  if ((rxId & 0x80000000) == 0x80000000)  {
    return -1;
  }

  //sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

  // 0x5F2 : Seconds, PW1, PW2, RPM
  if (rxId == 0x5F0) {
    tachometerValue = (rxBuf[6] << 8) + rxBuf[7]; // RPM
  }

  // 0x5F2 : Barometer, MAP, MAT, and Coolant Temp
  if (rxId == 0x5F2) {
    barometerValue = ((rxBuf[0] << 8) + rxBuf[1]) * 0.145038; // kpa
    MAPValue = ((rxBuf[2] << 8) + rxBuf[3]) * 0.145038; // kpa
    //MAPValue = ((rxBuf[2] << 8) + rxBuf[3]);
    MATValue = (rxBuf[4] << 8) + rxBuf[5]; // deg F
    coolantTemperatureValue = (rxBuf[6] << 8) + rxBuf[7]; // deg F
    //Serial.println(MAPValue);
    //Serial.println(MATValue);
  }

    // 0x5F3 : Throttle Position, Battery Voltage, EGO1 (unused), EGO2 = Fuel Pressure
  if (rxId == 0x5F3) {
    throttlePositionValue = ((rxBuf[0] << 8) + rxBuf[1]); // Percentage
    batteryVoltageValue = ((rxBuf[2] << 8) + rxBuf[3]); // Volts
  }


  // 0x5FD : Sensor 1 (Oil Pressure), 2 (Fuel Pressure), 3, and 4
  if (rxId == 0x5FD) {
    oilPressureValue = ((rxBuf[0] << 8) + rxBuf[1]); // psi
    fuelPressureValue = ((rxBuf[2] << 8) + rxBuf[3]); //  psi   
  }


  return 0;


}

void addSerialDisplayMessageToQueue(Stream &serialPort, uint16_t object, uint16_t index, uint16_t digitsValue, uint16_t imageIndex, char *stringsValue) {

  // Instantiate a new serial message
  DisplayScreenSerialMessage serialMessage;
  serialMessage.serialPort = &serialPort;
  serialMessage.object = object;
  serialMessage.index = index;
  serialMessage.digitsValue = digitsValue;
  serialMessage.imageIndex = imageIndex;  
  serialMessage.stringsValue = stringsValue;

  displayMessageSerialQueue.push(serialMessage);
}

void writeObjectToDisplay(Stream &serialPort, uint16_t object, uint16_t index, uint16_t data) {

  Stream* deviceSerial = &serialPort;
  uint16_t msb, lsb ;
  uint8_t checksum ;
  lsb = lowByte(data);
  msb = highByte(data);
  deviceSerial->write(1) ;
  checksum  = 1 ;
  deviceSerial->write(object) ;
  checksum ^= object ;
  deviceSerial->write(index) ;
  checksum ^= index ;
  deviceSerial->write(msb) ;
  checksum ^= msb;
  deviceSerial->write(lsb) ;
  checksum ^= lsb;
  deviceSerial->write(checksum) ;

  if (Serial1.available() ) {

    //while (Serial1.read() >= 0);
  }

  //delay(10);
}

uint16_t writeStringToDisplay (Stream &serialPort, uint16_t index, char *string) {

  
  char *p;
  unsigned int checksum;
  int len = strlen (string);

  if (len > 255) {
    return -1;
  }

  serialPort.write(GENIE_WRITE_STR);
  checksum  = GENIE_WRITE_STR;
  serialPort.write(index);
  checksum ^= index;
  serialPort.write((unsigned char)len);
  checksum ^= len;

  for (p = string ; *p ; ++p) {
    serialPort.write(*p);
    checksum ^= *p;
  }

  serialPort.write(checksum);

  return 0;
}

void processDemoLoop() {

  if (demoDelay.justFinished()) {

    // restart the delay again without drift
    demoDelay.repeat();

    // Updates 1 times per second
    demoCounter += 1;
    demoCounter = demoCounter % 10000;

    //Serial.println(demoCounter);

    //WriteObject(GENIE_OBJ_ILED_DIGITS_H, index, frame.wordValue[1]);
    //WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, frame.wordValue[0]);
    
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_WRITE_STR, MAIN_ODOMETER_DIGITS, 0, 0, "129,000.8mi");

    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, MAIN_TACHOMETER_DIGITS, demoCounter % 8000, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_TACHOMETER_ARROWS, demoCounter % 13, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, MAIN_SPEEDOMETER_DIGITS, demoCounter % 8000, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_SPEEDOMETER_ARROWS, demoCounter % 13, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_GEAR_SELECTOR, demoCounter % 7, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_FUEL_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_OIL_PRESSURE_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_COOLANT_TEMP_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_BATTERY_VOLTAGE_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, (demoCounter % 1000) * (-1), 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_RIGHT_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_RIGHT_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_LEFT_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_LEFT_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_RIGHT_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_RIGHT_DIGITS, demoCounter % 1000, 0, "");

    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_LEFT_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_LEFT_DIGITS, demoCounter % 1000, 0, "");

  }
}

// Main loop
void loop()
{

    if (debug == true)
    {
      readAllAnalogInputs();
    }


  processSerialQueue();

  // If a demo is running, run the demo program
  if (runningDemo == true) {

    processDemoLoop();

  }
  else {

    // Read all sensor values
    readSensorValues();

    // Update all the display screens as needed
    updateDisplayScreens();

    // Update the odometer reading in EEPROM
    updateOdometer();

    // Process the CANBUS message if a new message exists
    if (!digitalRead(CAN0_INT))
    {
      processedMessages += 1;
      processCANBUSMessage();
    }
  }

}
