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

// Software Serial
//#include <SoftwareSerial.h>

// Send Only Software Serial
#include <SendOnlySoftwareSerial.h>

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

// Reset Trigger Function
void(* resetFunc) (void) = 0;

// PIN identification for the CAN BUS shield
#define CAN0_INT 2   // Set INT to pin 2
MCP_CAN CAN0(10);     // Set CS to pin 10

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
float odometerValue = 78246.000f; // initial mileage from bill of sale
float currentTrip = 0.000f;
float odometerDelta = 0.00f;
int barometerValue = 0;
int MAPValue = 0;
int MATValue = 0;
int coolantTemperatureValue = 0;
int coolantPressureValue = 0;
int transmissionTemperatureValue = 0;
int transmissionPressureValue = 0;
int throttlePositionValue = 0;
int boostValue = 0;
int airFuelRatio = 0;
int gearAnalogReadThreshold = 300;
int currentFuelValue = 0;
int maxFuelValueDelta = 1;
bool writeFirstOdometer = true;

// Define pins to reset the displays
// TODO: Repurpose these inputs for the shifter selector
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
#define MAIN_ODOMETER_DIGITS         1
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
//#define LEFT_DISPLAY_SCREEN Serial2
//#define RIGHT_DISPLAY_SCREEN Serial3
//SoftwareSerial LEFT_DISPLAY_SCREEN(2, 3); // RX, TX
//SoftwareSerial RIGHT_DISPLAY_SCREEN(4, 5); // RX, TX

SendOnlySoftwareSerial LEFT_DISPLAY_SCREEN (9);  // Tx pin
SendOnlySoftwareSerial RIGHT_DISPLAY_SCREEN (8);  // Tx pin


#define FUEL_SENDER_INPUT A7
#define COOLANT_PRESSURE_INPUT A10
#define TRANSMISSION_PRESSURE_INPUT A1

#define GEAR_P_INPUT A3
#define GEAR_R_INPUT A3 //A13
#define GEAR_N_INPUT A5
#define GEAR_D_INPUT A3 //A12
#define GEAR_2_INPUT A11
#define GEAR_L_INPUT A4

#define GEAR_INPUT_1 21
#define GEAR_INPUT_2 19
#define GEAR_INPUT_3 7


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
//        case GENIE_OBJ_SOUND_VOLUME:
//          writeObjectToDisplay(*serialMessage.serialPort, GENIE_OBJ_SOUND, 1, 25);
//          break;
//        case GENIE_OBJ_SOUND:
//          writeObjectToDisplay(*serialMessage.serialPort, GENIE_OBJ_SOUND, 0, 0);
//          break;
      }

    }





  }
}

void writeFloatIntoEEPROM(int address, float number)
{
  //float f = 78246.00f;
  EEPROM.put(address, number);
  
//  EEPROM.write(address, (number >> 24) & 0xFF);
//  EEPROM.write(address + 1, (number >> 16) & 0xFF);
//  EEPROM.write(address + 2, (number >> 8) & 0xFF);
//  EEPROM.write(address + 3, number & 0xFF);
}

float readFloatFromEEPROM(int address)
{
  float f = 0.00f;
  EEPROM.get( address, f );
  return f;
//  
//  return ((long)EEPROM.read(address) << 24) +
//         ((long)EEPROM.read(address + 1) << 16) +
//         ((long)EEPROM.read(address + 2) << 8) +
//         (long)EEPROM.read(address + 3);
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
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_RIGHT_TEXT, 0, 0, "    TRIP");
  addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_TOP_RIGHT_UNITS, 0, 0, "        mi");

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
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_LEFT_TEXT, 0, 0, " BOOST / VAC");
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_LEFT_UNITS, 0, 0, "    psi");

  // Transmission Pressure
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_RIGHT_TEXT, 0, 0, "       AFR");
  addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_WRITE_STR, SIDE_BOTTOM_RIGHT_UNITS, 0, 0, "        ");


}

// Play the default sound on both screens
// NOTE: This take a ton of memory and causes issues - probably don't use it
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
  if (debug) {
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
}


// Setup function

void setup()
{

  //enables VBUS pad - needed to detect usb power
  USBCON|=(1<<OTGPADE); 
  
  // Initialize the debug Serial if needed
  if (debug) {
    Serial.begin(115200);
  }
  delay(4000);// Give screens a few moments to turn on otherwise they puke

  // Initialize the Serial pins for the displays. 115200 should be sufficient.
  MAIN_DISPLAY_SCREEN.begin(115200);  // Serial1 @ 115200 Baud
  LEFT_DISPLAY_SCREEN.begin(115200);  // Serial2 @ 115200 Baud
  RIGHT_DISPLAY_SCREEN.begin(115200);  // Serial3 @ 115200 Baud  

  // Initialize MCP2515 running a 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);

  // Configuring pin for /INT input
  pinMode(CAN0_INT, INPUT);

  // Set Pin Modes
  pinMode(GEAR_INPUT_1, INPUT);
  pinMode(GEAR_INPUT_2, INPUT);
  pinMode(GEAR_INPUT_3, INPUT);

  // Read initial value of odometer from EEPROM
  odometerValue = readFloatFromEEPROM(0);
    
  // Set all the default labels for the screens
  initializeScreenLabels();

  // Set the queue delay for processing serial communications to the screens
  // 5 milliseconds seems to be the minimum to not have the screens puke
  // The super slow parts are the string/text fields. The odometer field totally blows up when it's updated.
  serialQueueDelay.start(10);

  if (runningDemo == true) {
    demoDelay.start(1000); // 10 times per second
  }
  else
  {

    // Start Delays for Dipslay Screens (in milliseconds)
    speedometerDelay.start(300); // 3.333 times per second
    tachometerDelay.start(100); // 10 times per second
    pressureDelay.start(2000); // 1 time per second
    temperatureDelay.start(2000); // 1 time per second
    fuelDelay.start(2000); // // Every 2 seconds
    odometerWriteDelay.start(3000); // Every 3 seconds
    readAnalogSensorValuesDelay.start(500); // Two tinmes per second
    readDigitalSensorValuesDelay.start(500); // Two times per second - used for detecting the gears

    // Gear and Odometer these are updated so infrequently, there's an initial gap between when the screens
    // turn on and when these values are updated. We'll manually set the first values so that 
    // the screens are updated immediately. 
  
    // Set the current Gear (P/R/N/D/2/L)
    // FOR SOME REASON THIS FUCKING BREAKS EVERYTHING SO IT'S COMMENTED OUT
    //readSensorValues(true);
    
    // Update the current odometer mileage 
    //updateOdometer(true);

  }


}

void readSensorValues(bool forced) {

//  // **** Read Analog Sensor Values ****
  if (readAnalogSensorValuesDelay.justFinished()) {
    readAnalogSensorValuesDelay.repeat(); // start delay again without drift

    // FUEL SENDER
    // Read the input pin
    // Value is 0-1023, 0-5v
    int senderCurrentValue = analogRead(FUEL_SENDER_INPUT);  
    
    // If the currentFuelValue hasn't been initialized, set it to the first read value
    if (currentFuelValue == 0) currentFuelValue = senderCurrentValue;

    // Here we reduce the effect of the new reading by giving the current
    // fuel level value a multiplier. 
    int newFuelValue = ((4.0 * currentFuelValue) + senderCurrentValue) / 5.0;

    // FUEL LEVEL 
    // FULL: 0.2v = 40 value
    // EMPTY: 1.2v =  245 value
    int maxFuel = 245;
    int minFuel = 40;

    // Ensure value is not out of bounds
    if (newFuelValue > maxFuel) newFuelValue = maxFuel; // Tank Empty 
    if (newFuelValue < minFuel) newFuelValue = minFuel; // Tank Full

    currentFuelValue = newFuelValue;

    
    if (debug) {
      Serial.print("senderCurrentValue: ");
      Serial.println(senderCurrentValue);
      Serial.print("currentFuelValue: ");
      Serial.println(currentFuelValue);
    }

  }


  if (readDigitalSensorValuesDelay.justFinished() || forced) {
    readDigitalSensorValuesDelay.repeat(); // start delay again without drift
    // Transmission Gear

    int gearSelectorInput = (digitalRead(GEAR_INPUT_1)<<2) + (digitalRead(GEAR_INPUT_2)<<1) + digitalRead(GEAR_INPUT_3);
    
    if (gearSelectorInput == 4) gearNumberValue = 1;
    else if (gearSelectorInput  == 6) gearNumberValue = 2;
    else if (gearSelectorInput  == 0) gearNumberValue = 3;
    else if (gearSelectorInput  == 1) gearNumberValue = 4;
    else if (gearSelectorInput  == 3) gearNumberValue = 5;
    else if (gearSelectorInput  == 2) gearNumberValue = 6;

    // Update the Gear
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_GEAR_SELECTOR, gearNumberValue, 0, "");

  }

}

void updateOdometer(bool forced) {

  if (odometerWriteDelay.justFinished() || forced) {

    // stop and start again
    odometerWriteDelay.stop();
    
    // Update currentTrip
    // speedometerValue = miles per hour
    // time = 10 seconds
    // 1 mile per hour = 1.46 ft per second 
    odometerDelta = speedometerValue / 10.0 * 10.0 / 60.0 / 60.0;
    
    currentTrip = currentTrip + odometerDelta;
   
    odometerValue = odometerValue + odometerDelta;

    // Trip Display
    coolantPressureValue = currentTrip*10.0;

    // Write the new odometer value to eeprom
    writeFloatIntoEEPROM(0, odometerValue);

    //Serial.println("HERE");

    if (writeFirstOdometer) {
    
      // 6 chars is enough for 999,9999 miles
      char cstr[10] = "";
      //String odometerString = String(odometerValue/10) + " mi";
      //roundf(val * 100) / 100;
      long odoIntValue = (long)(odometerValue);
      String odometerString = String(odoIntValue) + " mi";
      
      odometerString.toCharArray(cstr, 10);
      
      //itoa(round((odometerValue + currentTrip)/10), cstr, 6);
      
      char *odometerCharacters = cstr;

      addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_WRITE_STR, MAIN_ODOMETER_DIGITS, 0, 0, odometerCharacters);
      
      if (debug) {
      Serial.println("Odometer Value: ****************************");      
      Serial.println(odometerString);
      Serial.println(odometerCharacters);
      Serial.println(odometerValue);
      Serial.println(odoIntValue);
      }
      
      writeFirstOdometer = false;
    }



    // Update Every Minute - it's ok if this floats around a bit
    odometerWriteDelay.start(10000);

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
    
     addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, MAIN_TACHOMETER_DIGITS, tachometerValue, 0, "");
     addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_TACHOMETER_ARROWS, min(12, ceil(tachometerValue / 550.0)), 0, "");
  }

  // **** Pressure Gauges ****
  if (pressureDelay.justFinished()) {
    pressureDelay.repeat(); // start delay again without drift

    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, batteryVoltageValue, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_RIGHT_DIGITS, barometerValue, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_LEFT_TEXT, MAPValue, 0, "");
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_RIGHT_DIGITS, airFuelRatio, 0, "");

    //trans
    //coolantPressureValue
    //transmissionPressureValue
    addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_LEFT_DIGITS, MAPValue - barometerValue, 0, "");
    addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_RIGHT_DIGITS, coolantPressureValue, 0, "");

   addSerialDisplayMessageToQueue(RIGHT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_BOTTOM_RIGHT_DIGITS, fuelPressureValue, 0, "");
   addSerialDisplayMessageToQueue(LEFT_DISPLAY_SCREEN, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, oilPressureValue, 0, "");



  // Oil pressure is 0-100psi on the gauge
  int oilPressureIndex = 0;
  if (oilPressureValue < 0) oilPressureIndex = 0;
  else if (oilPressureValue > 1000) oilPressureIndex = 18;
  else {
    oilPressureIndex = oilPressureValue / 55.555;
  }

    if (debug) {
      Serial.print("Oil Image: ");
      Serial.println(oilPressureIndex);
    }

   addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_OIL_PRESSURE_GAUGE, oilPressureIndex, 0, "");

     int batteryVoltageIndex = 0;
    //coolantTemperatureValue = 2760;
    if (batteryVoltageValue <= 80) batteryVoltageIndex = 0;
    
    else if (batteryVoltageValue >= 160) batteryVoltageIndex = 18;
    
    else {
      batteryVoltageIndex = ((batteryVoltageValue - 80) / 80.0) * 18;

    }

    // OK so this is dumb, the image on the screen is currupt for 12v (the middle)
    if (batteryVoltageIndex == 9) batteryVoltageIndex = 10;

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

    // Get the current sender
    int val = currentFuelValue;

    // FUEL LEVEL 
    // FULL: 0.2v = 40 value
    // EMPTY: 1.2v =  245 value

    // NEW SENDER DATA
    // FULL: ...
    // EMPTY: 209 value

    int maxFuel = 209; 
    int minFuel = 40; 

    // Calculate the fuel image number (0-18)
    // EMPTY: (245 - 245) / 11.5 = 0
    // FULL: (245 - 40) / 11.4 = 17.9
    int fuelImageNumber = round((maxFuel - val)/11.4); 

    // Images are only 0 - 18
    // These two checks will ensure the image isn't out of range
    // If the image is greater than 18, set it to 18
    if (fuelImageNumber > 18) fuelImageNumber = 18;
    // If the image is less than 0, set it to 0
    if (fuelImageNumber < 0) fuelImageNumber = 0;
   
    addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_OBJ_USERIMAGES, MAIN_FUEL_GAUGE, fuelImageNumber, 0, "");

    if (debug) {
      Serial.print("Val: ");
      Serial.println(val);          // debug value
      Serial.println(analogRead(FUEL_SENDER_INPUT));
      Serial.print("Fuel Image: ");
      Serial.println(fuelImageNumber);
      Serial.print("Current: ");
      Serial.println(currentFuelValue);
      //Serial.println(maxFuel);
      //Serial.println(minFuel);
    }


   
  }

  if (debug) {
      Serial.println("CAN Message Processed");
      Serial.println(processedMessages);
  }
   processedMessages = 0;

}

int processCANBUSMessage() {

  // Read data: len = data length, buf = data byte(s)
  CAN0.readMsgBuf(&rxId, &len, rxBuf);

  // If the message is an extendedID, return without processing
  if ((rxId & 0x80000000) == 0x80000000)  {
    return -1;
  }

  if (debug) {
    sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
    Serial.println(msgString);
  }
  
  // 0x5F0 : Seconds, PW1, PW2, RPM
  if (rxId == 0x5F0) {
    tachometerValue = (rxBuf[6] << 8) + rxBuf[7]; // RPM
  }

  // 0x61A : VSS1, VSS2, VSS3, VSS4
  if (rxId == 0x61A) {
    speedometerValue = ((rxBuf[0] << 8) + rxBuf[1]) * 2.23694; // Speed (convert meters/sec to miles/hour
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
    //Serial.println(throttlePositionValue);
    //Serial.println(batteryVoltageValue);
  }
 

  // 0x5FD : Sensor 1 (Oil Pressure), 2 (Fuel Pressure), 3, and 4
  if (rxId == 0x5FD) {
    oilPressureValue = ((rxBuf[0] << 8) + rxBuf[1]); // psi
    fuelPressureValue = ((rxBuf[2] << 8) + rxBuf[3]); //  psi   
  }

  // 0x60F : AFR1, 2, 3, ect. - I believe these are only 1 byte
  if (rxId == 0x60F) {
    airFuelRatio = rxBuf[0]; // air fuel ratio
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

    Serial.println(demoCounter);

    //WriteObject(GENIE_OBJ_ILED_DIGITS_H, index, frame.wordValue[1]);
    //WriteObject(GENIE_OBJ_ILED_DIGITS_L, index, frame.wordValue[0]);
    
    //addSerialDisplayMessageToQueue(MAIN_DISPLAY_SCREEN, GENIE_WRITE_STR, MAIN_ODOMETER_DIGITS, 0, 0, "129,000.8mi");

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

  // Detect if usb is connected
  if(!(USBSTA&(1<<VBUS))){ 
    delay(1000);
    resetFunc();
  }

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
    readSensorValues(false);

    // Update all the display screens as needsed
    updateDisplayScreens();

    // Update the odometer reading in EEPROM
   updateOdometer(false);

    // Process the CANBUS message if a new message exists
    if (!digitalRead(CAN0_INT))
    {
      //Serial.println("HERE");
      processedMessages += 1;
      processCANBUSMessage();
    }
  }

}
