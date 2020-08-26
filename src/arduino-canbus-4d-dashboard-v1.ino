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
bool runningDemo = true;

// Variables for the CAN BUS messages
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

// PIN identification for the CAN BUS shield
#define CAN0_INT 2   // Set INT to pin 2
MCP_CAN CAN0(9);     // Set CS to pin 9

millisDelay speedometerDelay;
millisDelay tachometerDelay;
millisDelay pressureDelay;
millisDelay temperatureDelay;
millisDelay fuelDelay;
millisDelay demoDelay;
millisDelay odometerWriteDelay;
millisDelay serialQueueDelay;

// Initialize the 3 separate display screens
//Genie mainDisplayScreen;
//Genie leftDisplayScreen;
//Genie rightDisplayScreen;

// Variables for CAN BUS values
int tachometerValue = 0;
int speedometerValue = 0;
int fuelValue = 0;
int oilPressureValue = 0;
int batteryVoltageValue = 0;
int gearNumberValue = 0;
int32_t odometerValue = 782460; // from bill of sale
int barometerValue = 0;
int MAPValue = 0;
int MATValue = 0;
int coolantTemperatureValue = 0;
int coolantPressureValue = 0;
int transmissionTempValue = 0;
int transmissionPessureValue = 0;

int sampleRPM = 0;
int sampleSPEED = 0;

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


#define GENIE_OBJ_FORM                  10
#define GENIE_OBJ_USERIMAGES            27
#define GENIE_OBJ_LED_DIGITS            15
#define GENIE_OBJ_STRINGS               17
#define GENIE_OBJ_SOUND                 22
#define GENIE_OBJ_ILED_DIGITS_H         38
#define GENIE_OBJ_ILED_DIGITS_L         47
#define GENIE_OBJ_ILED_DIGITS           47
#define GENIE_WRITE_STR                 2

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


Queue<DisplayScreenSerialMessage> displayMessageSerialQueue = Queue<DisplayScreenSerialMessage>(20);

void processSerialQueue() {
  if (serialQueueDelay.justFinished()) {
    serialQueueDelay.repeat(); // start delay again without drift

    int messageCount = displayMessageSerialQueue.count();
    //Serial.println(messageCount);
    if (messageCount > 0) {

      DisplayScreenSerialMessage serialMessage = displayMessageSerialQueue.pop();

      switch (serialMessage.object) {
        case GENIE_OBJ_ILED_DIGITS:
          testWriteToDisplay(*serialMessage.serialPort, GENIE_OBJ_ILED_DIGITS, serialMessage.index, serialMessage.digitsValue);
          break;
        case GENIE_OBJ_USERIMAGES:
          testWriteToDisplay(*serialMessage.serialPort, GENIE_OBJ_USERIMAGES, serialMessage.index, serialMessage.digitsValue);
          break;
        case GENIE_WRITE_STR:
          writeStringToDisplay(*serialMessage.serialPort, serialMessage.index, serialMessage.stringsValue);
          break;
        case GENIE_OBJ_SOUND:
          testWriteToDisplay(*serialMessage.serialPort, GENIE_OBJ_SOUND, 0, 0);
          break;
      }

    }





  }
}
//
//void updateDisplayScreenDigits(Genie &displayScreen, int objectNumber, int digitsValue) {
//
//  displayScreen.WriteObject(GENIE_OBJ_ILED_DIGITS, objectNumber, digitsValue);
//  //delay(WRITE_OBJECT_DELAY);
//
//}
//
//void updateDisplayScreenStrings(Genie &displayScreen, int objectNumber, char *stringsValue) {
//
//  displayScreen.WriteStr(objectNumber, stringsValue);
//  //delay(WRITE_OBJECT_DELAY);
//
//}
//
//void updateDisplayScreenImages(Genie &displayScreen, int objectNumber, int imageIndex) {
//
//  displayScreen.WriteObject(GENIE_OBJ_USERIMAGES, objectNumber, imageIndex);
//  //delay(WRITE_OBJECT_DELAY);
//
//}

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
  Serial.println("Init Labels");
  addSerialDisplayMessageToQueue(Serial2, GENIE_WRITE_STR, SIDE_TOP_TEXT, 0, 0, "  COOLANT");
  addSerialDisplayMessageToQueue(Serial2, GENIE_WRITE_STR, SIDE_TOP_UNITS, 0, 0, "   deg F");
  
  playScreenSound();
  
  //screen2.WriteStr(0, "  COOLANT");
  //screen2.WriteStr(5, "    deg F");
  //delay(5);
  //screen3.WriteStr(0, "    TRANS");
  //screen3.WriteStr(5, "    deg F");
  //delay(5);

}

void playScreenSound() {
  addSerialDisplayMessageToQueue(Serial2, GENIE_OBJ_SOUND, 0, 0, 0, "");
  addSerialDisplayMessageToQueue(Serial3, GENIE_OBJ_SOUND, 0, 0, 0, "");
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

  // Initialize the 3 separate screens
  //mainDisplayScreen.Begin(Serial1);
  //leftDisplayScreen.Begin(Serial2);
  //rightDisplayScreen.Begin(Serial3);

  // Reset the Displays
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.
  pinMode(RESETLINE1, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  pinMode(RESETLINE2, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  pinMode(RESETLINE3, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE1, 0);  // Reset the Display via D4
  digitalWrite(RESETLINE2, 0);  // Reset the Display via D4
  digitalWrite(RESETLINE3, 0);  // Reset the Display via D4
  delay(200);
  digitalWrite(RESETLINE1, 1);  // unReset the Display via D4
  digitalWrite(RESETLINE2, 1);  // unReset the Display via D4
  digitalWrite(RESETLINE3, 1);  // unReset the Display via D4

  delay (3500); //let the display start up after the reset (This is important)


  initializeScreenLabels();


  serialQueueDelay.start(5);

  if (runningDemo == true) {
    demoDelay.start(100); // 10 times per second
  }
  else
  {

    // Start Delays for Dipslay Screens (in milliseconds)
    speedometerDelay.start(100); // 10 times per second
    tachometerDelay.start(100); // 10 times per second
    pressureDelay.start(1000); // 1 time per second
    temperatureDelay.start(1000); // 1 time per second
    fuelDelay.start(2000); // // Every 2 seconds
    odometerWriteDelay.start(2000); // Every 2 seconds

    // Read initial value of odometer from EEPROM
    odometerValue = readLongFromEEPROM(0);

    //Serial.println(odometerValue);

  }


}

void updateOdometer() {

  if (odometerWriteDelay.justFinished()) {

    // start delay again without drift
    odometerWriteDelay.repeat();

    //odometerValue = odometerValue + digits;

    // Write the new odometer value to eeprom
    writeLongIntoEEPROM(0, odometerValue);

  }

}

void updateDisplayScreens() {

  // **** Speedometer ****
  if (speedometerDelay.justFinished()) {
    speedometerDelay.repeat(); // start delay again without drift

    //screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, speedometerValue);
    //screen1.WriteObject(GENIE_OBJ_USERIMAGES, 1, ceil(speedometerValue / 170.0));
    //delay(5);

    sampleSPEED = sampleSPEED + 1;
  }

  // **** Tachometer ****
  if (tachometerDelay.justFinished()) {
    tachometerDelay.repeat(); // start delay again without drift

    // Do some shit
    //screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 0, tachometerValue);
    //screen1.WriteObject(GENIE_OBJ_USERIMAGES, 2, ceil(tachometerValue / 750.0));
    //delay(5);
  }

  // **** Pressure Gauges ****
  if (pressureDelay.justFinished()) {
    pressureDelay.repeat(); // start delay again without drift

    //Serial.println(gearNumberValue);

    // Do some shit
    //mainDisplayScreen.WriteObject(GENIE_OBJ_USERIMAGES, 0, gearNumberValue);
    //mainDisplayScreen.WriteObject(GENIE_OBJ_USERIMAGES, 4, oilPressureValue);
  }

  // **** Temperature Gauges ****
  if (temperatureDelay.justFinished()) {
    temperatureDelay.repeat(); // start delay again without drift

    // Coolant Temperature - Main Screen
    //mainDisplayScreen.WriteObject(GENIE_OBJ_USERIMAGES, 5, coolantTemperatureValue);
    //screen2.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, coolantTemperatureValue);
    //screen3.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, transmissionTempValue);

    //Serial.println(digits);

    //delay(5);

    // Do some shit
    //Serial.println("TEMPERATURE!");
  }

  // **** Fuel Gauges ****
  if (fuelDelay.justFinished()) {
    fuelDelay.repeat(); // start delay again without drift

    //int val = analogRead(0);  // read the input pin
    //Serial.println(val);          // debug value

    // Do some shit
    //screen1.WriteObject(GENIE_OBJ_USERIMAGES, 3, fuelValue);
    //screen1.WriteObject(GENIE_OBJ_USERIMAGES, 6, batteryVoltageValue);
    //screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 2, odometerValue);
    //delay(5);
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

  // 0x5F2 : Barometer, MAP, MAT, and Coolant Temp
  if (rxId == 0x5F2) {
    barometerValue = ((rxBuf[0] << 8) + rxBuf[1]) * 0.145038; // kpa
    MAPValue = ((rxBuf[2] << 8) + rxBuf[3]) * 0.145038; // kps
    MATValue = (rxBuf[4] << 8) + rxBuf[5]; // deg F
    coolantTemperatureValue = (rxBuf[6] << 8) + rxBuf[7]; // deg F
    //Serial.println(barometerValue);
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

void testWriteToDisplay(Stream &serialPort, uint16_t object, uint16_t index, uint16_t data) {

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

    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_ILED_DIGITS, MAIN_TACHOMETER_DIGITS, demoCounter % 8000, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_USERIMAGES, MAIN_TACHOMETER_ARROWS, demoCounter % 13, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_ILED_DIGITS, MAIN_SPEEDOMETER_DIGITS, demoCounter % 8000, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_USERIMAGES, MAIN_SPEEDOMETER_ARROWS, demoCounter % 13, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_USERIMAGES, MAIN_GEAR_SELECTOR, demoCounter % 7, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_USERIMAGES, MAIN_FUEL_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_USERIMAGES, MAIN_OIL_PRESSURE_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_USERIMAGES, MAIN_COOLANT_TEMP_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(Serial1, GENIE_OBJ_USERIMAGES, MAIN_BATTERY_VOLTAGE_GAUGE, demoCounter % 19, 0, "");
    addSerialDisplayMessageToQueue(Serial2, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, demoCounter % 1000, 0, "");
    addSerialDisplayMessageToQueue(Serial3, GENIE_OBJ_ILED_DIGITS, SIDE_TOP_DIGITS, demoCounter % 1000, 0, "");


  }
}

// Main loop
void loop()
{
  //int val = analogRead(0);  // read the input pin
  //Serial.println(val);          // debug value
  //Serial.println(Serial1.available() );
  //  if (Serial1.available() > 4) {
  //    Serial.println("Serial 1 NOT available");
  //    delay(1000);
  //  }


  processSerialQueue();

  // If a demo is running, run the demo program
  if (runningDemo == true) {

    processDemoLoop();

  }
  else {

    // Update all the display screens as needed
    updateDisplayScreens();

    // Update the odometer reading in EEPROM
    updateOdometer();

    // Process the CANBUS message if a new message exists
    if (!digitalRead(CAN0_INT))
    {
      processCANBUSMessage();
    }
  }

}
