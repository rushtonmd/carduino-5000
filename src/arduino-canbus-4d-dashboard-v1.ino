/**********************************************************************************
   Carduino 5000

   This sketch combines CAN BUS and the 4D Systems library to read parameters
   from a Megasquirt and output the values to multiple 4D displays.

   Libraries
   https://github.com/coryjfowler/MCP_CAN_lib
   https://www.forward.com.au/pfod/ArduinoProgramming/TimingDelaysInArduino.html
   https://github.com/4dsystems/ViSi-Genie-Arduino-Library
   http://www.msextra.com/doc/pdf/Megasquirt_CAN_Broadcast.pdf


   Default Base ID: 0x5E8 (1520)

   Example:
   ID: 0x5F2 (1520 + 2 = 1522) = Barometer, MAP, MAT, and Coolant Temp

   Barometer (kPa) = (message[0] << 8) + message[1]) / 10.0;
   MAP (kPa) = (message[2] << 8) + message[3]) / 10.0;
   MAT (deg F) = (message[4] << 8) + message[5]) / 10.0;
   Coolang Temp (deg F) = (message[6] << 8) + message[7]) / 10.0;


*/

// 4D Systems library for Genie designer displays
#include <genieArduino.h>

// CAN BUS libraries
#include <mcp_can.h>
#include <SPI.h>

// Add library for UI delays
#include <millisDelay.h>

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

// Initialize the 3 separate display screens
Genie screen1;
Genie screen2;
Genie screen3;

// Variables for CAN BUS values
int tachometerValue = 0;
int speedometerValue = 0;
int fuelValue = 0;
int oilPressureValue = 0;
int batteryVoltageValue = 0;
int gearNumberValue = 0;
int odometerValue = 0;
int barometerValue = 0;
int MAPValue = 0;
int MATValue = 0;
int coolantTemperatureValue = 0;
int coolantPressureValue = 0;
int transmissionTempValue = 0;
int transmissionPessureValue = 0;

int sampleRPM = 0;
int sampleSPEED = 0;


#define RESETLINE1 22  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)
#define RESETLINE2 24  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)
#define RESETLINE3 26  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)

int message_counter = 0;
int digits = 0;

void writeUIObject(int OBJ, int objectNumber, int value) {

  screen1.WriteObject(OBJ, objectNumber, value);

  // Delay is required - with no delay the display pukes when too
  // many requests are made
  delay(5);
}

void initializeScreenLabels() {
  Serial.println("Init Labels");
  screen2.WriteStr(0, "  COOLANT");
  screen2.WriteStr(5, "    deg F");
  delay(5);
  screen3.WriteStr(0, "    TRANS");
  screen3.WriteStr(5, "    deg F");
  delay(5);

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
  screen1.Begin(Serial1);
  screen2.Begin(Serial2);
  screen3.Begin(Serial3);

  // Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.
  pinMode(RESETLINE1, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  pinMode(RESETLINE2, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  pinMode(RESETLINE3, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE1, 0);  // Reset the Display via D4
  digitalWrite(RESETLINE2, 0);  // Reset the Display via D4
  digitalWrite(RESETLINE3, 0);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE1, 1);  // unReset the Display via D4
  digitalWrite(RESETLINE2, 1);  // unReset the Display via D4
  digitalWrite(RESETLINE3, 1);  // unReset the Display via D4

  delay (3500); //let the display start up after the reset (This is important)

  initializeScreenLabels();

  // Start Delays for Dipslay Screens (in milliseconds)
  speedometerDelay.start(100); // 10 times per second
  tachometerDelay.start(75); // 13 (ish) times per second
  pressureDelay.start(250); // 4 times per second
  temperatureDelay.start(500); // 2 times per second
  fuelDelay.start(1000); // Every 1 second

  if (runningDemo == true) {
    demoDelay.start(100); // 10 times per second
  }


}

void updateDisplayScreens() {

  // **** Speedometer ****
  if (speedometerDelay.justFinished()) {
    speedometerDelay.repeat(); // start delay again without drift

    screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, speedometerValue);
    screen1.WriteObject(GENIE_OBJ_USERIMAGES, 1, ceil(speedometerValue / 170.0));
    delay(5);

    sampleSPEED = sampleSPEED + 1;
  }

  // **** Tachometer ****
  if (tachometerDelay.justFinished()) {
    tachometerDelay.repeat(); // start delay again without drift

    // Do some shit
    screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 0, tachometerValue);
    screen1.WriteObject(GENIE_OBJ_USERIMAGES, 2, ceil(tachometerValue / 750.0));
    delay(5);
  }

  // **** Pressure Gauges ****
  if (pressureDelay.justFinished()) {
    pressureDelay.repeat(); // start delay again without drift

    // Do some shit
    screen1.WriteObject(GENIE_OBJ_USERIMAGES, 0, gearNumberValue);
    screen1.WriteObject(GENIE_OBJ_USERIMAGES, 4, oilPressureValue);
  }

  // **** Temperature Gauges ****
  if (temperatureDelay.justFinished()) {
    temperatureDelay.repeat(); // start delay again without drift

    // Coolant Temperature - Main Screen
    int temp = 100 + digits % 180;
    screen1.WriteObject(GENIE_OBJ_USERIMAGES, 5, coolantTemperatureValue);
    screen2.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, coolantTemperatureValue);
    screen3.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, (temp * 12));
    digits = digits + 1;

    Serial.println(digits);

    delay(5);

    // Do some shit
    //Serial.println("TEMPERATURE!");
  }

  // **** Fuel Gauges ****
  if (fuelDelay.justFinished()) {
    fuelDelay.repeat(); // start delay again without drift

    // Do some shit
    screen1.WriteObject(GENIE_OBJ_USERIMAGES, 3, fuelValue);
    screen1.WriteObject(GENIE_OBJ_USERIMAGES, 6, batteryVoltageValue);
    screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 2, odometerValue);
    delay(5);
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
    Serial.println(barometerValue);
  }


  return 0;


}

void processDemoLoop() {

  if (demoDelay.justFinished()) {
    demoDelay.repeat(); // start delay again without drift

    // Updates 10 times per second
    
    tachometerValue = (tachometerValue + 197) % 9000;
    speedometerValue = (speedometerValue + 57) % 2000;
    fuelValue = (tachometerValue / 500) % 19;
    oilPressureValue = (tachometerValue / 500) % 19;
    batteryVoltageValue = (tachometerValue / 500) % 19;
    gearNumberValue = (tachometerValue / 500) % 7;
    odometerValue = speedometerValue;
    barometerValue = 0;
    MAPValue = 0;
    MATValue = 0;
    coolantTemperatureValue = (tachometerValue / 500) % 19;
    coolantPressureValue = 0;
    transmissionTempValue = 0;
    transmissionPessureValue = 0;
  }
}

// Main loop
void loop()
{

  updateDisplayScreens();

  //
  if (runningDemo = true) {
    processDemoLoop();
  }
  else {
    if (!digitalRead(CAN0_INT))                        // If CAN0_INT pin is low, read receive buffer
    {
      processCANBUSMessage();
    }
  }

}
