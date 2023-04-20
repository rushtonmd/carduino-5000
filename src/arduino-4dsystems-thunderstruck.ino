/*************************************************************************************************
    Samurai-Duino
    @rushtonmd - 2023

    Thunderstruck-EV Master Control Unit uses OBDII formatted messages via canbus 
    to request information from the unit. This sketch adds the appropriate libraries
    and logic to:

    - Retrieve information about the batteries, amps, etc. from the Master Control Unit
    - Send canbus messages to Speedhut gauges
    - Send canbus messages to a 4d Systems screen
    - Get temperature from DS18B20 sensor (attached to the motor controller)

***************************************************************************************************/


// SPI Library for the CanBus Shield
#include <SPI.h>

// SoftwareSerial uses I/O pins to communicate with the LCD Screen
#include <SoftwareSerial.h>

// Queueing Library
#include "Queue.h"

// OneWire and DallasTemperature libraries for the temperature sensors
#include <OneWire.h>
#include <DallasTemperature.h>

// Set up a new SoftwareSerial object
SoftwareSerial displaySerial (7, 6);

// RESETLINE is the pin used to connect to the reset of the LCD display
#define RESETLINE 5  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)

// CAN_2515 Included for the CanBus shield
#define CAN_2515
// #define CAN_2518FD

// Define the Dallas Temperature sensor bus/data pin
#define ONE_WIRE_BUS A5

// Define the Dallas Temperature sensor bus/data pin
#define VOLTAGE_INPUT_PIN A0

// Add library for timed delays (pretend multi tasking)
#include <millisDelay.h>

// Set SPI CS Pin according to your hardware
#if defined(SEEED_WIO_TERMINAL) && defined(CAN_2518FD)
// For Wio Terminal w/ MCP2518FD RPi Hat：
// Channel 0 SPI_CS Pin: BCM 8
// Channel 1 SPI_CS Pin: BCM 7
// Interupt Pin: BCM25
const int SPI_CS_PIN  = BCM8;
const int CAN_INT_PIN = BCM25;
#else

// For Arduino MCP2515 Hat:
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#endif


#ifdef CAN_2518FD
#include "mcp2518fd_can.h"
mcp2518fd CAN(SPI_CS_PIN); // Set CS pin
#endif

#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif

#define debug false


/***** Thunderstruck BMS/MCU OBD2 Definitions *****/

#define CAN_ID_MODE           0x22
#define PID_PREFIX            0xdd
#define PID_PACK_VOLTAGE      0x83
#define PID_CURRENT           0x84
#define PID_STATE_OF_CHARGE   0x85
#define PID_FAULTS            0x81
#define PID_FAULT_HARDWARE    0X40
#define PID_FAULT_CCENSUS     0X20
#define PID_FAULT_TCENSUS     0X10
#define PID_FAULT_HVC         0X08
#define PID_FAULT_LVC         0X04
#define PID_FAULT_HIGHTEMP    0X02
#define PID_FAULT_LOWTEMP     0X01

/***** Definitions for the 4D Systems LCD Screen *****/

#define GENIE_OBJ_FORM                  10
#define GENIE_OBJ_USERIMAGES            27
#define GENIE_OBJ_LED                   14
#define GENIE_OBJ_LED_DIGITS            15
#define GENIE_OBJ_STRINGS               17
#define GENIE_OBJ_SOUND                 22
#define GENIE_OBJ_ILED_DIGITS_H         38
#define GENIE_OBJ_ILED_DIGITS_L         47
#define GENIE_OBJ_ILED_DIGITS           47
#define GENIE_WRITE_STR                 2
#define GENIE_OBJ_IANGULAR_METER        39
#define GENIE_OBJ_IRULER                49
#define GENIE_OBJ_IGAUGE                40
#define GENIE_OBJ_ILED                  45
#define GENIE_OBJ_TIMER                 23
#define GENIE_WRITE_CONTRAST            4

// PID of this arduino / canbus - used for OBD2 requests
#define CAN_ID_PID          0x7DF

// All of the OBD2 PIDs are 2 bytes, so we need to send 3 total bytes
#define CAN_SEND_BYTES      0x03

#define CURRENT_DELAY  100
#define PACK_VOLTAGE_DELAY  2000
#define STATE_OF_CHARGE_DELAY  2000
#define BMS_FAULTS_DELAY  1000
#define VIDEO_SPINNER_DELAY  500
#define TEMPERATURE_SENSOR_DELAY 1000

// Numbered Values of Screen Elements on LCD
#define REGEN_GAUGE  4
#define CURRENT_GAUGE  2
#define BATTERY_TEMP_GAUGE  1
#define BATTERY_VOLTS_GAUGE  0
#define MOTOR_TEMP_GAUGE  4
#define BMS_LED  0
#define MOTOR_LED  1
#define VIDEO_TIMER  0

// Delay timers - These are used for setting delays for pretend multi-tasking
millisDelay requestCurrentDelay;
millisDelay requestPackVoltageDelay;
millisDelay requestStateOfChargeDelay;
millisDelay requestBMSFaultsDelay;
millisDelay serialQueueDelay;
millisDelay videoSpinnerDelay;
millisDelay temperatureSensorDelay;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature temperatureSensors(&oneWire);

// Number of devices found on the bus
uint8_t numberOfTemperatureSensors;

unsigned char PID_INPUT = 0x83;
unsigned char getPid    = 1;

// All Graph Values are 0-100%
int current = 0;
int regen = 0;
int voltage = 0;
int batteryTemperature = 0;
int bmsFault = 0;
int motorFault = 0;
int motorTemperature = 0;
int controllerTemperature = 0;
int tractionVoltage = 0;
int highBatteryTemperature = 0;
int lowBatteryTemperature = 0;

// Struct object for adding LCD screen information to the queue
typedef struct {
  Stream *serialPort;
  uint16_t object;
  uint16_t index;
  uint16_t digitsValue;
  uint16_t imageIndex;
  char *stringsValue;

} DisplayScreenSerialMessage;

// Initialize global queue for thd LCD screen
Queue<DisplayScreenSerialMessage> displayMessageSerialQueue = Queue<DisplayScreenSerialMessage>(50);

void set_mask_filt() {
    /*
        set mask, set both the mask to 0x3ff
    */
    CAN.init_Mask(0, 0, 0x7FC);
    CAN.init_Mask(1, 0, 0x7FC);

    /*
        set filter, we can receive id from 0x04 ~ 0x09
    */
    CAN.init_Filt(0, 0, 0x7E8);
    CAN.init_Filt(1, 0, 0x7E8);

    CAN.init_Filt(2, 0, 0x7E8);
    CAN.init_Filt(3, 0, 0x7E8);
    CAN.init_Filt(4, 0, 0x7E8);
    CAN.init_Filt(5, 0, 0x7E8);
}

// addSerialDisplayMessageToQueue
//
// The purpose of this function is to add messages to the LCD display queue. Any change
// needed to the screen is added to this queue and processed at a specific speed as 
// to not blow up the screen.
void addSerialDisplayMessageToQueue(Stream &serialPort, uint16_t object, uint16_t index, uint16_t digitsValue, uint16_t imageIndex, char *stringsValue) {

  // Instantiate a new serial message
  DisplayScreenSerialMessage serialMessage;
  serialMessage.serialPort = &serialPort;
  serialMessage.object = object;
  serialMessage.index = index;
  serialMessage.digitsValue = digitsValue;
  serialMessage.imageIndex = imageIndex;  
  serialMessage.stringsValue = stringsValue;

  // Add the formatted message to the queue
  displayMessageSerialQueue.push(serialMessage);
}

void processSerialQueue() {
  if (serialQueueDelay.justFinished()) {
    serialQueueDelay.repeat(); // start delay again without drift

    // Get the number of messages currently in the queue
    int messageCount = displayMessageSerialQueue.count();
    
    if (debug) {
      Serial.println("Processed Messages: ");
      Serial.println(messageCount);
    }

    if (messageCount > 0) {

      // Get the top message off the queue
      DisplayScreenSerialMessage serialMessage = displayMessageSerialQueue.pop();

      // Send the message to the display
      writeObjectToDisplay(*serialMessage.serialPort, serialMessage.object, serialMessage.index, serialMessage.digitsValue);  

    }

  }
}

// sendPid
// Sends the request to the BMS for a particular OBDII PID
void sendPid(unsigned char __pid) {

    // Build the request into a char array
    unsigned char tmp[8] = {CAN_SEND_BYTES, CAN_ID_MODE, PID_PREFIX, __pid, 0, 0, 0, 0};

    // Send the request over CAN BUS
    CAN.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}

// sendMessageToGauges
//
// Sends a message to update the Speedhut Gauges(s) over CAN BUS
void sendMessageToGauges(){
  
  // The only gauge currently is the controller temperature
  
  // Canbus is celcius, gauge values are F
  unsigned char tempVal = controllerTemperature & 0xff;
  unsigned char stmp[8] = {0, 0, 0, 0, tempVal, 0, 0, 0};
  CAN.sendMsgBuf(0x126, 0, 8, stmp);

}

void setup() {

  
  //if (debug) {
    // Serial port monitor only used in debugging
    SERIAL_PORT_MONITOR.begin(115200);  // Serial0 @ 200000 (200K) Baud

    // Wait until the serial is ready
    while(!SERIAL_PORT_MONITOR){};
  //}

  // Start the software serial for the LCD display
  displaySerial.begin(115200);



  // Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.  
  pinMode(RESETLINE, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE, 0);  // Reset the Display via RESETLINE
  delay(200);
  digitalWrite(RESETLINE, 1);  // unReset the Display via RESETLINE

  // Let the display start up after the reset (This is important)
  // Increase to 4500 or 5000 if you have sync problems as your project gets larger. Can depent on microSD init speed.
  delay (3000); 

  // Set contrast (brightne)
  writeContrast(displaySerial, 8);

    // Set the queue delay for processing serial communications to the screens
  // 5 milliseconds seems to be the minimum to not have the screens puke
  // So we'll set it at 10ms which should be ok
  serialQueueDelay.start(30);
    


  while (CAN_OK != CAN.begin(CAN_500KBPS)) {             // init can bus : baudrate = 500k
    if (debug){
      SERIAL_PORT_MONITOR.println("CAN init fail, retry...");        
    }

    delay(100);
  }

  if (debug) {
    SERIAL_PORT_MONITOR.println("CAN init ok!");
  }
  
  // Set mask filter for CAN BUS
  //set_mask_filt();

  // Start gathering data from temperature sensors
  temperatureSensors.begin();

  // Get the number of devices on the bus
  numberOfTemperatureSensors = temperatureSensors.getDeviceCount();

  if (debug){
    SERIAL_PORT_MONITOR.print("Sensor Count: ");
    SERIAL_PORT_MONITOR.println(numberOfTemperatureSensors);
  }

  // Start timers
  requestCurrentDelay.start(CURRENT_DELAY); 
  videoSpinnerDelay.start(VIDEO_SPINNER_DELAY);
  temperatureSensorDelay.start(TEMPERATURE_SENSOR_DELAY);
  requestPackVoltageDelay.start(PACK_VOLTAGE_DELAY);

}

/////////////////////// WriteContrast //////////////////////
//
// Alter the display contrast (backlight)
//
// Parms:   uint8_t value: The required contrast setting, only
//      values from 0 to 15 are valid. 0 or 1 for most displays
//      and 0 to 15 for the uLCD-43, uLCD-70, uLCD-35, uLCD-220RD
//
void writeContrast (Stream &serialPort, uint16_t value) {
  Stream* deviceSerial = &serialPort;
    unsigned int checksum ;
    deviceSerial->write(GENIE_WRITE_CONTRAST) ;
    checksum  = GENIE_WRITE_CONTRAST ;
    deviceSerial->write(value) ;
    checksum ^= value ;
    deviceSerial->write(checksum) ;
}

void updateDisplayScreen(){
  
    // **** Video Spinner ****
  if (videoSpinnerDelay.justFinished()) {
    videoSpinnerDelay.repeat(); // start delay again without drift

    // Update the screen with the current CURRENT/AMPS
    addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_TIMER, VIDEO_TIMER, 1, 0, "");

  }

  // **** Amps / Current and Regen ****
  if (requestCurrentDelay.justFinished()) {
    requestCurrentDelay.repeat(); // start delay again without drift

    // Request the new current from the BMS - this will happen async and the value 
    // will be updated when the current value returns.
    sendPid(PID_CURRENT);
    sendPid(PID_FAULTS);

    int displayCurrent = max(0,current) * 0.125;
    int displayRegen = min(100+(current * 0.1666), 100); 

    if (debug) {
      SERIAL_PORT_MONITOR.print("Current: ");
      SERIAL_PORT_MONITOR.println(displayCurrent);
      SERIAL_PORT_MONITOR.print("Regen");
      SERIAL_PORT_MONITOR.println(displayRegen);
    }

    // Update the screen with the current CURRENT/AMPS and Regen
    addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, CURRENT_GAUGE, displayCurrent, 0, "");
    addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, REGEN_GAUGE, displayRegen, 0, "");
  
    // FAULTS
    addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_LED, BMS_LED, bmsFault, 0, "");
 

  }

  // **** 12v Battery Voltage ****
  if (requestPackVoltageDelay.justFinished()) {
    requestPackVoltageDelay.repeat(); // start delay again without drift

    // Request the new Voltage from the BMS - this will happen async and the value 
    // will be updated when the current value returns.
    sendPid(PID_PACK_VOLTAGE);

    // Read the input voltage     
    int voltageSensorValue = analogRead(VOLTAGE_INPUT_PIN);

    // Votlage input uses a 4700/1000 voltage divider
    // anything above 2.8v should be considered 100%
    // anything below 1.5v should be considered 0%
    // range of reading is from 0-5v or 0-1023
    // 2.8 * 1023 / 5 = 577 <- max value
    // 1.5 * 1023 / 5 = 309 <- min value
    // voltage = (voltageSensorValue - 443) / 2.68 
    //671 = 12.1
    //733 = 13.8

    //voltage = (voltageSensorValue - 443) / 4.25;
    voltage = (voltageSensorValue * 0.342) - 180;

    voltage = min(100, voltage);
    voltage = max(0, voltage);

    // TEMPORARY
    // if (voltageSensorValue > 400) {
    //   voltage = 65;
    // } else {
    //   voltage = 0;
    // }
    if (debug) {
      SERIAL_PORT_MONITOR.print("******************* Voltage: ");
      SERIAL_PORT_MONITOR.println(voltageSensorValue);
      SERIAL_PORT_MONITOR.println(voltage);
    } 

    // Update the screen with the current CURRENT/AMPS and Regen
    addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, BATTERY_VOLTS_GAUGE, voltage, 0, "");

    // ******* TEMPORARY **********
    // For some reason the BMS isn't sending battery temperature so we'll use that gauge
    // for the traction battery voltage
    // Max Charge is 122
    // Min Charge is 95
    //int tractionVoltagePercentage = ((tractionVoltage - 95.0) / (122.0 - 95.0) ) * 100.0;
    
    //tractionVoltagePercentage = min(tractionVoltagePercentage, 100);
    //tractionVoltagePercentage = max(tractionVoltagePercentage, 0);
    
    //addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, BATTERY_TEMP_GAUGE, tractionVoltagePercentage, 0, "");

  }


  // **** Battery Temp (from BMS) ****
  // **** Motor Temp (from temp sensor) ****

  if (temperatureSensorDelay.justFinished()) {
    temperatureSensorDelay.repeat(); // start delay again without drift

    // Request the new current from the BMS - this will happen async and the value 
    // will be updated when the current value returns.
    //sendPid(PID_CURRENT);
    updateTemperatureSensors();

    // Update the battery temperature
    // CAN BUS NOT SENDING THIS SO NO IDEA 
    //addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, 1, batteryTemperature, 0, "");

    // Update the motor temperature
    addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, 3, motorTemperature, 0, "");

    sendMessageToGauges();

    // Battery Temperature Gauge
    if ((lowBatteryTemperature == 0) && (highBatteryTemperature == 0)) {
      addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, BATTERY_TEMP_GAUGE, 50, 0, "");
    } else if(highBatteryTemperature == 1) {
      addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, BATTERY_TEMP_GAUGE, 100, 0, "");      
    } else {
      addSerialDisplayMessageToQueue(displaySerial, GENIE_OBJ_IGAUGE, BATTERY_TEMP_GAUGE, 10, 0, "");
    }

  }
  // **** Motor Temp ****
  // **** BMS Faults ****


}

// updateTemperatureSensors
//
// This function is called on every loop, but the inner logic is evaluated 
// only when the temperatureSensorDelay has elapsed. 
// The purpose of this function is to read the sensor data from the 
// two Dallas temperature sensors, and update the global temperature
// variables. 
void updateTemperatureSensors() {


    // Request temperature data from all devices on the bus
    temperatureSensors.requestTemperatures();

    float temp = 0.0f;

    // Read the first temperature sensor
    // This one is read in Celsius because the physical
    // gauge is in Celsius 
    if (numberOfTemperatureSensors > 0) {
      temp = temperatureSensors.getTempCByIndex(0);
      controllerTemperature = round(temp);
    }

    // Read the second temperature sensor
    // This one is read in Fahrenheit and divided
    // by 2 to adjust for the units of the LCD gauge
    if (numberOfTemperatureSensors > 1) {
      temp = temperatureSensors.getTempFByIndex(1);
      motorTemperature = round(temp / 2.0);
    }

    if (debug){
      SERIAL_PORT_MONITOR.print("Motor Temp F: ");
      SERIAL_PORT_MONITOR.println(motorTemperature);
      SERIAL_PORT_MONITOR.print("Controller Temp C: ");
      SERIAL_PORT_MONITOR.println(controllerTemperature);

    }

    // Read and print temperatures from each device
    // for (uint8_t i = 0; i < numberOfTemperatureSensors; i++) {
    //   // Get the temperature of the current device
    //   float tempC = temperatureSensors.getTempCByIndex(i);

    //   // Print device number and its temperature
    //   Serial.print("Device ");
    //   Serial.print(i, DEC);
    //   Serial.print(": Temperature: ");
    //   Serial.print(tempC);
    //   Serial.println(" °C");
    // }

  

} 

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}


// Main loop for the entire program 
// This program loop runs as fast as it can and never stops
// Don't stop get it get it
void loop() {

  // Every loop we need to process CAN BUS messages
  taskCanRecv();

  if (debug) {
    taskDbg();
  }
  
  // Update the values that need to be sent to the LCD screen
  updateDisplayScreen();

  // Processes the LCD display queue
  processSerialQueue();


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

}

int processOBD2Response() {

  unsigned char len = 0;
  unsigned char buf[8];
  unsigned char pid;
  unsigned long id;


  // Read data: len = data length, buf = data byte(s)
  while ( CAN.readMsgBufID(&id, &len, buf) == CAN_OK) {
  
    if (debug){
      SERIAL_PORT_MONITOR.println("\r\n------------------------------------------------------------------");
      SERIAL_PORT_MONITOR.print("Get Data From id: 0x");
      SERIAL_PORT_MONITOR.println(CAN.getCanId(), HEX);
      SERIAL_PORT_MONITOR.println(id, HEX);
      for (int i = 0; i < len; i++) { // print the data
          SERIAL_PORT_MONITOR.print("0x");
          SERIAL_PORT_MONITOR.print(buf[i], HEX);
          SERIAL_PORT_MONITOR.print("\t");
      }
      SERIAL_PORT_MONITOR.println();
    }


    if(id == 0x7E8) {
      pid = buf[3];
    } else
    {
      pid = (int)id;
    }

    if (debug) {
      SERIAL_PORT_MONITOR.print("Pid: ");
      SERIAL_PORT_MONITOR.println(pid, HEX);
    }

    // 0x357 is thermister
    // 0-255 Degrees C
    if (pid == 0x357) {
      SERIAL_PORT_MONITOR.print("HERE: ");
      batteryTemperature = (buf[2] * 1.8) + 32; // Battery temperature converted to F
    }

    // 0x81 Faults
    if (pid == 0x81) {
      
      if (debug) { 
        SERIAL_PORT_MONITOR.print("Faults: ");
        SERIAL_PORT_MONITOR.println((int16_t)buf[4]);
      }
      
      //if (buf[4] & PID_FAULT_HARDWARE) {
      if ((int16_t)buf[4] > 0) {
        bmsFault = 1;
      }
      else {
        bmsFault = 0;
      }

      if ((int16_t)buf[4] & 0x01) {
        lowBatteryTemperature = 1;
      }
      else {
        lowBatteryTemperature = 0;
      }

      if ((int16_t)buf[4] & 0x02) {
        highBatteryTemperature = 1;
      }
      else {
        highBatteryTemperature = 0;
      }


      if (debug) {
        SERIAL_PORT_MONITOR.print(highBatteryTemperature);
        SERIAL_PORT_MONITOR.print(":");
        SERIAL_PORT_MONITOR.println(lowBatteryTemperature);
      }
      
    }

    // 0x83 Pack Voltage - 
    if (pid == 0x83) {
      
      tractionVoltage = ((int16_t)(buf[5] << 8) + buf[4]) / 10.0;
      if (debug) {
        SERIAL_PORT_MONITOR.print("Voltage: ");
        SERIAL_PORT_MONITOR.println(buf[4]);
      }
    }

    // 0x84 Pack Current
    if (pid == 0x84) {

      current = ((int16_t)(buf[5] << 8) + buf[4]) / 10.0;
      if (debug) {
        SERIAL_PORT_MONITOR.print("Current: ");
        SERIAL_PORT_MONITOR.println(current);
      }
    }
  }

}

void taskCanRecv() {
    unsigned char len = 0;
    unsigned char buf[8];
    unsigned long id;

    if (CAN_MSGAVAIL == CAN.checkReceive()) {                // check if get data
        processOBD2Response();

    }
}

void taskDbg() {
    while (SERIAL_PORT_MONITOR.available()) {
        char c = SERIAL_PORT_MONITOR.read();

        if (c >= '0' && c <= '9') {
            PID_INPUT *= 0x10;
            PID_INPUT += c - '0';

        } else if (c >= 'A' && c <= 'F') {
            PID_INPUT *= 0x10;
            PID_INPUT += 10 + c - 'A';
        } else if (c >= 'a' && c <= 'f') {
            PID_INPUT *= 0x10;
            PID_INPUT += 10 + c - 'a';
        } else if (c == '\n') { // END
            getPid = 1;
        }
    }
}
// END FILE
