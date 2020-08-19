/**********************************************************************************
 * Write String Test - Demo application
 * This demo simply illustrates the various ways you can now send data to a String 
 * object in ViSi-Genie, from an Arduino.
 * 
 * Demo uses Hardware Serial0 to communicate with the 4D Systems display module.
 * Simply create a Workshop4 Genie application for your 4D Systems display module, 
 * and place a single 'Strings' object on the display, and download it to your module.
 * It will have the ID Strings0. This is then populated with data in the demo code below.
 * 
 * PLEASE NOTE: If you are using a non-AVR Arduino, such as a Due, or other variants
 * such as a Chipkit or Teensy, then you will need to comment out the Flash String 
 * line below - Line 60, as it will prevent the demo from compiling.
 */

#include <genieArduino.h>

Genie screen1;
Genie screen2;

#define RESETLINE1 4  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)
#define RESETLINE2 5  // Change this if you are not using an Arduino Adaptor Shield Version 2 (see code below)

// Setup function
void setup()
{ 
  // Use a Serial Begin and serial port of your choice in your code and use the genie.Begin function to send 
  // it to the Genie library (see this example below)
  // max of 200K Baud is good for most Arduinos. Galileo should use 115200 or below.
  // Some Arduino variants use Serial1 for the TX/RX pins, as Serial0 is for USB.
  Serial.begin(115200);  
  Serial1.begin(115200);  // Serial0 @ 9600 Baud
  Serial2.begin(115200);  // Serial0 @ 9600 Baud
  screen1.Begin(Serial1);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
  screen2.Begin(Serial2);   // Use Serial0 for talking to the Genie Library, and to the 4D Systems display
  
  // Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
  // THIS IS IMPORTANT AND CAN PREVENT OUT OF SYNC ISSUES, SLOW SPEED RESPONSE ETC
  // If NOT using a 4D Arduino Adaptor, digitalWrites must be reversed as Display Reset is Active Low, and
  // the 4D Arduino Adaptors invert this signal so must be Active High.  
  pinMode(RESETLINE1, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(RESETLINE1, 0);  // Reset the Display via D4
  digitalWrite(RESETLINE2, 0);  // Reset the Display via D4
  delay(100);
  digitalWrite(RESETLINE1, 1);  // unReset the Display via D4
  digitalWrite(RESETLINE2, 1);  // unReset the Display via D4

  delay (3500); //let the display start up after the reset (This is important)

  //screen1.WriteContrast(0);
  screen1.WriteStr(0, "       OIL");
  //screen1.WriteStr(1, "     psi");
  //screen2.WriteStr(1, "    WATER");
  //screen2.WriteStr(5, "  deg. F");
 // screen1.WriteContrast(15);
 screen1.WriteObject(GENIE_OBJ_FORM, 0, 1);
}

int digits = 0;

// Main loop
void loop()
{
  //An optional third parameter specifies the base (format) to use; permitted values are BIN (binary, or base 2), OCT (octal, or base 8), DEC (decimal, or base 10), HEX (hexadecimal, or base 16). 
  //For floating point numbers, this parameter specifies the number of decimal places to use.
  //int x = -78;
  //long y = 171;
  //double z = 175.3456;
  digits = digits + 1;
  //Serial.println("TICK");
  screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 0, digits%100);
  delay(5);
  screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, digits%100);
  delay(5);
  screen1.WriteObject(GENIE_OBJ_ILED_DIGITS, 2, digits%10000);
  delay(5);
  screen1.WriteObject(GENIE_OBJ_USERIMAGES, 1, digits%13);
    delay(5);
  screen1.WriteObject(GENIE_OBJ_USERIMAGES, 2, digits%13);
    delay(5);
  screen1.WriteObject(GENIE_OBJ_USERIMAGES, 0, digits%7);
    delay(5);
  screen1.WriteObject(GENIE_OBJ_USERIMAGES, 4, digits%19);
    delay(5);
  //screen1.WriteObject(GENIE_OBJ_USERIMAGES, 5, digits%19);
   delay(5);

  //screen2.WriteObject(GENIE_OBJ_ILED_DIGITS, 1, -30);
  //screen2.WriteObject(GENIE_OBJ_ILED_DIGITS, 0, digits%100);

 
  
  //Serial.println("TOCK");
  //String Str = "This is string class";
  //genie.WriteStr(0, "TEST");  // Write to String0 Object, with the string "TEST"
  //delay(1000);
  //genie.WriteStr(0, z, digits); //3 decimal places
  //delay(100);
//  genie.WriteStr(0, 123.45678, 5); // 5 decimal places
//  delay(100);
//  genie.WriteStr(0, 123.45678); // 2 decimal places by default if no value is given to decimal place.
//  delay(100);
//  //genie.WriteStr(0, F("This string will be \n stored in flash memory")); // For AVR Arduinos only - Needs to be commented out for Due, Chipkit, Teensy etc.
//  //delay(1000);
//  //genie.WriteStr(0, "                                                        "); // Clear
//  //delay(10);
//  genie.WriteStr(0, x); //prints negative integer
//  delay(100);
//  genie.WriteStr(0, y);
//  delay(100);
//  genie.WriteStr(0, -x, BIN); //base 2 of 78
//  delay(100);
//  genie.WriteStr(0, y,16); //base 16
//  delay(100);
//  genie.WriteStr(0, 10); //base 10 by default
//  delay(100);
//  genie.WriteStr(0, 10,8); //base 8
//  delay(100);
//  genie.WriteStr(0, Str); //prints String Class
//  delay(100);
//  unsigned int zc = 123 ;
//  genie.WriteStr(0, zc); //prints unsigned ints
//  delay(100);
//  unsigned long e = 1234 ;
//  genie.WriteStr(0, e); //prints unsigned long
//  delay(100);
}

