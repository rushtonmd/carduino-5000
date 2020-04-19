// CAN Receive Example
//

#include <mcp_can.h>
#include <SPI.h>
#include <VariableTimedAction.h>
#include <SoftwareSerial.h>
#include "Nextion.h"
//#include "NexText.h"

//SoftwareSerial mySerial(10, 11);

//NexText t0 = NexText(0, 10, "t0");
//t0.setText("TEST");

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];

// PID Constants for the data blocks
unsigned long RPM_BLOCK_PID = 0x360;
unsigned long FUEL_PRESSURE_BLOCK_PID = 0x361;
unsigned long BATTERY_VOLTAGE_BLOCK_PID = 0x372;
unsigned long COOLANT_TEMP_BLOCK_PID = 0x3E0;
unsigned long TRANSMISSION_OIL_TEMP_BLOCK_PID = 0x3E1;

int RPM = 0;
float MANIFOLD_PRESSURE = 0.0;
float THROTTLE_POSITION = 0.0;
float COOLANT_PRESSURE = 0.0;
float FUEL_PRESSURE = 0.0;

int message_counter = 0;

#define CAN0_INT 2 // Set INT to pin 2
MCP_CAN CAN0(9);   // Set CS to pin 10

void writeString(String stringData)
{ // Used to serially push out a String with Serial.write()

    for (int i = 0; i < stringData.length(); i++)
    {
        //Serial1.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
    }

    Serial1.print(stringData);
    Serial1.write(0xff); //We need to write the 3 ending bits to the Nextion as well
    Serial1.write(0xff); //it will tell the Nextion that this is the end of what we want to send.
    Serial1.write(0xff);

    Serial.println(stringData);

} // end writeString function

class UIUpdater : public VariableTimedAction
{
private:
    unsigned long run()
    {
        //Serial.println("---- VALUES ----");
        //Serial2.print("t0.txt=\"");
        //Serial2.print("ABC");
        //Serial2.print("\"");
        //Serial2.print("\xFF\xFF\xFF");
        if (message_counter > 0)
        {
            Serial.println(message_counter);
            Serial.println(RPM);
            //Serial.println(MANIFOLD_PRESSURE);
            //Serial.println(THROTTLE_POSITION);
            //Serial.println(COOLANT_PRESSURE);
            //Serial.println(FUEL_PRESSURE);
            //writeString("n0.val="+ String(RPM)); // write to nextion display
            //writeString("z0.val="+ String(int(RPM * 0.027) - 45));
        }
        else
        {
            Serial.println("NO SIGNAL");
        }

        message_counter = 0;

        //returns the amount in seconds to wait before executing the next event
        //if 0 is returned, then the previous interval is maintained
        return 0;
    }

public:
    // Custom constructor if needed - this can be used to setup anything specific
    // to the UIUpdater during initialization
    UIUpdater()
    {
    }
};

// Initialize the UIUpdater class
UIUpdater UI_Updater;

void setup()
{
    Serial.begin(9600);
    Serial1.begin(115200);

    //mySerial.begin(9600); // set the data rate for the SoftwareSerial port
    Serial1.write(0xFF);
    Serial1.write(0xFF);
    Serial1.write(0xFF);
    writeString("n0.val=8");
    //Serial2.begin(9600);
    //nexInit();
    //Serial1.begin(9600);
    //mySerial.begin(9600);
    //NexText x = NexText(0,5,"t2");
    //x.setText("WOOT");

    // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) == CAN_OK)
        Serial.println("MCP2515 Initialized Successfully!");
    else
        Serial.println("Error Initializing MCP2515...");

    CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

    SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0)); // 4 MHz clock,

    Serial.println("MCP2515 Library Receive Example...");

    // Update the UI 10 times per second
    UI_Updater.start(100);
}
int ByteReceived;
void loop()
{
    VariableTimedAction::updateActions();
    //Serial2.println("TEST");

    if (Serial.available() > 0)
    {
        Serial.write(Serial.read());
    }

    if (!digitalRead(CAN0_INT))
    {
        while (CAN0.readMsgBuf(&rxId, &len, rxBuf) == CAN_OK)
        {
            message_counter++;
            if (rxId == RPM_BLOCK_PID)
            {
                // Bits 0 and 1 are the RPM value. Shift the [0] bits left 8 digits then add
                // to the [1] bits. This will give the correct integer value for the 2 sets
                // of hex values taken together.
                RPM = (rxBuf[0] << 8) + rxBuf[1];

                // Bits 2 and 3 are the Manifold Pressure value. Shift the [0] bits left 8 digits then add
                // to the [1] bits. This will give the correct integer value for the 2 sets
                // of hex values taken together.
                MANIFOLD_PRESSURE = ((rxBuf[2] << 8) + rxBuf[3]) * 0.014503773779; // to PSI

                // Bits 0 and 1 are the RPM value. Shift the [0] bits left 8 digits then add
                // to the [1] bits. This will give the correct integer value for the 2 sets
                // of hex values taken together.
                THROTTLE_POSITION = ((rxBuf[4] << 8) + rxBuf[5]) * 0.1; // to Percent

                // Bits 0 and 1 are the RPM value. Shift the [0] bits left 8 digits then add
                // to the [1] bits. This will give the correct integer value for the 2 sets
                // of hex values taken together.
                COOLANT_PRESSURE = ((rxBuf[6] << 8) + rxBuf[7]) * 0.014503773779; // to PSI
            }
            else if (rxId == FUEL_PRESSURE_BLOCK_PID)
            {
                // Bits 0 and 1 are the Fuel Pressure value. Shift the [0] bits left 8 digits then add
                // to the [1] bits. This will give the correct integer value for the 2 sets
                // of hex values taken together.
                FUEL_PRESSURE = ((rxBuf[0] << 8) + rxBuf[1]) * 0.014503773779; // to PSI
            }
        }
    }

    // Short delay to ensure things keep moving
    delay(1);
}

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/