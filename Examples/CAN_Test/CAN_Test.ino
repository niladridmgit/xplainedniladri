#include "samc21_can.h"

SAMC21_CAN can(0);

uint8_t buf[8]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

void setup()
{
    Serial.begin(115200);
    uint8_t ret;
    ret = can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ);
    if (ret == CAN_OK) {
        Serial.println("CAN Initialized Successfully!");
    } else {
        Serial.println("Error Initializing CAN...");
    }
}

void loop()
{
    uint8_t ret;
    uint32_t id=0x101;
    uint8_t len= 8;
    
    //uint8_t i;
    
    /*ret = can.readMsgBuf(&id, &len, buf);
    if (ret == CAN_OK) {
        Serial.print("Got a message from: ");
        Serial.print(id);
        Serial.print("  Length: ");
        Serial.print(len);
        Serial.print("  Data: ");
        for (i = 0; i < len; i++) {
            Serial.print(buf[i], HEX);
        }
        Serial.println("");
    }*/

    ret = can.sendMsgBuf(id, 0, len, buf);
    if(ret == CAN_FAILTX)
    {
      Serial.println("Send Fail");
    }
    delay(100);
    
}
