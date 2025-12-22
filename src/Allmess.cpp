#include "Allmess.h"
#include <Arduino.h>
#include <Print.h>
#include <HardwareSerial.h>
#include <Stream.h>
#include "SoftwareSerial.h"

EspSoftwareSerial::UART swSer1;
const int SerRxBufMaxLen = 500;
uint8_t   SerRxBuf[SerRxBufMaxLen];
int       SerRxBufLen = 0;

// Hardware communication with Allmeas, activates IR-Transceiver, wake-up device, retrieve datablock, put IR to sleep again
void AllmessRead(int rxpin, int txpin, int rxenpin, Stream &DebugOut) 
{
  if(rxenpin >= 0) { 
    pinMode(rxenpin, OUTPUT); 
    digitalWrite(rxenpin, HIGH);    // Enable RX
    delay(10);
  }

  pinMode(rxpin, INPUT);
  pinMode(txpin, OUTPUT); digitalWrite(txpin, HIGH);  // IR LED

  if(&DebugOut != NULL) {
    DebugOut.print("AllmessRead: Using RX pin "); DebugOut.print(rxpin); DebugOut.print(", TX pin "); DebugOut.print(txpin); DebugOut.println();
    DebugOut.print("sending wakeup, ");
  }
  
  swSer1.begin(2400,EspSoftwareSerial::SWSERIAL_8N1,-1,txpin);
  swSer1.flush();
  uint8_t c;
  while(swSer1.available()) c = swSer1.read();

  // Send 2.2s 0x55 = 528, 3s = 
  for (int i=0;i<720;i++) {
    swSer1.write(0x55);
  }
  swSer1.flush();
  while(swSer1.available()) c = swSer1.read();
  // Wait min. 14ms (33 Bits time)
  delay(15); 
  swSer1.end();

  // REQ_UD2 (Request for User Data type 2), Point-To-Point Address, Short Telegram
  const char cmdbuf[5]={0x10,0x5B,0xFE,0x59,0x16};
  const char cmdSND_NKE[5]={0x10,0x40,0x00,0x40,0x16}; // Answer with 0xE5
  const char cmdSND_NKE_BCAST[5]={0x10,0x40,0xFE,0x3E,0x16}; // Answer with 0xE5
  swSer1.begin(2400,EspSoftwareSerial::SWSERIAL_8E1,rxpin,txpin);

  // Try send Reset
  for(int i=0; i<3; i++) {
    swSer1.write(cmdSND_NKE,5);
    //Serial1.flush();
    //delay(5); // wait that last byte is being sent
    //while(Serial1.available()) c = Serial1.read();

    if(&DebugOut != NULL) DebugOut.print("send NKE and read answer, ");

    // read RSP_UD for max 200ms (but reset time as long as new data are coming)
    uint32_t start_time = millis();
    while((uint32_t)(millis() - start_time) < (uint32_t)500) {
      if(swSer1.available()) {
        c = swSer1.read();
        if(c == 0xe5) {
          if(&DebugOut != NULL) DebugOut.print("0xe5 ");
          i = 100;
        }
      }
    }
  }  


  SerRxBufLen = 0;
  if(&DebugOut != NULL) DebugOut.print("sending 1st command, ");

  for(int i=0; i<3; i++) {
    swSer1.write(cmdbuf,5);
    swSer1.flush();
    delay(5); // wait that last byte is being sent
    while(swSer1.available()) c = swSer1.read();

    if(&DebugOut != NULL) DebugOut.print("await and read answer, ");

    // read RSP_UD for max 200ms (but reset time as long as new data are coming)
    uint32_t start_time = millis();
    while((uint32_t)(millis() - start_time) < (uint32_t)500) {
      if(swSer1.available()) {
        c = swSer1.read();
        start_time = millis();
        if(SerRxBufLen < SerRxBufMaxLen) {
          SerRxBuf[SerRxBufLen++] = c;
        }
      }
    }
    if(SerRxBufLen > 0) i=100;
  }

  if(&DebugOut != NULL) DebugOut.printf("done. Read %i bytes.\r\n", SerRxBufLen);
  swSer1.end();
  pinMode(txpin, OUTPUT); digitalWrite(txpin, HIGH);  // IR LED // IR LED Off

  if(rxenpin >= 0) { 
    pinMode(rxenpin, OUTPUT); 
    digitalWrite(rxenpin, LOW);    // Disable RX
  }
}
