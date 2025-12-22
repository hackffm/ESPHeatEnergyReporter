/* ESPHeatEnergyReporter.ino
   ESP8266 based Heat Energy Meter Reporter via Allmess IR interface
   Reads heat energy meter data via Allmess IR interface and reports via Serial

   Copyright (c) 2024  Thomas M. (
 *
 */

#define IR_RX_PIN     14
#define IR_TX_PIN     12
#define IR_RXEN_PIN   13

#if defined(ARDUINO_ARCH_ESP8266)
 #include <ESP8266WiFi.h>
 #define DEBUG_ESP_BAUDRATE 74880
 #define LED1_PIN 2 // GPIO2 

 ADC_MODE(ADC_VCC);
#endif
#include <WS2812Write.h>
#include "LittleFS.h"

#include "Allmess.h"
#define START_ADDRESS 0x13  // Start address for decoding

char     CustomerNumber[10] = "xxxxxxxx\0";
uint8_t  Alarm1 = 0; 
uint8_t  Alarm2 = 0;
bool     ChecksumOk = false;
uint32_t EnergyValue = 0;
int8_t   EnergyUnit = 0;
double   EnergykWh = 0.0;
uint32_t VolumeValue = 0;
int8_t   VolumeUnit = 0;
double   Volumem3 = 0.0;
uint8_t  TimePointHour = 0;
uint8_t  TimePointMin = 0;
uint8_t  TimePointDay = 0;
uint8_t  TimePointMonth = 0;
uint8_t  TimePointYear = 0; 
float    FlowTempHot = 0.0;
float    FlowTempCold = 0.0;

void ParseMBusDataLL(uint8_t *buf, int bufLen) {
  uint8_t c;

  Serial.println("Dump Buffer:");
  for(int i=0; i<bufLen; i++) {
    Serial.printf("0x%02x, ", (int)buf[i]);
    if((i % 16) == 15) Serial.println(" ");
  }  

  Serial.println("\r\nParse Buffer:");
  int state1 = 0;
  int state2 = 0;
  uint8_t LL1 = 0;
  uint8_t LL2 = 0;
  const int8_t diff_lenghts[16] = {0,1,2,3,4,4,6,8,0,1,2,3,4,6,8,0}; 
  int8_t diff_l = -1;
  int cnt = 0;
  int cnt2 = 0;
  uint8_t dif = 0;
  uint8_t dife1 = 0;
  uint8_t vif = 0;
  uint8_t vife1 = 0;
  uint8_t checksum = 0; ChecksumOk = false;
  uint64_t val64 = 0;
  for(int i=0; i<bufLen; i++) {
    c = buf[i];
    switch(state1) {
      case 0:
        // try detect long frame
        if(c == 0x68) state1 = 1;
        break;
      case 1:
        // read first LL byte
        LL1 = c;
        state1++;
        break;
      case 2:
        // read second LL byte
        LL2 = c;
        state1++;
        break;
      case 3:
        // validate header
        if((c != 0x68) || (LL1 != LL2)) {
          Serial.println(" Malformed header.");
          state1 = 0;
        } else {
          Serial.printf(" Header found, %i bytes length:\r\n", LL1);
          state1++;
          cnt=0;
        }        
        break;
      case 4:
        if(c != 0x08) {
          Serial.println(" No 0x08 subheader.");
          state1 = 0;
        } else {
          if(bufLen - i - 1 < LL1) {
            Serial.println(" Insufficient data received!");
            state1 = 0;
          } else {
            for (int j=0; j<LL1; j++) checksum += buf[i+j];
            i++;
            Serial.printf(" Subheader 0x08, Slave address 0x%02X, CI 0x%02X \r\n", (int)buf[i++], (int)buf[i++]);
            snprintf(CustomerNumber, 9, "%02x%02x%02x%02x", (int)buf[i+3], (int)buf[i+2], (int)buf[i+1], (int)buf[i]);
            Serial.printf(" Customer number %s\r\n", CustomerNumber);
            i += 4;
            Serial.printf(" Manufacturer number %02X %02X, Gen %02X\r\n", (int)buf[i++], (int)buf[i++], (int)buf[i++]);
            Serial.printf(" Medium %02X, Number of reading %i, Error code %02X\r\n", (int)buf[i++], (int)buf[i++], (int)buf[i++]);
            i += 1; // Skip signature
            state1++;
            cnt = 0;
            cnt2 = 0;
          }
        }
        break;  
      case 5:
        diff_l = diff_lenghts[c & 0x07];
        if(c == 0x0f) {
          i++;
          Alarm1 = buf[i++];
          Alarm2 = buf[i++];
          uint8_t ChecksumTest = buf[i++];
          Serial.printf(" DIF ENDMARKER, Alarm 0x%02X 0x%02X, Check Sum 0x%02X ?= 0x%02x, End Marker 0x%02X\r\n", 
            (int)Alarm1, (int)Alarm2, (int)ChecksumTest, (int)checksum, (int)buf[i++]);
          ChecksumOk = (ChecksumTest == checksum);
          state1 = 100;
        } else {
          dif = c;
          dife1 = 0;
          cnt = diff_l;
          cnt2 = 0;
          if(c & 0x80) {
            Serial.printf(" DIF+ 0x%02X (len = %i),", (int)c, cnt);
            dife1 = buf[i+1];
            state1++;
          } else {
            Serial.printf(" DIF 0x%02X (len = %i)\r\n    ", (int)c, cnt);  
            state1=10;          
          }
        }
        break;
      case 6:
        diff_l = diff_lenghts[c & 0x07];
        cnt += diff_l;
        if(c & 0x80) {
          Serial.printf(" DIF+ 0x%02X (len = %i),", (int)c, cnt);
        } else {
          Serial.printf(" DIF 0x%02X (len = %i)\r\n    ", (int)c, cnt);  
          state1=10;          
        }
        break;
      case 10:
        if(cnt2 == 0) vif = c;
        if(cnt2 == 1) vife1 = c;
        if(c & 0x80) {
          Serial.printf("VIF+ 0x%02X (len = %i), ", (int)c, cnt2);
          cnt2++;
        } else {
          Serial.printf("VIF 0x%02X (len = %i)\r\n    ", (int)c, cnt2);  
          state1++; 
          // Decode main numbers 
          uint8_t dif_ln = dif & 0x0f;
          val64 = 0;
          if((dif_ln == 1) || (dif_ln == 2) || (dif_ln == 3) || (dif_ln == 4) || (dif_ln == 6) || (dif_ln == 7)) {
            for(int j=0; j<cnt; j++) val64 = (val64 << 8) + buf[i + cnt - j];
            Serial.printf("val = %llu = 0x%llx\r\n    ", val64, val64);
          } else if((dif_ln == 9) || (dif_ln == 10) || (dif_ln == 11) || (dif_ln == 12) || (dif_ln == 14)) {
            for(int j=0; j<cnt; j++) {
              uint8_t h;
              h = buf[i + cnt - j];
              if(((h & 0xf0) >> 4) > 9) h = 0;
              if((h & 0x0f) > 9) h = 0;
              int hval = ((h & 0xF0) >> 4) * 10 + (h & 0x0F);
              val64 = (val64 * 100) + hval;
            }
            Serial.printf("val = %llu = 0x%llx\r\n    ", val64, val64);
          }


          // Decode some non-extended VIFs here: dif, cnt, vif is filled here, i+1 points to first data byte
          if(((vif & 0xf8) == 0x00) && ((dif & 0xc0) == 0x00)) {
            EnergyUnit = (vif & 0x07) - 3;
            EnergyValue = val64;
            EnergykWh = (double)EnergyValue * pow(10.0, EnergyUnit - 3); // convert to kWh
            Serial.printf("Energy %d*E%d Wh, %.3f kWh\r\n    ", EnergyValue, (int)EnergyUnit, EnergykWh);
          }
          if(((vif & 0xf8) == 0x10) && ((dif & 0xc0) == 0x00)) {
            VolumeUnit = (vif & 0x07) - 6;
            VolumeValue = val64; 
            Volumem3 = (double)VolumeValue * pow(10.0, VolumeUnit);
            Serial.printf("Volume %d*E%d m3, %.2f m3\r\n    ", VolumeValue, (int)VolumeUnit, Volumem3);
          }
          if(((vif & 0xf8) == 0x58) && ((dif & 0xc0) == 0x00)) {
            int8_t Unit = (vif & 0x03) - 3;
            float  TempC = (float)val64 * pow(10.0, Unit);
            if(vif & 0x04) {
              FlowTempCold = TempC; Serial.printf("Flow temperature cold %.1f °C\r\n    ", TempC);
            } else {
              FlowTempHot  = TempC; Serial.printf("Flow temperature hot  %.1f °C\r\n    ", TempC);
            }
          }
          if(((vif & 0xfe) == 0x6c) && ((dif & 0xc0) == 0x00)) {
            if(vif & 0x01) {
              TimePointMin   = (uint8_t)((val64 >> 0) & 0x3f);
              TimePointHour  = (uint8_t)((val64 >> 8) & 0x1f);
              val64 = (val64 >> 16) & 0xffff;
            } else {
              TimePointMin = 0; TimePointHour = 0;
            }
            TimePointDay   = (uint8_t)((val64 >> 0) & 0x1f);
            TimePointMonth = (uint8_t)((val64 >> 8) & 0x0f);
            TimePointYear  = (uint8_t)((val64 >> 5) & 0x07) + (uint8_t)((val64 >> 9) & 0x78);
            Serial.printf("TimePoint 20%02d-%02d-%02d %02d:%02d\r\n    ", (int)TimePointYear, (int)TimePointMonth,
              (int)TimePointDay, (int)TimePointHour, (int)TimePointMin);
          }

        }    
        break;
      case 11:
        if(cnt == 0) {
          i--;
          state1 = 5;
          Serial.println(" .");
        } else {
          Serial.printf("%02X ", c);
          cnt--;
        }
        break;



    }

  }   
}

void setup() {
  WiFi.mode(WIFI_OFF); // Disable WiFi to save power
  Serial.begin(DEBUG_ESP_BAUDRATE);
  Serial.println("ESPHeatEnergyReporter starting up...");
  pinMode(LED1_PIN, OUTPUT);

  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  
  File file = LittleFS.open("/text.txt", "r");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  
  Serial.println("File Content:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();


}

void loop() {
  digitalWrite(LED1_PIN, HIGH); // Turn the LED off
  delay(1000);
  analogWrite(LED1_PIN, 254); // Turn the LED on but dimmed
  delay(1000);
  Serial.println("Looping...");
  if(Serial.available()) {
    uint8_t c = Serial.read();
    if(c == 'r') {
      Serial.println("Reading Allmess data...");
      AllmessRead(IR_RX_PIN, IR_TX_PIN, IR_RXEN_PIN, Serial); // RX, TX, RX Enable
      if(SerRxBufLen > 0) {
        ParseMBusDataLL(SerRxBuf, SerRxBufLen);
      }
    } 
  }
}

