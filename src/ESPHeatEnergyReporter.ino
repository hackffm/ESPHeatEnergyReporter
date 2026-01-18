/* ESPHeatEnergyReporter.ino
   ESP8266 based Heat Energy Meter Reporter via Allmess IR interface
   Reads heat energy meter data via Allmess IR interface and reports via Serial

   Copyright (c) 2025 Lutz Lisseck
 *
 */

#define IR_RX_PIN     14
#define IR_TX_PIN     12
#define IR_RXEN_PIN   13

String hostname = "EnergyMeter";
const char* NTP_SERVER = "pool.ntp.org";
const char* TZ_INFO    = "CET-1CEST,M3.5.0,M10.5.0/3";  // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for Timezone codes

// Wifi credentials are in a MyCreds.h file that must reside in /<HOME>/.platformio/lib/MyCreds/MyCreds.h
// see attic/MyCreds.h for an example
#if defined __has_include
#  if __has_include (<MyCreds.h>)
#    include <MyCredsHackffm.h>  // Define WIFI_SSID and WIFI_PASSWORD here - see file in Attic for example
#  endif
#endif

#if defined(ARDUINO_ARCH_ESP8266)
 #include <ESP8266WiFi.h>
 #include <WiFiUdp.h>
 #define DEBUG_ESP_BAUDRATE 74880
 #define LED1_PIN 2 // GPIO2 

 ADC_MODE(ADC_VCC);
#endif
//#include <NTPClient.h>
#include <time.h>
#include <sys/time.h> // struct timeval
#include <WS2812Write.h>
#include "LittleFS.h"
#include <elapsedMillis.h>

#include "Allmess.h"
#include "MBusParser.h"

MBusParser mbusParser;


bool GotTime = false;
tm timeinfo;
time_t now;

typedef struct {
  uint32_t magic;
  uint32_t sleep_seconds_left;
  uint32_t max_seconds_per_sleep;
  uint32_t boot_cnt;
  uint32_t target_hour;
  uint32_t target_minute;
  time_t   time_before_sleep;
  time_t   target_time_after_sleep;
  time_t   target_sleep_seconds;
} rtc_data_t;
rtc_data_t rtc_data;

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
char  replyPacket[] = "Ok.";  // a reply string to send back
IPAddress broadcastIP(255, 255, 255, 255);  

float volts; 

time_t epochTime;

bool startWifi() {
  WiFi.setHostname(hostname.c_str());
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA); // WiFi.disconnect(true);
  
  #if defined(ARDUINO_ARCH_ESP8266)
  WiFi.setOutputPower(10);
  #else
  WiFi.setTxPower(WIFI_POWER_7dBm);
  #endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  bool ret = false;

  int tries = 0;
  Serial.println("Connecting to WiFi..");
  while ((WiFi.status() != WL_CONNECTED) && (tries < 20)) {
    delay(500);
    Serial.print(".");
    tries++;
  }
  if(WiFi.status() == WL_CONNECTED) {
    configTime(0, 0, NTP_SERVER);
    setenv("TZ", TZ_INFO, 1); tzset();

    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RRSI: ");
    Serial.println(WiFi.RSSI());

    uint32_t start = millis();
    do {
      time(&now);
      localtime_r(&now, &timeinfo);
      Serial.print(".");
      delay(10);
    } while (((millis() - start) <= (1000 * 3)) && (timeinfo.tm_year < (2016 - 1900)));
    if (timeinfo.tm_year >= (2025 - 1900)) {
      GotTime = true;
    } 

    // from here time is available
    time(&epochTime);
    tm local_time;
    localtime_r(&epochTime, &local_time);
    char buf[40];
    strftime(buf, sizeof(buf), "%Y%m%d %H:%M:%S", &local_time);
    Serial.printf("%s \r\n", buf);  

    Udp.begin(localUdpPort);
    Udp.beginPacket(broadcastIP, localUdpPort);
    Udp.printf("Vcc: %1.3f V, ID: %s\n", volts/1000, hostname.c_str());
    Udp.endPacket();

    ret = true;
  } else {
    Serial.println("Wifi connection failed!");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
  }
  return ret;
}

// Turns system off, sleeps for sleepSeconds seconds
void goSleep(uint16_t sleepSeconds, RFMode rfmode) {
  //ws2812Write(RGB_PIN, 0);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  digitalWrite(LED1_PIN, HIGH);
  Serial.flush(); delay(1);
  //digitalWrite(2, LOW);
  ESP.deepSleep(sleepSeconds * 1000000UL, rfmode);
  while(1) yield();
}

// Set a new wake time that is in the future
// As the CPU might have been waken up too early, you can sepcify a minimum number of seconds ahead
// example: now it is 19:50, wakeTarget is 20:00 -> with minSecondsAhead of 30min (1800s), 
// the wakeup will be set to 20:00 but on the next day. 
void setWakeTime(uint8_t hour, uint8_t minute, uint16_t minSecondsAhead, uint32_t maxSecondsPerSleep, float clockErrorPpm=0.0) {
  struct tm timeinfo;
  time_t now;
  time(&now); // Current time in seconds since epoch
  localtime_r(&now, &timeinfo); // Get current local timme here
  timeinfo.tm_hour = hour;
  timeinfo.tm_min = minute;
  timeinfo.tm_sec = 0;
  time_t target = mktime(&timeinfo);
  // Target could be in the past if the system woke up a little too late (eg now: 20:05, target 20:00)
  if(target <= now) target += 24 * 3600;
  int32_t sleepSeconds = (int32_t)(target - now);

  // If the target time is too close, move it to the next day
  if(sleepSeconds < minSecondsAhead) { 
    sleepSeconds += 24 * 3600;
    target += 24 * 3600;
  }
  
  if(clockErrorPpm != 0.0) {
    // Adjust sleep time according to clock error
    //float adjustmentFactor = 1.0 + (clockErrorPpm / 1000000.0);
    sleepSeconds += (int32_t)((float)sleepSeconds * (clockErrorPpm / 1000000.0));
  }
  if(sleepSeconds < 0) sleepSeconds = 0;
  rtc_data.target_sleep_seconds = sleepSeconds; // store after adjustment
  Serial.printf("Setting wakeup time to %02u:%02u, in %u seconds\r\n", hour, minute, sleepSeconds);
  rtc_data.magic = 0xDEADBEEF;
  rtc_data.target_hour = hour;
  rtc_data.target_minute = minute;
  rtc_data.sleep_seconds_left = sleepSeconds;
  rtc_data.max_seconds_per_sleep = maxSecondsPerSleep;
  rtc_data.time_before_sleep = now;
  rtc_data.target_time_after_sleep = target;
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc_data, sizeof(rtc_data_t));
  if(rtc_data.sleep_seconds_left > 0) {
    if((uint32_t)sleepSeconds > rtc_data.max_seconds_per_sleep) {
      sleepSeconds = rtc_data.max_seconds_per_sleep;
    }
    rtc_data.sleep_seconds_left -= sleepSeconds;
    Serial.printf("Going to sleep for first %u seconds\r\n", sleepSeconds);
    ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc_data, sizeof(rtc_data_t));
    if((uint32_t)sleepSeconds > rtc_data.max_seconds_per_sleep) {
      goSleep(sleepSeconds, WAKE_RF_DISABLED);
    } else {
      goSleep(sleepSeconds, WAKE_RFCAL);
    }
  }
}

elapsedMillis loopTimer = 0;
uint8_t AfterDeepSleep = 0;
float ClockErrorPpm = 0.0;
void setup() {
  WiFi.mode(WIFI_OFF); // Disable WiFi to save power
  struct rst_info *rstInfo = system_get_rst_info();
  volts = ESP.getVcc();
  
  Serial.begin(DEBUG_ESP_BAUDRATE);
  Serial.println("ESPHeatEnergyReporter starting up...");
  pinMode(LED1_PIN, OUTPUT);
  analogWrite(LED1_PIN, 254); // Turn the LED on but dimmed

  // Check how we got here
  if(rstInfo->reason == REASON_DEEP_SLEEP_AWAKE) {
    if(ESP.rtcUserMemoryRead(0, (uint32_t*)&rtc_data, sizeof(rtc_data_t))) {
      if(rtc_data.magic == 0xDEADBEEF) {
        if(rtc_data.sleep_seconds_left > 0) {
          // More sleep cycles to go
          uint32_t sleepSeconds = rtc_data.sleep_seconds_left;
          if(sleepSeconds > rtc_data.max_seconds_per_sleep) {
            sleepSeconds = rtc_data.max_seconds_per_sleep;
          }
          rtc_data.sleep_seconds_left -= sleepSeconds;
          Serial.printf("Woke from deep sleep, going back to sleep for another %u seconds, %u seconds left\r\n", sleepSeconds, rtc_data.sleep_seconds_left);
          ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtc_data, sizeof(rtc_data_t));
          if(sleepSeconds > rtc_data.max_seconds_per_sleep) {
            goSleep(sleepSeconds, WAKE_RF_DISABLED);
          } else {
            goSleep(sleepSeconds, WAKE_RFCAL);
          }
        } else {
          // Final wakeup
          rtc_data.boot_cnt++;
          AfterDeepSleep = 1;
        }
      }
    } else {
      Serial.println("RTC memory read failed.");
      rtc_data.magic = 0;
      rtc_data.boot_cnt = 0;
    }
  } else {
    // First boot or other reset
    Serial.println("Not waking from deep sleep.");
    rtc_data.magic = 0;
    rtc_data.boot_cnt = 0;
  }

  startWifi(); 
  if((AfterDeepSleep == 1) && GotTime) {
    // We woke up from deep sleep and time is available
    time(&now); 
    localtime_r(&now, &timeinfo);
    time_t targetSleepSeconds = rtc_data.target_sleep_seconds;
    time_t actualSleepSeconds = now - rtc_data.time_before_sleep;
    int32_t timeDiff = (int32_t)(targetSleepSeconds - actualSleepSeconds);
    Serial.printf("Final woke from deep sleep, current time %02d:%02d:%02d\r\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    Serial.printf("Intended sleep time %u seconds, actual sleep time %u seconds, difference %+d seconds too fast.\r\n",
      (uint32_t)targetSleepSeconds, (uint32_t)actualSleepSeconds, timeDiff);
    float ClockErrorPpmCalc = ((float)timeDiff / (float)targetSleepSeconds) * 1000000.0;
    Serial.printf("Clock error: %+0.2f ppm = %+0.5f %%\r\n", ClockErrorPpmCalc, ClockErrorPpmCalc / 10000.0);
    // Only accept clock errors below 15%
    if(abs(ClockErrorPpmCalc) < 150000.0) {
      ClockErrorPpm = ClockErrorPpmCalc;
    }
  }
   

  Serial.printf("deepSleepMax %u s\r\n", (uint32_t)(ESP.deepSleepMax()/1000000UL));
  Serial.printf("Reset reason: %d\r\n", rstInfo->reason);
  uint32_t rtc_ts1 = system_get_rtc_time();
  delay(1000);
  uint32_t rtc_ts2 = system_get_rtc_time();
  Serial.printf("RTC timer: %u -> %u (diff %u)\r\n", rtc_ts1, rtc_ts2, rtc_ts2 - rtc_ts1);
  Serial.printf("RTC_Time: %u\r\n", system_get_rtc_time());
  Serial.printf("clk cal : %d \n\r",system_rtc_clock_cali_proc());

  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  
  File file = LittleFS.open("/text.txt", "r");
  if(!file){
    Serial.println("Failed to open file for reading");
  } else {
    FSInfo info {};
    LittleFS.info(info);
    Serial.printf("File Content (free bytes %u):\r\n", info.totalBytes - info.usedBytes);
    while(file.available()){
      Serial.write(file.read());
    }
    file.close();
  }
  loopTimer = 0;
}


void loop() {
  if((loopTimer % 500) > 250) {
    digitalWrite(LED1_PIN, HIGH); // Turn the LED off
  } else {
    analogWrite(LED1_PIN, 254); // Turn the LED on but dimmed
  }
  if(Serial.available()) {
    uint8_t c = Serial.read();
    if(c == 'r') {
      Serial.println("Reading Allmess data...");
      AllmessRead(IR_RX_PIN, IR_TX_PIN, IR_RXEN_PIN, Serial); // RX, TX, RX Enable
      if(SerRxBufLen > 0) {
        mbusParser.ParseLL(SerRxBuf, SerRxBufLen);
      }
    } 
  }
  if(loopTimer > 2000) {
    Serial.println("Going to deep sleep...");
    //goSleep(30, RF_DEFAULT);  
    if(GotTime) {
      struct tm timeinfo;
      time_t now;
      time(&now);
      localtime_r(&now, &timeinfo);

      // Append timestamp to file
      File file = LittleFS.open("/text.txt", "a");
      if(!file){
        Serial.println("Failed to open file for writing");
      } else {
        char buf[40];
        strftime(buf, sizeof(buf), "%Y%m%d %H:%M:%S", &timeinfo);
        file.printf("%s, %4.0f, %1.3f, %.3f, %.2f, %.2f, %.2f\r\n",
          buf, ClockErrorPpm, volts/1000, mbusParser.EnergykWh, mbusParser.Volumem3, mbusParser.FlowTempHot, mbusParser.FlowTempCold);
        file.close();
      }

      Serial.printf("Current time %02d:%02d:%02d\r\n", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
      now += 15 * 60; // Add 15min to current time to avoid waking up too early
      localtime_r(&now, &timeinfo);
      timeinfo.tm_min = 10;
      timeinfo.tm_hour += 1;
      if(timeinfo.tm_hour >= 24) {
        timeinfo.tm_hour = 0;
      }

      Serial.printf("Setting next wakeup time to %02u:%02u\r\n", timeinfo.tm_hour, timeinfo.tm_min);
      setWakeTime(timeinfo.tm_hour, timeinfo.tm_min, 0,  120*60 /* 300 */, ClockErrorPpm);
      
    } else {
      goSleep(600, WAKE_RFCAL);  // No time available, sleep for 10min and try again
    }
  }
}
