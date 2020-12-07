// Mesh node5 - PIR sensor

#include <Arduino.h>

#include <EspNowFloodingMesh.h>
#include <SimpleMqtt.h>

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h>
//#include "Adafruit_Si7021.h"

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
//#include <ArduinoJson.h>          // https://github.com/bblanchon/ArduinoJson

#include <SimpleTimer.h>          // Simple Task Time Manager
SimpleTimer timer;

void set_op_mode(uint8_t op_mode);

//#include <Bounce2.h>
// input sensors
//Bounce s1 = Bounce(); 
//Bounce s2 = Bounce();

// not used
// hold pin 13
int holdPin = 13;            // defines GPIO 0 as the hold pin (will hold CH_PD high untill we power down).

ADC_MODE(ADC_VCC);
int vdd;


#define DEBUG(x)            Serial.println(x);

#define LDR_ANALOG_PIN      A0

#define PIN_LED             2

void ota_loop(unsigned long );

//flag for saving data
bool shouldSaveConfig = false;


// CRC function used to ensure data validity
uint32_t calculateCRC32(const uint8_t *data, size_t length);

// helper function to dump memory contents as hex
void printMemory();

// !!! Arduiono OTA uses first 128 bytes
// Structure which will be stored in RTC memory.
// First field is CRC32, which is calculated based on the
// rest of structure contents.
// Any fields can go after CRC32.
struct {
  uint32_t crc32;
  uint32_t mode;
  char ssid[32];
  char pwd[32];
} rtcData;

// Adafruit_BMP280 bme; // I2C
//Adafruit_Si7021 sensor = Adafruit_Si7021();

// node boot time
time_t boot_time;


// your private mesh secret keys
#include <espmesh.h>

#ifndef ESP_NOW_CHANNEL
     // default - change this
     #define AP_PWD          "12345678"
     #define ESP_NOW_CHANNEL 1
     // AES 128bit
     unsigned char secredKey[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                                      0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
     unsigned char iv[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                                      0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
     int bsid = 0x010101;
     const int ttl = 3;
#endif

/******** MESH NODE SETUP ********/
#define DEVICENAME      "pir2round"
#define AP_NAME         "MESH_" DEVICENAME
#define AP_NAME_OTA     "MESH_OTA_" DEVICENAME
const char deviceName[] = DEVICENAME;
/*****************************/

bool LedOn = false;

SimpleMQTT simpleMqtt = SimpleMQTT(ttl, deviceName, 23, 40, 10);

bool new_ota_command = false;
#define MAX_OTA_COMMAND_LEN 50
char ota_command[MAX_OTA_COMMAND_LEN];

void reset_node(void) {
  ESP.reset();
  delay(5000);
}

void process_ota_command(char *cmd) {
  uint8_t l = strlen(cmd);
  Serial.print(">>> OTA CMD: ");
  Serial.println(cmd);

  if (strcmp("start", cmd) == 0) {
    set_op_mode(7);
    timer.setTimeout(1, reset_node);
    return;
  }

  if (strncmp("pow_", cmd, 4) == 0) {
    char *p = cmd + 4;
    float dBm = atof(p);
    WiFi.setOutputPower(dBm);
    Serial.print("Set tx power: ");
    Serial.println(dBm);
  }

  if (strcmp("mac_addr", cmd) == 0) {
    byte mac[6];
    char mac_addr[20];
    WiFi.macAddress(mac);
    sprintf(mac_addr, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
    simpleMqtt._string(PUBLISH, "mac_addr", mac_addr);
    Serial.print("my mac addr: ");
    Serial.println(mac_addr);
    return;
  }

  if (strcmp("repeater_on", cmd) == 0) {
    espNowFloodingMesh_setToBatteryNode(false);
    Serial.println("OTA: Repeter ON");
    return;
  }

  if (strcmp("repeater_off", cmd) == 0) {
    espNowFloodingMesh_setToBatteryNode(true);
    Serial.println("OTA: Repeter OFF");
    return;
  }

  if (strcmp("stats", cmd) == 0) {
    Serial.println("OTA: STATS");
    telemetry_db_item *tdb = espNowFloodingMesh_get_tdb_ptr();

    bool found = false;
    uint16_t idx = 0;
    while (idx < TELEMETRY_STATS_SIZE && !found) {
      if (tdb[idx].mac_addr[0] == 0 && tdb[idx].mac_addr[1] == 0 &&
          tdb[idx].mac_addr[2] == 0 && tdb[idx].mac_addr[3] == 0 &&
          tdb[idx].mac_addr[4] == 0 && tdb[idx].mac_addr[5] == 0) {
        // found empty spot
        found = true;
        break;
      }
      idx++;
    }
    simpleMqtt._bin(PUBLISH, "stats", (uint8_t *)tdb,
                    idx * sizeof(telemetry_db_item));
    Serial.print("Bytes sent: ");
    Serial.println(idx * sizeof(telemetry_db_item));
    return;
  }

#define MAX_JSON_LEN 200
  if (strcmp("stats_pkt", cmd) == 0) {
    Serial.println("OTA: PACKETS STATS ");
    telemetry_stats_st *telemetry_stats =
        espNowFloodingMesh_get_tmt_stats_ptr();
    char json[MAX_JSON_LEN];
    snprintf(json, MAX_JSON_LEN, "{\"boot_ts\":%lu,\"sent\":%u,\"recv\":%u,\"dup\":%u,\"fwd\":%u}",
             boot_time,
             telemetry_stats->sent_pkt, 
             telemetry_stats->received_pkt,
             telemetry_stats->dup_pkt,
             telemetry_stats->fwd_pkt);
             // telemetry_stats->ttl0_pkt);
    simpleMqtt._string(PUBLISH, "stats_pkt", json);
    return;
  }

  if (cmd[0] == '#' && l >= 8 && l <= sizeof(rtcData.ssid) + sizeof(rtcData.pwd) ) {
    Serial.println("OTA: Storing new SSID and PWD");
    memset(rtcData.ssid, 0, sizeof(rtcData.ssid));
    memset(rtcData.pwd, 0, sizeof(rtcData.pwd));
    uint8_t i = 1;
    for (; (i < l) && cmd[i] != ':'; i++)
      ;  // find :
    strncpy(rtcData.ssid, cmd + 1, i - 1);
    if (l > i) {
      strncpy(rtcData.pwd, cmd + i + 1, l - i);
    }
    set_op_mode(8);  // set mode and write ssid:pwd
    timer.setTimeout(1, reset_node);
    return;
  }
}

void ledOff() {
  digitalWrite(PIN_LED, HIGH);
  LedOn = false;
}

// set node operational mode
void set_op_mode(uint8_t op_mode) {
  rtcData.mode = op_mode;
  // Update CRC32 of data
  rtcData.crc32 = calculateCRC32((uint8_t*) &rtcData.mode, sizeof(rtcData) - 4);
  // Write struct to RTC memory
  if (ESP.rtcUserMemoryWrite(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    Serial.print("Write new mode: ");
    // printMemory();
    Serial.println(op_mode);
  }
}

uint8_t check_op_mode() {

  // Read struct from RTC memory
  if (ESP.rtcUserMemoryRead(0, (uint32_t*) &rtcData, sizeof(rtcData))) {
    // Serial.println("Read: ");
    // printMemory();
    Serial.println();
    uint32_t crcOfData = calculateCRC32((uint8_t*) &rtcData.mode, sizeof(rtcData) - 4);
    //Serial.print("CRC32 of data: ");
    //Serial.println(crcOfData, HEX);
    //Serial.print("CRC32 read from RTC: ");
    //Serial.println(rtcData.crc32, HEX);
    if (crcOfData != rtcData.crc32) {
      Serial.println("CRC32 in RTC memory doesn't match CRC32 of data. Data is probably invalid!");
      return 0;
    } else {
      Serial.println("CRC32 check ok, data is probably valid.");
      uint8_t mode = rtcData.mode;
      return mode;
    }
  }
  return 0;
}


void wifiPortal(uint8_t mode) {
  
  if (mode == 8) {
    // rtcData was read at the check_op_mode and it's valid
    WiFi.begin(rtcData.ssid, rtcData.pwd);
    uint8_t timeoutcnt = 10;
    while (WiFi.status() != WL_CONNECTED && timeoutcnt-- > 0) {
      delay(500);
      Serial.print(".");
    } 
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Connected to rtcData wifi AP");
      WiFi.printDiag(Serial);
      ota_loop(300);
      set_op_mode(3);
      Serial.println("OTA timeout, Reboot...");
      delay(500);
      ESP.reset();
    }
  }

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // reset settings - for testing
  // wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(120);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  // const char *ap_name = "MESH_" + DEVICENAME;
  if (!wifiManager.autoConnect(AP_NAME, AP_PWD)) {
    Serial.println("failed to connect and hit timeout.");
    Serial.println("Falling to OTA mode...");
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
	  WiFi.softAP(AP_NAME_OTA, AP_PWD);
    MDNS.begin("esp8266", WiFi.softAPIP());
    ota_loop(300);
    //reset and try again, or maybe put it to deep sleep
    Serial.println("Reboot...");
    set_op_mode(3);
    delay(500);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");

  //Serial.print("\nlocal ip: ");
  //Serial.println(WiFi.localIP());

  WiFi.printDiag(Serial);

  ota_loop(300);
  set_op_mode(2);
  
  Serial.println("OTA timeout, Reboot...");
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  ESP.reset();
  delay(5000);
}

// global time stamp for last update 
unsigned long last_time = 0;

void ota_loop(unsigned long int timeout_s) {

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  ArduinoOTA.setPassword(AP_PWD);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
    //set_op_mode(1); // mesh mode
    // Reboot 
    Serial.println("OTA done, Reboot...");
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    // timeout_s += 2000;
    last_time = millis();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Waiting fro OTA");

  unsigned long now = millis();
 
  timeout_s = timeout_s * 1000; // to ms

  while ( (now - last_time) < timeout_s ) {
    // non blocking
    ArduinoOTA.handle();
    delay(10);
    now = millis();
  }

  Serial.println("OTA timeout.");
}

unsigned long start_time;

void falltosleep() {
  DEBUG("Sleep...\n");
  Serial.print("> before sleep: "); Serial.println(millis() - start_time);

  digitalWrite(holdPin, LOW);  // set GPIO 0 low this takes CH_PD & powers down the ESP
  ESP.deepSleepInstant(3600e6, WAKE_RF_DEFAULT);

  // ESP.deepSleep(60e6); // 60 sec
  // RF_NO_CAL
  // ESP.deepSleepInstant(microseconds, mode); // mode WAKE_RF_DEFAULT, WAKE_RFCAL, WAKE_NO_RFCAL, WAKE_RF_DISABLED
}

void reboot() {
  ESP.reset();
}


bool setLed;
bool ledValue;


// --------------------------------------------------------------------------------------------------------------

void setup() {
  start_time = millis();

  // not used
  // pinMode(holdPin, OUTPUT);  // sets GPIO 0 to output
  // digitalWrite(holdPin, HIGH);  // sets GPIO 0 to high (this holds CH_PD high even if the PIR output goes low)


  Serial.begin(250000);
  Serial.printf("\n\nSDK version:%s\n\r", system_get_sdk_version());
  Serial.print("V 1.05 - ");
  Serial.print(deviceName);
  Serial.println(" - Booting...");

  // check RTC memory, should we run in Wifi mode (setup via WifiPortal, OTA) or normal MESH operations
  uint8_t mode = check_op_mode();
  Serial.print("Operation mode: ");
  Serial.println(mode);
  if ( mode == 7 || mode == 8 ) { wifiPortal(mode); }

  //pinMode(LED, OUTPUT);
  //pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  espNowFloodingMesh_secredkey(secredKey);
  espNowFloodingMesh_setAesInitializationVector(iv);
  espNowFloodingMesh_setToMasterRole(false, ttl);
  espNowFloodingMesh_setToBatteryNode(true);
  espNowFloodingMesh_begin(ESP_NOW_CHANNEL, bsid, false);

  espNowFloodingMesh_ErrorDebugCB([](int level, const char *str) {
    Serial.print(level); Serial.println(str); //If you want print some debug prints
  });

  while (!espNowFloodingMesh_syncTimeAnnonceAndWait((uint8_t *) deviceName, strlen(deviceName), 10,12,10)) {
    //Sync failed??? No connection to master????
    Serial.println("No connection to mesh, Sending again.");
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(3000);
    ESP.restart();
  }
  Serial.print(">After sync TS: "); Serial.println(millis() - start_time);

  boot_time = espNowFloodingMesh_getRTCTime();

  // Handle MQTT events from master. Do not call publish inside of call back.
  // --> Endless event loop and crash
  simpleMqtt.handleEvents([](const char *src_node, const char *msgid,
                             char command, const char *topic,
                             const char *value) {
    // Serial.print("CB: Topic: ");
    // Serial.println(topic);
    // Serial.print("CB: Value: ");
    // Serial.println(value);

    if (simpleMqtt.compareTopic(topic, deviceName, "/ota/set")) {
      strncpy(ota_command, value, MAX_OTA_COMMAND_LEN - 1);
      ota_command[MAX_OTA_COMMAND_LEN - 1] = '\0';
      new_ota_command = true;
      Serial.println("> GOT OTA MESSAGE");
      Serial.print("> TS: ");
      Serial.println(millis() - start_time);
      Serial.print("> msg: ");
      Serial.println(ota_command);
    }

  });

  simpleMqtt.subscribeTopic_sync("m", "/ota/set");
 
  vdd = ESP.getVcc();
  char b[32];
  snprintf(b, sizeof(b), "%u", vdd);

  Serial.println("Publishing....");
  simpleMqtt.publish("m", "/int/sensor/value", b);
 
  if (new_ota_command) {
    new_ota_command = false;
    process_ota_command(ota_command);
  }

  Serial.print("> before loop TS: "); Serial.println(millis() - start_time);

  //falltosleep();

}

// --------------------------------------------------------------------------------------------------------------------------

bool buttonStatechange = false;
bool we_are_done = false;

void loop() {
  espNowFloodingMesh_loop();

  const char *lost_msg = simpleMqtt.resend_loop();
  if (lost_msg != NULL) {
    Serial.print("LOST MSG: ");
    Serial.println(lost_msg);
  }

  if (new_ota_command) {
    new_ota_command = false;
    process_ota_command(ota_command);
  }

  uint16_t mc_used_slots = simpleMqtt.mc_get_used_slots();

  if (mc_used_slots == 0 && !we_are_done) {
    we_are_done = true;
    timer.setTimeout(40, falltosleep);
  }

  timer.run();
}


uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

// prints all rtcData, including the leading crc32
void printMemory() {
  char buf[3];
  uint8_t *ptr = (uint8_t *)&rtcData;
  for (size_t i = 0; i < sizeof(rtcData); i++) {
    sprintf(buf, "%02X", ptr[i]);
    Serial.print(buf);
    if ((i + 1) % 32 == 0) {
      Serial.println();
    } else {
      Serial.print(" ");
    }
  }
  Serial.println();
}
