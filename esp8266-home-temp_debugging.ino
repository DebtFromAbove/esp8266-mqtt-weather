/* DHTServer - ESP8266 Webserver with a DHT sensor as an input

   Based on ESP8266Webserver, DHTexample, and BlinkWithoutDelay (thank you)

   Version 1.0  5/3/2014  Version 1.0   Mike Barela for Adafruit Industries
   Version 0.12 04/03/2018  updated to DHT library 1.3.0
   Version 0.13 04/03/2018  Added some debugging string improvements, added pause before DHT initialisation
   Version 0.14 04/03/2018  Increased delay beofre DHT initialisation
   Version 0.15 04/03/2018  Turned on DHT debugging
   Version 0.16 06/03/2018  Changed circuit to switch DHT22 VCC on/off using GPIO
   Version 0.17 06/03/2018  Changed dest IP to the attic server
   Version 0.18 16/03/2018  Doubled delay pre-DHT22 read to see if it improves stability - It did
   Version 0.19 21/03/2018  Disabling wifi on start, reading sensor, enabling wifi then sending, hopefully will improve stability
                              - It did fix the problem
   Version 0.20 25/03/2018  Adding Reset_reason logging and loop count
   Version 0.30 12/04/2018  Major code overhaul, modular functions etc.
   Version 0.31 24/05/2018  Added a logging function.
   Version 0.32 05/06/2018  Added more MQTT client connect debugging, improved + moved out logging to a new library.
   Version 0.33 13/06/2018  rewrote and improved the logging library
   Version 0.34 15/06/2018  Added exponential filtering method and conditional handling for temperature, humidity and voltage
   Version 0.34 16/06/2018  Tidied up code, added logging of failed initialisation states (6630 -7130 milliseconds execution time)
   Version 0.35 16/06/2018  Tried to make MQTT publoshing more reliable, still not perfect. Added static IP (needs some cleanup)
      (6173 -6260 milliseconds execution time) Staic IP seems to reduce time by at least 500ms
   Version 0.36 23/06/2018  Added paramaterized wificonnect static IP function, added JSON MQTT publishing

*/

const String VER = "0.36";

/*
   TODO: Add wifiManager code to allow programming on boot
   TODO: Add OTA update?
   TODO: JSONify all metrics into a single MQTT queue
*/

#include <ESP8266WiFi.h>
#include <DHT.h>
#include <stdio.h>
#include <stdlib.h>
#include <PubSubClient.h>
#include "ESPLogging.h"
#include "ArduinoJson.h"

#define DHTTYPE DHT22
const int DHTPIN = 4;
const int DHTPWRPIN = 5;
const int DHTDELAYMILLIS = 2000;
const int LEDPIN = 2;
const int MAXATTEMPTS = 25;
const int MAXMQTTATTEMPTS = 3;
const int RTCMEMOFFSET = 0;
const float EXPFILTERWEIGHT = 0.8;
const uint8_t MAXHANGUPATTEMPTS = 10;

const uint8_t MAXLOGS = 30;
const uint8_t LOGLEVEL = 4; // 0-disabled, 1-error, 2-warn, 3-info, 4-debug

extern "C" {
  #include "user_interface.h"
  extern struct rst_info resetInfo;
}

uint32_t computedCRC32;
uint32_t newCRC32;

typedef struct {
  uint32_t crc32;
  uint32_t loopCount;
  float lastExpFilteredTemp;
  float lastExpFilteredHum;
  float lastExpFilteredVolt;
} rtcStore;

rtcStore rtcMem;

const char* mySSID     = "";
const char* myPassword = "";
const char* sensorName = "temp-humidity";
String clientName = "tempHum-";
const String mqttServer = "192.168.1.222";
const String mqttUser = "guest";
const String mqttPassword = "guest";
const int MQTTPORT = 1883;
uint8_t mac[6];
uint8_t l, loopCount;

IPAddress myIp(192, 168, 1, 18);
IPAddress myGateway(192, 168, 1, 1);
IPAddress mySubnet(255, 255, 255, 0);

// MQTT topics to publish data to
const char* metricsTopic = "openhab/tempsensor/metrics";
const char* logsTopic = "openhab/tempsensor/logs";

char *RstReason[] = {
  "REASON_DEFAULT_RST",
  "REASON_WDT_RST",
  "REASON_EXCEPTION_RST",
  "REASON_SOFT_WDT_RST",
  "REASON_SOFT_RESTART",
  "REASON_DEEP_SLEEP_AWAKE",
  "REASON_EXT_SYS_RST"
};

unsigned long start_time;
unsigned long end_time;
String logMessages[MAXLOGS];
unsigned int raw=0;
float voltage, expVoltage;
float expHumidity, expTempC;
// Variables required for the getTemperature() function
float humidity, temp_c;  // Values read from sensor
bool keepGoing = true;  // Allows for break point after initial startup checks
// Time to sleep (in seconds):
const int sleepTimeS = 30;
// Amount of time to wait before sleeping (helps when re-programming),
// TODO remove for production
const int sleepDelay = 10;

StaticJsonBuffer<200> jsonMetricsBuffer;
StaticJsonBuffer<2000> jsonLogsBuffer;

// Initialize DHT sensor
// NOTE: For working with a faster than ATmega328p 16 MHz Arduino chip, like an ESP8266,
// you need to increase the threshold for cycle counts considered a 1 or 0.
// You can do this by passing a 3rd parameter for this threshold.  It's a bit
// of fiddling to find the right value, but in general the faster the CPU the
// higher the value.  The default for a 16mhz AVR is a value of 6.  For an
// Arduino Due that runs at 84mhz a value of 30 works.
// This is for the ESP8266 processor on ESP-01
DHT dht(DHTPIN, DHTTYPE, 11); // 11 works fine for ESP8266

// A TCP WiFi client to send packets via MQTT
WiFiClient espWiFiClient;
// Declare a new (global visible) logging client
ESPLogging logger;
// An MQTT client instance
PubSubClient MQTTClient(espWiFiClient);

/* NOTE: Removing these as we are always ? 2s between reads due to deep sleep
// Generally, you should use "unsigned long" for variables that hold time
unsigned long previousMillis = 0;        // will store last temp was read
const long interval = 2000;              // interval at which to read sensor
*/

// Function declarations
void getTemperature();
float getVoltage();
uint32_t calculateCRC32(uint8_t *data, size_t length);
void flashLed(int interval, int pin, int n);
void wifiOff();
void wifiOn();
bool wifiConnect(const char* ssid, const char* passwd, IPAddress ip,
  IPAddress gateway, IPAddress subnet,  const int attempts);
bool initialiseRTCMemory(int memOffset, int lengthInBytes);
int getBootDevice(void);
String macToStr(const uint8_t* mac);
bool reconnect(PubSubClient thisClient, const char* clientID, const int attempts);
void printMemory(uint8_t *data, size_t length);
float expFilteredResult(float rawValue, float lastExp, float weight);

void setup(void)
{

  start_time = millis();
  Serial.begin(115200);
  logger.begin(LOGLEVEL, MAXLOGS, &Serial);
  logger.info("Sketch version: "+VER);

  wifiOff();

  // First boot after UART flashing, ESP will hang on reboot unless you
  // actually press the RST button or ground the RST pin
  // We should handle this somehow more than just logging it maybe?
  if ( getBootDevice() == 1 ) {

    if (initialiseRTCMemory(RTCMEMOFFSET, sizeof(uint32_t)+sizeof(rtcMem))) {
      logger.debug("First poweron after UART programming, initializing RTC memory");
    }
    else {
      logger.error("First poweron after UART programming, initializing RTC memory FAILED!");
    }
  }

  /* Enum values from the espressif user_interface.h library

  enum rst_reason {
    REASON_DEFAULT_RST      = 0,    /* normal startup by power on
    REASON_WDT_RST          = 1,    /* hardware watch dog reset
    REASON_EXCEPTION_RST    = 2,    /* exception reset, GPIO status won’t change
    REASON_SOFT_WDT_RST     = 3,    /* software watch dog reset, GPIO status won’t change
    REASON_SOFT_RESTART     = 4,    /* software restart ,system_restart , GPIO status won’t change
    REASON_DEEP_SLEEP_AWAKE = 5,    /* wake up from deep-sleep
    REASON_EXT_SYS_RST      = 6     /* external system reset
  };
  */

  rst_info *rsti;
  rsti = ESP.getResetInfoPtr();
  logger.debug(String("ResetInfo.reason = ") + rsti->reason + " = " + RstReason[rsti->reason]);

  if (rsti->reason == REASON_DEFAULT_RST) {
    // Normal power on, initialise RTC memory and continue
    logger.debug("Reset reason was power on device");
    if (initialiseRTCMemory(RTCMEMOFFSET, sizeof(rtcMem))) {
      logger.debug("RTC memory initialised");

      Serial.println("DEBUG: new memory is: ");
      printMemory((uint8_t*) &rtcMem, sizeof(rtcMem));

      if (! ESP.rtcUserMemoryRead(RTCMEMOFFSET, (uint32_t*) &rtcMem, sizeof(rtcMem)))  {
        logger.error("Failed to return initialised RTC memory into data struct "
          "(after normal power on), restarting");
        logger.stopLogging();
        keepGoing = false;
      }
      else {
        logger.debug("newly initialised loop count is: " + String(rtcMem.loopCount));
      }
    }
    else {
      logger.error("Initializing RTC memory FAILED! (after normal power on), restarting");
      logger.stopLogging();
      keepGoing = false;
    }

  }
  else if (rsti->reason == REASON_EXT_SYS_RST || rsti->reason == REASON_DEEP_SLEEP_AWAKE ){
    // Deep sleep wake or reset button was pushed. RTC memory should survive this
    if(! ESP.rtcUserMemoryRead(RTCMEMOFFSET, (uint32_t*) &rtcMem, sizeof(rtcMem))) {
      logger.error("Failed to retrieve CRC32 from memory");
      logger.stopLogging();
      keepGoing = false;
    }
    logger.debug("Retrieved stored memory structure, performing CRC check");
    logger.debug("Retrieved CRC32 is: " + String(rtcMem.crc32, HEX));

    Serial.println("DEBUG: retrieved memory is: ");
    printMemory((uint8_t*) &rtcMem, sizeof(rtcMem));

    computedCRC32 = calculateCRC32((uint8_t*) &rtcMem.loopCount, sizeof(rtcMem)-4);
    logger.debug("Newly computed CRC32 is: " + String(computedCRC32, HEX));
    if (computedCRC32 != rtcMem.crc32) {
      logger.warn("Computed CRC32 does not match stored value, reinitialising");
      logger.stopLogging();
      keepGoing = false;
    }
    else {
      logger.debug("Loop Count is: " + String(rtcMem.loopCount));
    }
  }
  else {
    // Reset from previous failed attempt or something went wrong (watchdog etc.)
    logger.debug("Error handling / restart case");
    logger.debug(String("ResetInfo.reason = ") + rsti->reason);

    if (initialiseRTCMemory(RTCMEMOFFSET, sizeof(rtcMem))) {
      logger.debug("RTC memory initialised");

      Serial.println("DEBUG: new memory is: ");
      printMemory((uint8_t*) &rtcMem, sizeof(rtcMem));

      if(! ESP.rtcUserMemoryRead(RTCMEMOFFSET, (uint32_t*) &rtcMem, sizeof(rtcMem))) {
        logger.error("Failed to return initialised RTC memory into data struct, "
          "(after error reset startup) restarting");
          logger.stopLogging();
          keepGoing = false;
      }
      else {
        logger.debug("Reset case: newly initialised loop count is: " + String(rtcMem.loopCount));
        logger.debug("Newly initialised CRC32 is: " + String(rtcMem.loopCount, HEX));
      }
    }
    else {
      logger.error("Initializing RTC memory FAILED! (after error reset startup), restarting");
      logger.stopLogging();
      keepGoing = false;
    }
  }

  // Handle the case where we should stop at this point and restart
  if (! keepGoing ) {
    logger.startLogging();
    wifiOn();
    MQTTClient.setServer(mqttServer.c_str(), MQTTPORT);

    WiFi.macAddress(mac);
    clientName += macToStr(mac);

    // We start by connecting to a WiFi network
    if (! wifiConnect(mySSID, myPassword, myIp, myGateway, mySubnet, MAXATTEMPTS)) {
      logger.warn("Connecting to WiFi failed");
    }

    if (! MQTTClient.connected()) {
      reconnect(MQTTClient, clientName.c_str(), MAXMQTTATTEMPTS);
    }

    logger.debug("We have " + String(logger.numLogs()) + " logs to send, not counting this one.");
    l = logger.getLogs(logMessages);

    end_time = millis();
    unsigned long executionTime = end_time - start_time;

    JsonObject& logsRoot = jsonLogsBuffer.createObject();
    logsRoot["sensor"] = sensorName;
    logsRoot["client"] = clientName;
    logsRoot["executionTime"] = executionTime;
    logsRoot["lastResetCode"] = rsti->reason;
    JsonArray& msgs = logsRoot.createNestedArray("msgs");
    for (uint8_t i = 0; i<l; i++) {
      msgs.add(logMessages[i]);
    }

    char serialisedLogsJson[logsRoot.measureLength()+1];
    logsRoot.printTo(serialisedLogsJson, logsRoot.measureLength());

    if (! MQTTClient.publish(logsTopic, serialisedLogsJson , true)) {
        logger.error("Failed to publish to logs topic.");
    }
    delay(5);

    MQTTClient.disconnect();
    delay(10);
    ESP.restart();
  }

  // Setup LED pin to be an OUTPUT
  pinMode(LEDPIN, OUTPUT);
  // Set DHT22 pwr pin output + low to disale dht22
  pinMode(DHTPWRPIN, OUTPUT);
  digitalWrite(DHTPWRPIN, LOW);

  // TODO: remove before production
  flashLed(300, LEDPIN, 2);

  logger.debug("Pause some time (currently "+String(DHTDELAYMILLIS)+" milliseconds defined in constant) before initialising DHT22");
  digitalWrite(DHTPWRPIN, HIGH);
  delay(DHTDELAYMILLIS);          // Allow DHT to stabilise for x milliseconds before reading
  dht.begin();           // initialize temperature sensor

  logger.debug("Reading temperature and humidity");
  getTemperature();
  logger.debug("Temp is: " + String(temp_c));
  logger.debug("Reading battery voltage");
  voltage = getVoltage();
  logger.debug("Voltage is: " + String(voltage));

  // All other values below this enum indicate some error that would mean
  // rtcMem is reset to default - settin rtc historic smoothed values to current values
  if (rsti->reason < REASON_DEEP_SLEEP_AWAKE) {
    rtcMem.lastExpFilteredTemp = temp_c;
    rtcMem.lastExpFilteredHum = humidity;
    rtcMem.lastExpFilteredVolt = voltage;
    logger.debug("We had a poweron/error reset startup, setting stored "
      "temp,hum,volt values to ones just read.");
  }

  // Calculate and return smoothed values for temperature, humidity and voltage
  if( temp_c < 0.1 || rtcMem.lastExpFilteredTemp < 0.1 || humidity < 0.1
      || rtcMem.lastExpFilteredHum < 0.1 || voltage < 0.1
      || rtcMem.lastExpFilteredVolt < 0.1 ) {

    logger.warn("Current or stored voltage, humidity or temperature filtered value "
      "was < 0.1. Something didn't return a good result.");
  }
  else {
    expTempC = expFilteredResult(temp_c, rtcMem.lastExpFilteredTemp, EXPFILTERWEIGHT);
    expHumidity = expFilteredResult(humidity, rtcMem.lastExpFilteredHum, EXPFILTERWEIGHT);
    expVoltage = expFilteredResult(voltage, rtcMem.lastExpFilteredVolt, EXPFILTERWEIGHT);
  }

  wifiOn();
  MQTTClient.setServer(mqttServer.c_str(), MQTTPORT);

  WiFi.macAddress(mac);
  clientName += macToStr(mac);

  // We start by connecting to a WiFi network
  if (! wifiConnect(mySSID, myPassword, myIp, myGateway, mySubnet, MAXATTEMPTS)) {
    logger.warn("Connecting to WiFi failed");
  }

  if (! MQTTClient.connected()) {
    reconnect(MQTTClient, clientName.c_str(), MAXMQTTATTEMPTS);
  }

  JsonObject& metricsRoot = jsonMetricsBuffer.createObject();
  metricsRoot["sensor"] = sensorName;
  metricsRoot["client"] = clientName;
  metricsRoot["temp"] = temp_c;
  metricsRoot["expTemp"] = expTempC;
  metricsRoot["humidity"] = humidity;
  metricsRoot["expHumidity"] = expHumidity;
  metricsRoot["voltage"] = voltage;
  metricsRoot["expVoltage"] = expVoltage;

  char serialisedMetricsJson[metricsRoot.measureLength()+1];
  metricsRoot.printTo(serialisedMetricsJson, metricsRoot.measureLength());

  if (! MQTTClient.publish(metricsTopic, serialisedMetricsJson, true)) {
    logger.error("Failed to publish to logs topic.");
  }
  delay(5);

  // INcrement loop counter, update fields and write data back to Memory
  rtcMem.loopCount += 1;
  rtcMem.lastExpFilteredTemp = expTempC;
  rtcMem.lastExpFilteredHum = expHumidity;
  rtcMem.lastExpFilteredVolt = expVoltage;
  rtcMem.crc32 = calculateCRC32((uint8_t*) &rtcMem.loopCount, sizeof(rtcMem)-4);
  logger.debug("New CRC32 value is: " + String(rtcMem.crc32, HEX));

  if (ESP.rtcUserMemoryWrite(RTCMEMOFFSET, (uint32_t*) &rtcMem, sizeof(rtcMem))) {
    logger.debug("Wrote new values back to RTC Memory");
  }
  else {
    logger.error("Failed to write new values back to RTC Memory");
    keepGoing = false;
  }

  end_time = millis();
  unsigned long executionTime = end_time - start_time;
  // record current time since start as last step before putting data into transmit string
  logger.debug("Execution tim is " + String(end_time - start_time) + " milliseconds.");

  logger.debug("We have " + String(logger.numLogs()) + " logs to send, not counting this one.");
  l = logger.getLogs(logMessages);

  JsonObject& logsRoot = jsonLogsBuffer.createObject();
  logsRoot["sensor"] = sensorName;
  logsRoot["client"] = clientName;
  logsRoot["executionTime"] = executionTime;
  logsRoot["loopCount"] = rtcMem.loopCount -1;
  logsRoot["lastResetCode"] = rsti->reason;
  JsonArray& msgs = logsRoot.createNestedArray("msgs");
  for (uint8_t i = 0; i<l; i++) {
    msgs.add(logMessages[i]);
  }

  char serialisedLogsJson[logsRoot.measureLength()+1];
  logsRoot.printTo(serialisedLogsJson, logsRoot.measureLength());

  if (! MQTTClient.publish(logsTopic, serialisedLogsJson , true)) {
      logger.error("Failed to publish to logs topic.");
  }
  delay(5);

  // In case the write-back to rtcMemory failed, restart after we've hopefully sent logs
  if (! keepGoing) ESP.restart();

  // Hangup MQTT connection and Wifi but wait until it closes properly to ensure
  // all messages get sent
  MQTTClient.disconnect();
  loopCount=0;
  while (MQTTClient.connected()) {
    delay(10);
    loopCount += 1;

    if(loopCount > MAXHANGUPATTEMPTS) {
      logger.warn("Timeout waiting for mqtt and wifi to disconnect");
      break;
    }
  }
  logger.debug("It took " + String(loopCount) + " loops x 10ms to hangup on MQTT and WiFi connection.");
  delay(10);

  // TODO: Remove this for production
  logger.debug("waiting "+String(sleepDelay)+" seconds before going to sleep");
  delay(sleepDelay * 1000);

  logger.debug("Setting DHT pin output/low just before deep sleep");
  pinMode(DHTPIN, OUTPUT);
  digitalWrite(DHTPIN, LOW);
  digitalWrite(DHTPWRPIN, LOW);

  ESP.deepSleep(sleepTimeS * 1000000);
}

void loop(void)
{
}

// flash onboard LED pin on, then off
void flashLed(int interval, int pin, int n) {
  for (int i=0; i<n; i++) {
    delay(interval);              // wait for n seconds
    digitalWrite(pin, LOW);   // turn the LED on (HIGH is the voltage level)
    delay(interval);              // wait for n seconds
    digitalWrite(pin, HIGH);    // turn the LED off by making the voltage LOW
  }
}

// return DHT temperature and humidity, only reads once as can't do it more
// than once per two seconds.
void getTemperature() {
  // Wait at least 2 seconds seconds between measurements.
  // if the difference between the current time and last time you read
  // the sensor is bigger than the interval you set, read the sensor
  // Works better than delay for things happening elsewhere also

  /* Removing this check as we are using deep sleep between reads
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // save the last time you read the sensor
    previousMillis = currentMillis;
  */

    // Reading temperature for humidity takes about 250 milliseconds!
    // Sensor readings may also be up to 2 seconds 'old' (it's a very slow sensor)
    humidity = dht.readHumidity();          // Read humidity (percent)
    temp_c = dht.readTemperature();     // Read temperature as Fahrenheit
    // Check if any reads failed and exit early (to try again).
    if (isnan(humidity) || isnan(temp_c)) {
      logger.warn("Failed to read from DHT sensor!");
      return;
    }
  // }
}

// Compares two floats (for voltage readings) and returns 0, -1, +1 for equal,
// less than or more than. Used in qsort function
int comparator(const void * a, const void * b) {
  float f1 = *(const float *)a;
  float f2 = *(const float *)b;

  float precision = 0.01;
  if (((f1 - precision) < f2) &&
      ((f1 + precision) > f2))
  {
    // Floats are equal
    return 0;
  }
  else
  {
    // Floats are inequal
    if (f1 < f2) {
        return -1;
    }
    else {
      return 1;
    }
  }
}

// return voltage, averaged over a number of reading
// (as can be done really quickly)
float getVoltage() {
  int readings = 5;
  float values[readings];
  unsigned int raw;
  float v, volt;

  for (int i=0; i<readings; i++) {
    raw = analogRead(A0);
    delay(2); // Wait for 2ms to make sure analog read is stable next time around
    v=(float)raw/1023.0;
    v=v*4.2;
    if isnan(volt) {
      logger.warn("getVoltage - Failed to read analog pin!");
      v = 0.0;
    }
    values[i] = v;
  }

  qsort (values, readings, sizeof(values[0]), comparator);
  return values[readings/2];
}

// Calculates a CRC32 value over a set of data, used for integrity
// checking the stored values in RTC memory
uint32_t calculateCRC32(uint8_t *data, size_t length) {
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

// Turn off wifi to save energy and make more stable DHT readings
void wifiOff() {
  WiFi.mode( WIFI_OFF );
  WiFi.forceSleepBegin();
  delay( 1 );
}

// Re-enable wifi after pause, to allow connection to send readings
void wifiOn() {
  WiFi.forceSleepWake();
  delay( 1 );
  //  Bring up the WiFi connection
  WiFi.mode( WIFI_STA );
}

bool wifiConnect(const char* ssid, const char* passwd, IPAddress ip, IPAddress gateway, IPAddress subnet, const int attempts) {
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(ssid, passwd);
  for (int i=0; i<attempts; i++) {
    if (WiFi.status() != WL_CONNECTED) {
      delay(250);
      Serial.print(".");
    }
    else {
      logger.debug("WiFi connected");
      logger.info("IP address: " + WiFi.localIP().toString());
      return true;
    }
  }
  // Exited for loop so connect must have FAILED
  return false;
}

bool initialiseRTCMemory(int memOffset, int lengthInBytes) {
  uint32_t initialiseValue = 0;
  int dataSize = sizeof(initialiseValue); // Should be 4 bytes
  int buckets = (lengthInBytes / 4);
  if (buckets == 0) buckets = 1;
  for (int i=0; i<buckets; i++) {
      // write 4 bytes with the initialise value of 0 (from a 32bit integer)
      if (! ESP.rtcUserMemoryWrite(memOffset + i, &initialiseValue, dataSize)) {
        return false;
      }
  }
  return true;
}

int getBootDevice(void) {
  int bootmode;
  asm (
    "movi %0, 0x60000200\n\t"
    "l32i %0, %0, 0x118\n\t"
    : "+r" (bootmode) /* Output */
    : /* Inputs (none) */
    : "memory" /* Clobbered */
  );
  return ((bootmode >> 0x10) & 0x7);
}

String macToStr(const uint8_t* mac) {
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

bool reconnect(PubSubClient thisClient, const char* clientID, const int attempts) {
  logger.debug("Attempting to connect to MQTT broker...");
  // Loop until we're reconnected
  for (int i=0; i<attempts; i++) {
    if (!thisClient.connected()) {

      logger.debug("MQTT client not yet connected to broker.");
      // Attempt to connect
      // If you do not want to use a username and password, change next line to
      // if (client.connect("ESP8266Client")) {
      if (thisClient.connect(clientID)) {
        logger.debug("MQTT client connected");
        return true;
      }
      else {
        logger.warn("MQTT client failed to connect on attempt " + String(i+1) + ", rc=" + String(thisClient.state()));
        logger.debug(" try again in 2 seconds");
        // Wait 2 seconds before retrying
        delay(2000);
      }
    }
  }
}

// Debugging function to print contents of RTC memory
void printMemory(uint8_t *data, size_t length) {
  char buf[3];
  uint8_t *ptr = data;
  for (size_t i = 0; i < length; i++) {
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

float expFilteredResult(float rawValue, float lastExp, float weight) {
  // yn = w × xn + (1 – w) × yn – 1
  float newExpFilteredValue = (weight * rawValue) + (( 1.0 - weight ) * lastExp);
  return newExpFilteredValue;
}
