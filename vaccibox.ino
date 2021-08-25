#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Arduino_JSON.h>
#include "SIM800L.h"
#include <EEPROM.h>


//gps
const int gpsRxPin = 3;
const int gpsTxPin = 4;

int accuracy = 0;
byte Y, M, D, H, MN, S;

//gsm
const int gsmRxPin = 8;
const int gsmTxPin = 7;
const int gsmRstPin = 6;

//status rgb
const int R = 11;
const int G = 12;
const int B = A4;

//buzzer
//const int buzzer = 91;

//led
const int led = 9;

//rollerSwitch
const int rollerSwitch = A1;

//relay
const int relay = 10;

//bat level
const int batLevel = A5;

//##configs##

//hardcoded boxid
const char* boxId = "cc79a9b7";

//api
const char apn[] = "";
const char readingsUrl[] = "http://167.86.75.239:9123/api/v1/readings";
const char settingsUrl[] = "http://167.86.75.239:9123/api/v1/settings";
const char contentType[] = "application/json";

//settings
int sendIntervalMs = 10000; //
int lowTemp = 2;
int highTemp = 10;
int lowBattPercentage = 20;

//ds18b20
const int oneWireBus = 13;
OneWire oneWire(oneWireBus);
DallasTemperature tempSensor(&oneWire);

//gps
TinyGPSPlus gps;
SoftwareSerial gpsSerial(gpsRxPin, gpsTxPin);
//sim 800
SIM800L* sim800l;

//current eeprom address
int addr = 0;

void setup()
{
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Serial.println("Initializing...");
  pinMode(rollerSwitch, OUTPUT);
  pinMode(relay, OUTPUT);
  ///m pinMode(buzzer, OUTPUT);
  ///m pinMode(R, OUTPUT);
  ///m pinMode(G, OUTPUT);
  ///m pinMode(B, OUTPUT);

  pinMode(led, OUTPUT);
  
  //initialise gsm software serial
  SoftwareSerial* gsmSerial = new SoftwareSerial(gsmRxPin, gsmTxPin);
  gsmSerial->begin(9600);
  delay(1000);
  // Initialize SIM800L driver with an internal buffer of 200 bytes and a reception buffer of 512 bytes, debug disabled
  sim800l = new SIM800L((Stream *)gsmSerial, gsmRstPin, 100, 256, (Stream *)&Serial);
  //setup gsm module
  setupModule();
}

void loop()
{

  sim800l->setPowerMode(NORMAL);
  getSettings();
  
  //get sensor readings and send
  //int batVal = getBatLevel();
  //int switchVal = getRollerState();
  float tempVal = getcurrTemperature();
  float latitude = getLat();
  float longitude = getLon();
  String timeStamp = getTimestamp();
  
  sendData(0,longitude,latitude,0,accuracy,timeStamp,tempVal);
  
  smartDelay(5000);

  // delay(5000);

  // sim800l->setPowerMode(SLEEP);

}

// This custom version of delay() ensures that the tinyGPS object
// is being "fed". From the TinyGPS++ examples.
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    // If data has come in from the GPS module
    while (gpsSerial.available()) 
      gps.encode(gpsSerial.read()); // Send it to the encode function
    // tinyGPS.encode(char) continues to "load" the tinGPS object with new
    // data coming in from the GPS module. As full NMEA strings begin to come in
    // the tinyGPS library will be able to start parsing them for pertinent info
  } while (millis() - start < ms);
}


float getcurrTemperature() {
  tempSensor.requestTemperatures();
  float temperatureC = tempSensor.getTempCByIndex(0);
  return temperatureC;
}

int getRollerState() {
  int state = digitalRead(rollerSwitch);
  return state;
}

float getLat() {
  if (gps.encode(gpsSerial.read())) {
    if (gps.location.isValid())
    {
      accuracy = 1;
      //dtostrf(gps.location.lat(), 10, 7, latitude);
      float latitude = gps.location.lat();
      return latitude;
    }
    else {
      accuracy = 0;
      float latitude = 0;
      return latitude;
    }
  }
}

float getLon() {
  if (gps.encode(gpsSerial.read())) {
    if (gps.location.isValid())
    {
      accuracy = 1;
      float longitude = gps.location.lng();
      return longitude;
    }
    else {
      accuracy = 0;
      float longitude = 0;
      return longitude;
    }
  }
}

String getTimestamp() {
  if (gps.time.isValid())
  {
    H = gps.time.hour();
    MN = gps.time.minute();
    S = gps.time.second();

  }
  else {
    H = "00";
    MN = "00";
    S = "00";

  }
  if (gps.date.isValid())
  {
    Y = gps.date.year();
    M = gps.date.month();
    D = gps.date.day();

  }
  else
  {
    Y = 0;
    M = 0;
    D = 0;
  }
  //the full timestamp
  String timeStamp = String(Y) + String(M) + String(D) + String(H) + String(MN) + String(S);
  return timeStamp;
}

void switchRelay(int state) {
  digitalWrite(relay, state);
}

int getBatLevel() {
  int maxVal = 800;
  int level = analogRead(batLevel);
  int batLevel = map(level, 0, maxVal, 0, 100);
  return batLevel;
}



//void testBeep(int pulses) {
//  for (int i = 0; i < pulses; i++) {
//    tone(buzzer, 400, 500);
//    noTone(buzzer);
//  }
//}

void blink(int pulses, int interval) {
  for ( int i =0; i < pulses; i++) {
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(interval);                       // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      delay(interval);
  }
      
}



void setupModule() {
  // Wait until the module is ready to accept AT commands
  while (!sim800l->isReady()) {
    delay(1000);
    blink(2, 200);
  }

  // Wait for the GSM signal
  uint8_t signal = sim800l->getSignal();
  while (signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
    blink(1, 200);
  }
  delay(1000);

  // Wait for operator network registration (national or roaming network)
  NetworkRegistration network = sim800l->getRegistrationStatus();
  while (network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
    delay(1000);
    network = sim800l->getRegistrationStatus();
    blink(3, 200);
  }
  delay(1000);

  // Setup APN for GPRS configuration
  bool success = sim800l->setupGPRS(apn);
  while (!success) {
    success = sim800l->setupGPRS(apn);
    delay(5000);
    blink(4, 200);
  }
  Serial.println(F("GPRS config OK"));
}


void sendData(int batVal, float latitude,float longitude,int switchVal,int accuracy,String timeStamp,float tempVal) {
  
  String data ="{\"box_id\":\"" + String(boxId) +"\",\"lat\":\""+String(latitude)+"\",\"long\":\""+String(longitude) + "\",\"accuracy\":\""+String(accuracy)+"\",\"timestamp\":\""+timeStamp+"\",\"temp\":\""+String(tempVal)+"\"}";
  
  const char* payload = data.c_str();

  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }

  // Check if connected, if not reset the module and setup the config again
  if (!connected) {
    sim800l->reset();
    setupModule();
    return;
  }

  Serial.println(payload);
  // Do HTTP POST communication with 10s for the timeout (read and write)
  uint16_t rc = sim800l->doPost(readingsUrl, contentType, payload, 10000, 10000);

  // Close GPRS connectivity (5 trials)
  connected = sim800l->disconnectGPRS();
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->disconnectGPRS();
  }  

  // End of program... wait...
  // while (1);
}

void getSettings() {
  // Establish GPRS connectivity (5 trials)
  bool connected = false;
  for (uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }


  // Check if connected, if not reset the module and setup the config again
  if (!connected) {
    sim800l->reset();
    setupModule();
    return;
  }

  // Do HTTP GET communication with 10s for the timeout (read)
  uint16_t rc = sim800l->doGet(settingsUrl, 10000);
  if (rc == 200) {
    // Success, store and output the data received on the serial
    String settingsPayload = sim800l->getDataReceived();

    JSONVar settingsObject = JSON.parse(settingsPayload);
    //settings
    JSONVar sendIntervalMs = settingsObject["temp_interval_ms"];
    JSONVar highTemp = settingsObject["low_temp_celsius"];
    JSONVar lowTemp = settingsObject["high_temp_celsius"];
    JSONVar lowBattPercentage = settingsObject["low_batt_percentage"];

  } else {
    // Failed...
    Serial.println(rc);
  }
  // Close GPRS connectivity (5 trials)
  bool disconnected = sim800l->disconnectGPRS();
  for(uint8_t i = 0; i < 5 && !disconnected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }

}
