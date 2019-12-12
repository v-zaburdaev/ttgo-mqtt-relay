#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>

/**************************************************************
 *
 * This sketch connects to a website and downloads a page.
 * It can be used to perform HTTP/RESTful API calls.
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 **************************************************************/

// Your GPRS credentials (leave empty, if missing)
const char apn[]      = "internet.mts.ru"; // Your APN
const char gprsUser[] = ""; // User
const char gprsPass[] = ""; // Password
const char simPIN[]   = ""; // SIM card PIN code, if any

// TTGO T-Call pin definitions
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
#define DS18B20_Pin          14
#define HEATER_Pin           15
#define LED                  13
//#define SENSOR_VP_GPIO       36
//#define SENSOR_VN_GPIO       39 // LED3
#define BATT_LVL             35

  struct {
    char tempBufferData[8]="";
    char t[6]="";
  } temperature[4];


// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to the module)
#define SerialAT  Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define TINY_GSM_DEBUG SerialMon
//#define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include "utilities.h"


// GPIO where the DS18B20 is connected to
const int oneWireBus = DS18B20_Pin;     

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
int numberOfDevices = 0;
DeviceAddress tempDeviceAddress; 

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

volatile int interruptCounter;

int totalInterruptCounter;  
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
 
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
 
}

void timerInit(){
  
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000, true);
  timerAlarmEnable(timer);
  
  }

TinyGsmClient client(modem);
PubSubClient mqtt(client);

// MQTT details
//const char* broker = "tailor.cloudmqtt.com";
//const char* mqttuser = "iobfumxq";
//const char* mqttpass = "Fxzp7UhINe38";
//int mqttport = 14605;

const char* broker = "95.79.40.140";
const char* mqttuser = "garage";
const char* mqttpass = "123345567";
int mqttport = 1883;


//const char* topicLed = "GsmClientTest/led";
//const char* topicInit = "GsmClientTest/init";
//const char* topicLedStatus = "GsmClientTest/ledStatus";

bool mqttSend=false;

const int  port = 80;
int mode=0;
int minute=60;
int hour=60;
int uptime=0;
int ledStatus = LOW;
int8_t batteryLvl=0;
int batteryAdcLvl=0;
long lastReconnectAttempt = 0;

const char* refreshTopic = "refresh/all";
const char* heaterGetTopic = "heater/getEna";
const char* heaterSetTopic = "heater/setEna";
const char* heaterTimerTopic = "heater/getTimer";
const char* heaterSetTimerTopic = "heater/setTimer";
const char* refreshTopicAll = "refresh/all";
int heaterStatus=0;
int heaterTimer=0;
int heaterTimerDefault=60;
int restTime=60;
int restTimeDefault=60;



void modemInit(){
    // Set-up modem reset, enable, power pins
    pinMode(MODEM_PWKEY, OUTPUT);
    pinMode(MODEM_RST, OUTPUT);
    pinMode(MODEM_POWER_ON, OUTPUT);

    digitalWrite(MODEM_PWKEY, LOW);
    digitalWrite(MODEM_RST, HIGH);
    digitalWrite(MODEM_POWER_ON, HIGH);
  
    // Set GSM module baud rate and UART pins
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(3000);
  
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    SerialMon.println("Initializing modem...");
    modem.restart();
    // Or, use modem.init() if you don't need the complete restart
  
    String modemInfo = modem.getModemInfo();
    SerialMon.print("Modem: ");
    SerialMon.println(modemInfo);
  
    // Unlock your SIM card with a PIN if needed
    if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
      modem.simUnlock(simPIN);
    }
    mode=1;
  }

void mqttCallback(char* topic, unsigned char* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();
  String p="";
  if(len>0){
    for(int i=0;i<=len-1;i++){
        p=p+String((char)payload[i]);
        
      }
    }
  
  
  // Only proceed if incoming message's topic matches
//  if (String(topic) == topicLed) {
//    ledStatus = !ledStatus;
//  }
  if (String(topic) == heaterSetTopic) {
    if(p=="1"){
        SerialMon.println("startHeater");
        startHeater();
      } else {
        SerialMon.println("STOPHeater");
        stopHeater();
       }
       mqttPubAll();
  }
  if (String(topic) == heaterSetTimerTopic) {
    if(heaterStatus==0) {
        heaterTimerDefault=p.toInt();
      } else {
        heaterTimer=p.toInt();
      }
      mqttPubAll();
  }
  if (String(topic) == refreshTopic){
      SerialMon.println("refresh");
      mqttPubAll();
    }
  
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);
  
  mqtt.setServer(broker, mqttport);

  boolean status = mqtt.connect("GARAGE", mqttuser, mqttpass);

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");
//  mqtt.publish(topicInit, "GsmClientTest started");
  mqtt.subscribe(refreshTopic);
  mqtt.subscribe(heaterSetTopic);
  mqtt.subscribe(heaterSetTimerTopic);
  return mqtt.connected();
}
char buf[12];
void mqttPubAll(){
  if(mqtt.connected()){
      mqttSend=true;
      mqtt.publish("refresh/any", "0");
      mqtt.publish(heaterGetTopic, heaterStatus? "1" : "0");
      if(heaterStatus){
        itoa(heaterTimer, buf, 10);
        mqtt.publish(heaterTimerTopic, buf);
        } else {
        itoa(heaterTimerDefault, buf, 10);
        mqtt.publish(heaterTimerTopic, buf);
      }

      if(temperature!=NULL){
        if(temperature[0].t!=NULL && temperature[0].tempBufferData!=NULL){
//            SerialMon.print("PUB ");
//            SerialMon.print(temperature[0].t);
//            SerialMon.print(":");
//            SerialMon.print(temperature[0].tempBufferData);
            mqtt.publish(temperature[0].t, temperature[0].tempBufferData, 4);
          }
        }
 
             
      itoa(batteryLvl, buf, 10);
      mqtt.publish("battery/level", buf);
      
      if(isCharging()){
        mqtt.publish("battery/charging", "1");
      } else {
        mqtt.publish("battery/charging", "0");
      }

      batteryAdcLvl = analogRead(BATT_LVL);
      float volt = batteryAdcLvl/547.002398082;
      dtostrf(volt, 0, 2, buf);
      mqtt.publish("battery/adclvl", buf);
      // 4.17=2281
      itoa(uptime, buf, 10);
      mqtt.publish("device/uptime", buf);

      mqttSend=false;
    } 

  }

  void startHeater(){
    heaterStatus=1;
    heaterTimer=heaterTimerDefault;
    SerialMon.print("startHeater timer=");
    SerialMon.println(heaterTimer);
    digitalWrite(HEATER_Pin,LOW);
    delay(250);
    digitalWrite(HEATER_Pin,HIGH);
    mqttPubAll();
    }
  void tickHeater(){
    getTemperature();
    batteryLvl=getBatteryLevel();
    if(heaterStatus==1){
      heaterTimer--;
      if(heaterTimer<=0){
        stopHeater();
        }
      mqttPubAll();
      }
      
    }
  void stopHeater(){
    heaterStatus=0;
    SerialMon.println("stopHeater");
    digitalWrite(HEATER_Pin,LOW);
    delay(2500);
    digitalWrite(HEATER_Pin,HIGH);
    mqttPubAll();
  }


  void getTemperature(){
    sensors.requestTemperatures(); // Send the command to get temperatures
  
    // Loop through each device, print out temperature data
    for(int i=0;i<numberOfDevices; i++){
      // Search the wire for address
      if(sensors.getAddress(tempDeviceAddress, i)){
        // Output the device ID
        SerialMon.print("Temperature for device: ");
        SerialMon.println(i,DEC);
        float tempC = sensors.getTempC(tempDeviceAddress);
        SerialMon.print("Temp C: ");
        sprintf(temperature[i].tempBufferData, "%.1f",tempC);
        sprintf(temperature[i].t, "t%d",i);

        SerialMon.print(tempC);
      }
    }
    
    }
char* printAddress(DeviceAddress deviceAddress) {
  char ret[64];
  String r="";
  for (uint8_t i = 0; i < 8; i++){
    if (deviceAddress[i] < 16) Serial.print("0");
      ////SerialMon.print(deviceAddress[i], HEX);
      char buf[2];
      sprintf(buf, "%X", deviceAddress[i]);
      r=r+buf;
  SerialMon.println(r);    
  }
  
//  r.toCharArray(ret, 32);
//  SerialMon.println(ret);
  return ret;
}

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  pinMode(HEATER_Pin, OUTPUT_OPEN_DRAIN);
  //pinMode(BATT_LVL, ANALOG);
  digitalWrite(HEATER_Pin,HIGH);
  pinMode(LED, OUTPUT);

  int potValue = analogRead(BATT_LVL);
  SerialMon.println(potValue);
      
  // Keep power when running from battery
  Wire.begin(I2C_SDA, I2C_SCL);
  bool   isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  batteryLvl=getBatteryLevel();
  if(isChargeFull()){
    SerialMon.println("Battery full charged.");
    }

    if(isCharging()){
      SerialMon.println("Battery charging.");
      }
  
  modemInit();
  timerInit();
    // Start the DS18B20 sensor
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();
  
  SerialMon.print("Locating devices...");
  SerialMon.print("Found ");
  SerialMon.print(numberOfDevices, DEC);
  SerialMon.println(" devices.");
  getTemperature();
  
}



void loop() {
  if(mode==0){
    modemInit();
    }
  if(mode==1){
    SerialMon.print("Waiting for network...");
    if (!modem.waitForNetwork(240000L)) {
      SerialMon.println(" fail. reinit.");
      delay(10000);
      mode=0;
      return;
    }

    mode=2;
    SerialMon.println(" OK");
    }
  if(mode==2){
      if (modem.isNetworkConnected()) {
        SerialMon.println("Network connected");
        mode=3;
      } 
    }
  if(mode==3){
    SerialMon.print(F("Connecting to APN: "));
    SerialMon.print(apn);
    if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
      SerialMon.println(" fail");
      delay(10000);
      mode=2;
      return;
    }
    SerialMon.println(" OK");
    mode=4;
    }
    if(mode==4){
        mqtt.setServer(broker, 1883);
        mqtt.setCallback(mqttCallback);
        mode=5;
      }
    if(mode==5){
      
        if (!mqtt.connected()) {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        digitalWrite(LED, LOW);
        // Reconnect every 10 seconds
        unsigned long t = millis();
        if (t - lastReconnectAttempt > 10000L) {
          lastReconnectAttempt = t;
          if (mqttConnect()) {
            lastReconnectAttempt = 0;
          } else {
            mode=3;
            }
        }
        return;
      }
      digitalWrite(LED, HIGH);
      
      
    
      mqtt.loop();
      
    }

    
   if (interruptCounter > 0) {
 
    portENTER_CRITICAL(&timerMux);
    interruptCounter--;
    //interruptCounter=0;
    portEXIT_CRITICAL(&timerMux);
    
    minute--;
    if(minute<=0){
      minute=60;
      hour--;
      if(hour<=0){
        hour=60;
        uptime++;
      }
      tickHeater();
      mqttPubAll();
      }
    
  }
  delay(150);
}
