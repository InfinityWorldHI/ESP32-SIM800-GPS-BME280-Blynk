/*  ESP32 SIM800L BME280
 *   Created: 26/05/2020
 *  Last Update 6/05/2020
*/
//******************************************************************
// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
//******************************************************************
// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial1 for AT commands (to SIM800 module)
#define SerialAT Serial1

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb

// Define the serial console for debug prints, if needed
//#define DUMP_AT_COMMANDS

#include <Wire.h>
#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif
//******************************************************************
#include "MQ135.h"          //For MQ135
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//******************************************************************
// BME280 pins
#define I2C_SDA_2            13
#define I2C_SCL_2            14

#define pinMQ135             33          //For MQ135
#define Soil_PIN             34

#define uS_TO_S_FACTOR 1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  1800        /* Time ESP32 will go to sleep (in seconds) 3600 seconds = 1 hour */
//180 second wake-up

#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00
//******************************************************************
// I2C for SIM800 (to keep it running when powered from battery)
TwoWire I2CPower = TwoWire(0);

// I2C for BME280 sensor
TwoWire I2CBME = TwoWire(1);
Adafruit_BME280 bme;
//******************************************************************
TinyGsmClient client(modem);
TinyGPSPlus gps;
MQ135 AirSensor = MQ135(pinMQ135);          //For MQ135
//******************************************************************
const char apn[]  = "mobile.vodafone.it";
const char user[] = "";
const char pass[] = "";
const char auth[] = "Auth Token";

const char simPIN[]   = ""; 

//Blynk details
const char server[] = "139.59.206.133";
const int  port = 80;
//******************************************************************
//String VpinTemp          = "V0";
//String VpinHumi          = "V1";
//String VpinSoil          = "V2";
//String VpinCPPM          = "V3";
//String VpinPres          = "V4";
//******************************************************************
//Calibration values
int InAir = 1288;
int InWater = 345;
//******************************************************************
float latitude;
float longitude;
//******************************************************************
bool GPS_Sat = false;
bool GPS_Loc = false;
//******************************************************************
unsigned long previousMillis = 0;
//***********************************************************
bool setPowerBoostKeepOn(int en){
  I2CPower.beginTransmission(IP5306_ADDR);
  I2CPower.write(IP5306_REG_SYS_CTL0);
  if (en) {
    I2CPower.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } else {
    I2CPower.write(0x35); // 0x37 is default reg value
  }
  return I2CPower.endTransmission() == 0;
}
//******************************************************************
int convertToPercent(int value){
  int percentValue = 0;
  percentValue = map(value, InAir, InWater, 0, 99);
  return percentValue;
}
//******************************************************************
void GetLocation(){
  Serial.println("Getting location: ");
  while (GPS_Sat == false){
    while (Serial2.available() > 0){
      if (gps.encode(Serial2.read())){
        displayInfo();
      }
      if(GPS_Loc){break;}
    }
    if(GPS_Loc){break;}
    
    if (millis() > 5000 && gps.charsProcessed() < 10){
      Serial.println("No GPS detected");
      GPS_Sat = false;
    }
    
    if (millis() - previousMillis > 30000){   //wait 0.5min
      previousMillis = millis();
      break;
    }
  }
  GPS_Loc = false;
}
//***********************************************************
void displayInfo() {
  if (gps.location.isValid()) {
    Serial.println("---------------------------");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6); 
    latitude = (gps.location.lat());
    longitude = (gps.location.lng());
    GPS_Loc = true;
  }
  else {
    Serial.println("Location: Not Available");
    GPS_Loc = false;
  }
  Serial.println("---------------------------");
  delay(1000);
}
//***********************************************************
void GetData(){
  bool debuge = true;          //For MQ135
  float Temp = bme.readTemperature();
  float Humi = bme.readHumidity();
  
  float Pressure = bme.readPressure() / 100.0F;
  
  long soil_H = 0;
  for(int i = 0; i < 50; i++){
    soil_H += analogRead(Soil_PIN);
    delay(2);
  }
  soil_H = soil_H / 50;
  //for calibration
  SerialMon.println("Analoge: ");
  SerialMon.println(soil_H);
  
  int Soil = convertToPercent(soil_H);
  //------------------------------------------------
  //MQ135
  uint32_t periodMQ135 = 2000L;
  float ppm, cppm, rzero, crzero;
  for(uint32_t StartMQ135 = millis(); (millis()-StartMQ135) < periodMQ135;){
    float fac   = AirSensor.getCorrectionFactor(Temp, Humi);
    float res   = AirSensor.getResistance(); 
    float cres  = AirSensor.getCorrectedResistance(Temp, Humi);
    ppm   = AirSensor.getPPM(); 
    cppm  = AirSensor.getCorrectedPPM(Temp, Humi);
    rzero = AirSensor.getRZero();
    crzero  = AirSensor.getCorrectedRZero(Temp, Humi);
    if(debuge){
      Serial.print("ADC VALUE = ");
      Serial.print(analogRead(pinMQ135));
      Serial.print("  Voltage = ");
      Serial.print((analogRead(pinMQ135) * 3.3 ) / (2047));
      Serial.println("volts");
    }
    delay(500);
  }
  //------------------------------------------------
  SerialMon.print("Temperature: ");
  SerialMon.println(Temp);
  SerialMon.print("Humidity: ");
  SerialMon.println(Humi);
  SerialMon.print("Pressure: ");
  SerialMon.println(Pressure);
  SerialMon.print("Soil: ");
  SerialMon.println(Soil);
  SerialMon.print("CPPM: ");
  SerialMon.println(cppm);
  
  SendToBlynk("V0", Temp);
  SendToBlynk("V1", Humi);
  SendToBlynk("V2", Soil);
  SendToBlynk("V3", cppm);
  SendToBlynk("V4", Pressure);
//  SendToBlynk("V5", latitude);
//  SendToBlynk("V6", longitude);
  SendToBlynkMap("V7", Temp, Humi, Soil, cppm, Pressure);
}
//******************************************************************
void SendToBlynk(String Vpin, float Value){
  String resource = "";
  resource = String("/") + auth + String("/update/") + String(Vpin) + String("?value=") + String(Value);
  
  SerialMon.print(F("Connecting to "));
  SerialMon.print(server);
  if (!client.connect(server, port)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");
  
  // Make a HTTP GET request:
  client.print(String("GET ") + resource + " HTTP/1.0\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    // Print available data
    while (client.available()) {
      char c = client.read();
      SerialMon.print(c);
      timeout = millis();
    }
  }
  
  SerialMon.println();
  client.stop();
}
//******************************************************************
void SendToBlynkMap(String Vpin, float Value1, float Value2, int Value3, float Value4, float Value5){
  String resource = "";
  resource = String("/") + auth + String("/update/") + String(Vpin) + "?value=1&value=" + String(latitude, 6) + "&value=" + String(longitude, 6) + "&value=Temp:" + String(Value1) + "C_Humi:" + String(Value2) ;
  resource += "_S:" + String(Value3) + "_Co2:" + String(Value4) + "PPM_Pre:" + String(Value5);
//  SerialMon.println(resource);
  SerialMon.print(F("Connecting to "));
  SerialMon.print(server);
  if (!client.connect(server, port)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");
  
  // Make a HTTP GET request:
  client.print(String("GET ") + resource + " HTTP/1.0\r\n");
  client.print(String("Host: ") + server + "\r\n");
  client.print("Connection: close\r\n\r\n");

  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 10000L) {
    // Print available data
    while (client.available()) {
      char c = client.read();
      SerialMon.print(c);
      timeout = millis();
    }
  }
  
  SerialMon.println();
  client.stop();
}
//******************************************************************
void setup() {
  // Set serial monitor debugging window baud rate to 115200
  SerialMon.begin(115200);
  //-----------------------------------------
  // Start I2C communication
  I2CPower.begin(I2C_SDA, I2C_SCL, 400000);
  I2CBME.begin(I2C_SDA_2, I2C_SCL_2, 400000);
  //-----------------------------------------
  // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
  //-----------------------------------------
  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  //-----------------------------------------
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);
  //-----------------------------------------
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  //-----------------------------------------
  // Restart SIM800 module, it takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // use modem.init() if you don't need the complete restart
  //-----------------------------------------
  // Unlock your SIM card with a PIN if needed
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) {
    modem.simUnlock(simPIN);
  }
  //-----------------------------------------
  // You might need to change the BME280 I2C address, in our case it's 0x76
  if (!bme.begin(0x76, &I2CBME)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
  //-----------------------------------------
  analogReadResolution(11);
  analogSetWidth(11);
  analogSetPinAttenuation(pinMQ135, ADC_6db);          //For MQ135
  //-----------------------------------------
  // Configure the wake up source as timer wake up  
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  //-----------------------------------------
  Serial2.begin(9600, SERIAL_8N1, 25, 35);
  delay(3000);
  GetLocation();
}
//******************************************************************
void loop() {
  SerialMon.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, user, pass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" OK");

  GetData();

  SerialMon.println(F("Server disconnected"));

  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));
  
  // Put ESP32 into deep sleep mode (with timer wake up)
  SerialMon.println("Going Sleep ");
  esp_deep_sleep_start();
}
//******************************************************************
