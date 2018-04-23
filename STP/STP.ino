////////////////////////////// LIBARY INCLUDES //////////////////////////////////////////


#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_INA219.h>
#include <SD.h>
//#include <SD_t3.h>
#include <Printers.h>
#include <XBee.h>
#include "TinyGPS++.h"


//////////////////////////////  OBJECTS INIT  //////////////////////////////////////////


Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
//SoftwareSerial mySerial(0, 1);  //TODO: replace RxPin, TxPin with correct pin#
//Adafruit_GPS GPS(&mySerial);
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_CCS811 ccs;
Adafruit_INA219 ina219;
XBee xbee = XBee();
TinyGPSPlus TGPS;


/////////////////////////////SENSORS' GLOBAL VARIABLES/////////////////////////////////////


#define UVpin 5 //TODO set the UVpin
#define MULTIPIN 15
const int RS = 10;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read

//#define buzzer 9 //TODO set the buzzer pin

bool decending = false;
float UVreading = 0;
File SD_File;

struct PAT {
  float pascals;
  float altm;
  float tempC;
} pat;

struct GPS {
  int hours;
  int minuts;
  int seconds;
  int milliseconds;
  float latitude;
  String lat;
  float longitude;
  String lon;
  float latitudeDegrees;
  float longitudeDegrees;
  float _speed;
  float angle;
  float _altitude;
  int satellites_num;
} gps;

struct Accelerometer {
  int x;
  int y;
  int z;
  float accel_x;
  float accel_y;
  float accel_z;
  uint8_t orientation;
} accel;

struct GAS {
  float temp;
  int eCO2;
  int TVOC;
} gas;

struct multimeter {
  float current;
} multi;


////////////////////////////  SNSOR: Pressure/Altitude/Temperature ////////////////////////////////////////


void PAT_init() {
  if (! baro.begin()) {
    Serial.println("SENSOR FAIL: MPL3115A2");
    return;
  }
  else {
    Serial.print("MPL3115A2 found.");
  }
}

void sensor_PAT() {
  //pat.pascals = baro.getPressure();   //pascals/3377 Inches (Hg)
  pat.altm = baro.getAltitude();
  pat.tempC = baro.getTemperature();
}


/////////////////////////// SENSOR: GPS //////////////////////////////////////////////////

void GPS_init(){
  Serial2.begin(9600);
}
void sensor_GPS(){
  if(Serial2.available()>0){
    TGPS.encode(Serial2.read());
  }
  if (TGPS.altitude.isUpdated()){
    Serial.println(TGPS.altitude.meters());
  }
  
}
/*
void GPS_init() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Sets output to only RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //update rate of 1s
  //GPS.sendCommand(PGCMD_ANTENNA); //Can report if antenna is connected or not
}

void sensor_GPS() {
  GPS.parse(GPS.lastNMEA()); //This is going to parse the last NMEA sentence the Arduino has received, breaking it down into its constituent parts.
  GPS.newNMEAreceived();
  gps.hours = GPS.hour;
  gps.minutes = GPS.minute;
  gps.seconds = GPS.seconds;
  gps.milliseconds = GPS.milliseconds;
  if (GPS.fix) {
    gps.latitude = GPS.latitude;
    gps.lat = GPS.lat;
    gps.longitude = GPS.longitude;
    gps.lon = GPS.lon;
    gps.latitudeDegrees = GPS.latitudeDegrees;
    gps.longitudeDegrees = GPS.longitudeDegrees;
    gps._speed = GPS.speed; //(knots)
    gps.angle = GPS.angle;
    gps._altitude = GPS.altitude;
    gps.satellites_num = (int)GPS.satellites;
  }
  else {
    Serial.println("GPS not fix");
  }
}
*/

////////////////////////  SENSOR: Accelerometer //////////////////////////////////////////


void accel_init() {
  if (! mma.begin()) {
    Serial.println("SENSOR FAIL: MMA8451");
    return;
  }
  Serial.println("MMA8451 found.");
  mma.setRange(MMA8451_RANGE_2_G);  //TODO: set range to ±2g, ±4g or ±8g
}

void sensor_accel() {
  mma.read();

  accel.x = mma.x;
  accel.y = mma.y;
  accel.z = mma.z;

  sensors_event_t event;
  mma.getEvent(&event);

  //the following three is in m/s^2
  accel.accel_x = event.acceleration.x;
  accel.accel_y = event.acceleration.y;
  accel.accel_z = event.acceleration.z;

  // example orientation output:
  //MMA8451_PL_PUF -> means Portrait Up Front, MMA8451_PL_PDB ->Portrait Down Back , MMA8451_PL_LRF -> Landscape Right Front... etc
  //(U or D + B or F combination for Portrait, R or L + F or B combination for Landscape)
  accel.orientation = mma.getOrientation();
  //delay used in example: 500
  Serial.print(accel.x);
  Serial.print(accel.y);
  Serial.print(accel.z);
  Serial.print(accel.accel_x);
  
}


/////////////////////////////////////// Air Quality Sensor (Gas Sensor) ///////////////////////////////////////////////


void gas_init() {
  Serial.println("CCS811 test");
  if (!ccs.begin()) {
    Serial.println("SENSOR FAIL: CCS811");
    return;
  }
  Serial.println("CCS811 found.");

  //calibrate temperature sensor
  while (!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}

void sensor_gas() {
  if (ccs.available()) {
    float temp = ccs.calculateTemperature();
    if (!ccs.readData()) {
      gas.eCO2 = ccs.geteCO2();
      gas.TVOC = ccs.getTVOC();
      gas.temp = temp;
    }
    else {
      Serial.println("ERROR!");
      while (1);
    }
  }
  //delay used in example: 500
}


//////////////////////////////////////////////  Multimeter ///////////////////////////////////////////////////////////////////


void multi_init() {
  /*
  //uint32_t currentFrequency;

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
  */
  
}

void sensor_multi() {
  // Read a value from the INA169 board
  float sensorValue = analogRead(MULTIPIN);

  // Remap the ADC value into a voltage number (5V reference)
  sensorValue = (sensorValue * VOLTAGE_REF) / 1023;

  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  multi.current = sensorValue / (10 * RS);

  // Output value (in amps) to the serial monitor to 3 decimal
  // places
  /*
  multi.shunt_voltage = ina219.getShuntVoltage_mV();        //  mV
  multi.bus_voltage = ina219.getBusVoltage_V();             //  V
  multi.current_mA = ina219.getCurrent_mA();                //  mA
  multi.power_mW = ina219.getPower_mW();                    //  mW
  multi.load_voltage = multi.bus_voltage + (multi.shunt_voltage / 1000);  //  V
  */
  // delay used in example: 2000
}


///////////////////////////////////////////  UV Sensor  //////////////////////////////////////////////////////////////////////


void UV_init() {
  pinMode(UVpin, INPUT);
}

void sensor_UV() {
  UVreading = analogRead(UVpin); //output in voltage TODO: conversion - devide by 0.1 to get UV index?
}


////////////////////////////////////////////  Buzzer  /////////////////////////////////////////////////////////////////////////

/*
void buzzer_init(){
  pinMode(buzzer, OUTPUT);
}

void buzzer(){
  if (PAT.altm > 200 && !decending){
    decending = true;
  }
  if ( decending ){
    if (PAT.altm < 100){
      digitalWrite(buzzer, HIGH);
    }
  }
  else{
    digitalWrite(buzzer, LOW);
  }
}
*/

//////////////////////////////////////////// SD Card //////////////////////////////////////////////////////////////////////////

/*
void SD_init() {
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("SD CARD FAIL");
    return;
  }
  Serial.println("SD init done.");
}

void write_to_SD() {
  SD_File = SD.open("test.txt", FILE_WRITE);

  if (SD_File) {
    // writing PAT data
    SD_File.print("Pressure: " + (float)pat.pascals / 3377); SD_File.println(" Inches (Hg)");
    SD_File.print("Altitude: " + pat.altm); SD_File.println(" meters");
    SD_File.print("Temperature: " + pat.tempC); SD_File.println("*C");

    // writing GPS data
    SD_File.print("\nTime: ");
    SD_File.print(gps.hour, DEC); SD_File.print(':');
    SD_File.print(gps.minute, DEC); SD_File.print(':');
    SD_File.print(gps.seconds, DEC); SD_File.print('.');
    SD_File.println(gps.milliseconds);
    SD_File.print("Location: ");
    SD_File.print(gps.latitude, 4); SD_File.print(gps.lat);
    SD_File.print(", ");
    SD_File.print(gps.longitude, 4); SD_File.println(gps.lon);
    SD_File.print("Latitude Degrees: " + gps.latitudeDegrees); SD_File.println("Longitude Degrees: " + gps.longitudeDegrees);
    SD_File.print("Speed (knots): "); SD_File.println(gps._speed);
    SD_File.print("Angle: "); SD_File.println(gps.angle);
    SD_File.print("Altitude: "); SD_File.println(gps._altitude);
    SD_File.print("Satellites: "); SD_File.println((int)gps.satellites_num);
    //Writing Accelerometer Data
    SD_File.print("Accelerometer Data:");
    SD_File.print("X:\t"); SD_File.print(accel.x);
    SD_File.print("\tY:\t"); SD_File.print(accel.y);
    SD_File.print("\tZ:\t"); SD_File.print(accel.z);
    SD_File.print("accel X: " + accel.accel_x); SD_File.println("m/s^2");
    SD_File.print("accel Y: " + accel.accel_y); SD_File.println("m/s^2");
    SD_File.print("accel Z: " + accel.accel_z); SD_File.println("m/s^2");
    switch (accel.orientation) {
      case MMA8451_PL_PUF:
        SD_File.println("Portrait Up Front");
        break;
      case MMA8451_PL_PUB:
        SD_File.println("Portrait Up Back");
        break;
      case MMA8451_PL_PDF:
        SD_File.println("Portrait Down Front");
        break;
      case MMA8451_PL_PDB:
        SD_File.println("Portrait Down Back");
        break;
      case MMA8451_PL_LRF:
        SD_File.println("Landscape Right Front");
        break;
      case MMA8451_PL_LRB:
        SD_File.println("Landscape Right Back");
        break;
      case MMA8451_PL_LLF:
        SD_File.println("Landscape Left Front");
        break;
      case MMA8451_PL_LLB:
        SD_File.println("Landscape Left Back");
        break;
    }
    SD_File.println();

    // writing Air Quality (Gas) Data
    SD_File.println("Gas Data:");
    SD_File.println("CO2: " + gas.eCO2);
    SD_File.println("TOVC: " + gas.TOVC);
    SD_File.println("Temperature: " + gas.temp);

    // writing Multimeter Data
    SD_File.println("Multimeter Data:");
    SD_File.print("Bus Voltage: " + multi.bus_voltage); SD_File.println(" V");
    SD_File.print("Shunt Voltage: " + multi.shunt_voltage); SD_File.println(" mV");
    SD_File.print("Load Voltage: " + mult.load_voltage); SD_File.println(" V");
    SD_File.print("Current: " + multi.current_mA); SD_File.println(" mA");
    SD_File.print("Power: " + multi.power_mW); SD_File.println(" mW");

    // writing UV Sensor Data
    SD_File.println("UV Sensor Data:");
    SD_File.println("UV: " + UVreading + " V");

    SD_File.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
*/
void SD_init() {
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(11, OUTPUT);
  if (!SD.begin(11)) {
    Serial.println("SD CARD FAIL");
    return;
  }
  Serial.println("SD init done.");
}

void write_to_SD() {
  SD_File = SD.open("TEST.txt", FILE_WRITE);
  Serial.println("r");
  SD_File.println("test");
  SD_File.close();
  /*
  if (SD_File) {
    SD_File.println("test");
    SD_File.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    
  }
  */
}

//////////////////////////////////////////////// Communcation //////////////////////////////////////////////////////////////

void comm_init() {
  //xbee.setSerial(Serial);
  //Serial3.begin(9600);
}

void send_data() {
  /*
  uint8_t payload[] = {}; // can only send 8 bit
  Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload)); //TODO address of remote XBee
  xbee.send(tx);
  */
  //Serial.print("x :");
  //Serial.print(accel.accel_x);
  //Serial.print(accel.accel_y);
  //Serial.print(accel.accel_z);
  Serial.print("p: ");Serial.println(pat.pascals/3377);
  Serial.print("alt: ");Serial.println(pat.altm);
  Serial.print("tempC: ");Serial.println(pat.tempC);
  //Serial.print("sdiofjew'ds");
  //Serial.print("co2: ");Serial.println(gas.eCO2);
  //Serial.print("TVOC: ");Serial.println(gas.TVOC);
  //Serial.print("temp: ");Serial.println(gas.temp);
  //Serial.print("current: "); Serial.println(multi.current);
  /*
  Serial.print("shunt voltage(mV): ");Serial.println( multi.shunt_voltage);
  Serial.print("bus voltage(V): ");Serial.println( multi.bus_voltage);
  Serial.print("current(mA): ");Serial.println( multi.current_mA);
  Serial.print("power(mW): ");Serial.println( multi.power_mW);
  Serial.print("load voltage(V): ");Serial.println( multi.load_voltage);
  */
}


///////////////////////////////////////////////////  Main  ///////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600);
  //SD_init();
  PAT_init();
  //GPS_init();
  //accel_init();
  //gas_init();
  //UV_init();
  //multi_init();
  comm_init();
  
  //buzzer_init();
}

uint32_t timer = millis();
bool dataSent = false;

void loop() {
  /*
  if (timer > millis())  timer = millis();
  if (millis() - timer > 500) {
    timer = millis();
    sensor_PAT();
    sensor_GPS();
    sensor_accel();
    sensor_gas();
    sensor_UV();
    if (!dataSent){
      send_data();
      dataSent = true;
    }
    else{
      dataSent = false;
    }
    buzzer();
  }
  */
  //sensor_GPS();
  sensor_PAT();
  //sensor_accel();
  //sensor_gas();
  //send_data();
  //sensor_multi();
  send_data();
  //Serial.print("hello12");
  //write_to_SD();
}////////////////////////////// LIBARY INCLUDES //////////////////////////////////////////


#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_INA219.h>
#include <SD.h>
//#include <SD_t3.h>
#include <Printers.h>
#include <XBee.h>
#include "TinyGPS++.h"


//////////////////////////////  OBJECTS INIT  //////////////////////////////////////////


Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
//SoftwareSerial mySerial(0, 1);  //TODO: replace RxPin, TxPin with correct pin#
//Adafruit_GPS GPS(&mySerial);
Adafruit_MMA8451 mma = Adafruit_MMA8451();
Adafruit_CCS811 ccs;
Adafruit_INA219 ina219;
XBee xbee = XBee();
TinyGPSPlus TGPS;


/////////////////////////////SENSORS' GLOBAL VARIABLES/////////////////////////////////////


#define UVpin 5 //TODO set the UVpin
#define MULTIPIN 15
const int RS = 10;          // Shunt resistor value (in ohms)
const int VOLTAGE_REF = 5;  // Reference voltage for analog read

//#define buzzer 9 //TODO set the buzzer pin

bool decending = false;
float UVreading = 0;
File SD_File;

struct PAT {
  float pascals;
  float altm;
  float tempC;
} pat;

struct GPS {
  int hours;
  int minuts;
  int seconds;
  int milliseconds;
  float latitude;
  String lat;
  float longitude;
  String lon;
  float latitudeDegrees;
  float longitudeDegrees;
  float _speed;
  float angle;
  float _altitude;
  int satellites_num;
} gps;

struct Accelerometer {
  int x;
  int y;
  int z;
  float accel_x;
  float accel_y;
  float accel_z;
  uint8_t orientation;
} accel;

struct GAS {
  float temp;
  int eCO2;
  int TVOC;
} gas;

struct multimeter {
  float current;
} multi;


////////////////////////////  SNSOR: Pressure/Altitude/Temperature ////////////////////////////////////////


void PAT_init() {
  if (! baro.begin()) {
    Serial.println("SENSOR FAIL: MPL3115A2");
    return;
  }
  else {
    Serial.print("MPL3115A2 found.");
  }
}

void sensor_PAT() {
  //pat.pascals = baro.getPressure();   //pascals/3377 Inches (Hg)
  pat.altm = baro.getAltitude();
  pat.tempC = baro.getTemperature();
}


/////////////////////////// SENSOR: GPS //////////////////////////////////////////////////

void GPS_init(){
  Serial2.begin(9600);
}
void sensor_GPS(){
  if(Serial2.available()>0){
    TGPS.encode(Serial2.read());
  }
  if (TGPS.altitude.isUpdated()){
    Serial.println(TGPS.altitude.meters());
  }
  
}
/*
void GPS_init() {
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Sets output to only RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //update rate of 1s
  //GPS.sendCommand(PGCMD_ANTENNA); //Can report if antenna is connected or not
}

void sensor_GPS() {
  GPS.parse(GPS.lastNMEA()); //This is going to parse the last NMEA sentence the Arduino has received, breaking it down into its constituent parts.
  GPS.newNMEAreceived();
  gps.hours = GPS.hour;
  gps.minutes = GPS.minute;
  gps.seconds = GPS.seconds;
  gps.milliseconds = GPS.milliseconds;
  if (GPS.fix) {
    gps.latitude = GPS.latitude;
    gps.lat = GPS.lat;
    gps.longitude = GPS.longitude;
    gps.lon = GPS.lon;
    gps.latitudeDegrees = GPS.latitudeDegrees;
    gps.longitudeDegrees = GPS.longitudeDegrees;
    gps._speed = GPS.speed; //(knots)
    gps.angle = GPS.angle;
    gps._altitude = GPS.altitude;
    gps.satellites_num = (int)GPS.satellites;
  }
  else {
    Serial.println("GPS not fix");
  }
}
*/

////////////////////////  SENSOR: Accelerometer //////////////////////////////////////////


void accel_init() {
  if (! mma.begin()) {
    Serial.println("SENSOR FAIL: MMA8451");
    return;
  }
  Serial.println("MMA8451 found.");
  mma.setRange(MMA8451_RANGE_2_G);  //TODO: set range to ±2g, ±4g or ±8g
}

void sensor_accel() {
  mma.read();

  accel.x = mma.x;
  accel.y = mma.y;
  accel.z = mma.z;

  sensors_event_t event;
  mma.getEvent(&event);

  //the following three is in m/s^2
  accel.accel_x = event.acceleration.x;
  accel.accel_y = event.acceleration.y;
  accel.accel_z = event.acceleration.z;

  // example orientation output:
  //MMA8451_PL_PUF -> means Portrait Up Front, MMA8451_PL_PDB ->Portrait Down Back , MMA8451_PL_LRF -> Landscape Right Front... etc
  //(U or D + B or F combination for Portrait, R or L + F or B combination for Landscape)
  accel.orientation = mma.getOrientation();
  //delay used in example: 500
  Serial.print(accel.x);
  Serial.print(accel.y);
  Serial.print(accel.z);
  Serial.print(accel.accel_x);
  
}


/////////////////////////////////////// Air Quality Sensor (Gas Sensor) ///////////////////////////////////////////////


void gas_init() {
  Serial.println("CCS811 test");
  if (!ccs.begin()) {
    Serial.println("SENSOR FAIL: CCS811");
    return;
  }
  Serial.println("CCS811 found.");

  //calibrate temperature sensor
  while (!ccs.available());
  float temp = ccs.calculateTemperature();
  ccs.setTempOffset(temp - 25.0);
}

void sensor_gas() {
  if (ccs.available()) {
    float temp = ccs.calculateTemperature();
    if (!ccs.readData()) {
      gas.eCO2 = ccs.geteCO2();
      gas.TVOC = ccs.getTVOC();
      gas.temp = temp;
    }
    else {
      Serial.println("ERROR!");
      while (1);
    }
  }
  //delay used in example: 500
}


//////////////////////////////////////////////  Multimeter ///////////////////////////////////////////////////////////////////


void multi_init() {
  /*
  //uint32_t currentFrequency;

  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
  */
  
}

void sensor_multi() {
  // Read a value from the INA169 board
  float sensorValue = analogRead(MULTIPIN);

  // Remap the ADC value into a voltage number (5V reference)
  sensorValue = (sensorValue * VOLTAGE_REF) / 1023;

  // Follow the equation given by the INA169 datasheet to
  // determine the current flowing through RS. Assume RL = 10k
  // Is = (Vout x 1k) / (RS x RL)
  multi.current = sensorValue / (10 * RS);

  // Output value (in amps) to the serial monitor to 3 decimal
  // places
  /*
  multi.shunt_voltage = ina219.getShuntVoltage_mV();        //  mV
  multi.bus_voltage = ina219.getBusVoltage_V();             //  V
  multi.current_mA = ina219.getCurrent_mA();                //  mA
  multi.power_mW = ina219.getPower_mW();                    //  mW
  multi.load_voltage = multi.bus_voltage + (multi.shunt_voltage / 1000);  //  V
  */
  // delay used in example: 2000
}


///////////////////////////////////////////  UV Sensor  //////////////////////////////////////////////////////////////////////


void UV_init() {
  pinMode(UVpin, INPUT);
}

void sensor_UV() {
  UVreading = analogRead(UVpin); //output in voltage TODO: conversion - devide by 0.1 to get UV index?
}


////////////////////////////////////////////  Buzzer  /////////////////////////////////////////////////////////////////////////

/*
void buzzer_init(){
  pinMode(buzzer, OUTPUT);
}

void buzzer(){
  if (PAT.altm > 200 && !decending){
    decending = true;
  }
  if ( decending ){
    if (PAT.altm < 100){
      digitalWrite(buzzer, HIGH);
    }
  }
  else{
    digitalWrite(buzzer, LOW);
  }
}
*/

//////////////////////////////////////////// SD Card //////////////////////////////////////////////////////////////////////////

/*
void SD_init() {
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);
  if (!SD.begin(10)) {
    Serial.println("SD CARD FAIL");
    return;
  }
  Serial.println("SD init done.");
}

void write_to_SD() {
  SD_File = SD.open("test.txt", FILE_WRITE);

  if (SD_File) {
    // writing PAT data
    SD_File.print("Pressure: " + (float)pat.pascals / 3377); SD_File.println(" Inches (Hg)");
    SD_File.print("Altitude: " + pat.altm); SD_File.println(" meters");
    SD_File.print("Temperature: " + pat.tempC); SD_File.println("*C");

    // writing GPS data
    SD_File.print("\nTime: ");
    SD_File.print(gps.hour, DEC); SD_File.print(':');
    SD_File.print(gps.minute, DEC); SD_File.print(':');
    SD_File.print(gps.seconds, DEC); SD_File.print('.');
    SD_File.println(gps.milliseconds);
    SD_File.print("Location: ");
    SD_File.print(gps.latitude, 4); SD_File.print(gps.lat);
    SD_File.print(", ");
    SD_File.print(gps.longitude, 4); SD_File.println(gps.lon);
    SD_File.print("Latitude Degrees: " + gps.latitudeDegrees); SD_File.println("Longitude Degrees: " + gps.longitudeDegrees);
    SD_File.print("Speed (knots): "); SD_File.println(gps._speed);
    SD_File.print("Angle: "); SD_File.println(gps.angle);
    SD_File.print("Altitude: "); SD_File.println(gps._altitude);
    SD_File.print("Satellites: "); SD_File.println((int)gps.satellites_num);
    //Writing Accelerometer Data
    SD_File.print("Accelerometer Data:");
    SD_File.print("X:\t"); SD_File.print(accel.x);
    SD_File.print("\tY:\t"); SD_File.print(accel.y);
    SD_File.print("\tZ:\t"); SD_File.print(accel.z);
    SD_File.print("accel X: " + accel.accel_x); SD_File.println("m/s^2");
    SD_File.print("accel Y: " + accel.accel_y); SD_File.println("m/s^2");
    SD_File.print("accel Z: " + accel.accel_z); SD_File.println("m/s^2");
    switch (accel.orientation) {
      case MMA8451_PL_PUF:
        SD_File.println("Portrait Up Front");
        break;
      case MMA8451_PL_PUB:
        SD_File.println("Portrait Up Back");
        break;
      case MMA8451_PL_PDF:
        SD_File.println("Portrait Down Front");
        break;
      case MMA8451_PL_PDB:
        SD_File.println("Portrait Down Back");
        break;
      case MMA8451_PL_LRF:
        SD_File.println("Landscape Right Front");
        break;
      case MMA8451_PL_LRB:
        SD_File.println("Landscape Right Back");
        break;
      case MMA8451_PL_LLF:
        SD_File.println("Landscape Left Front");
        break;
      case MMA8451_PL_LLB:
        SD_File.println("Landscape Left Back");
        break;
    }
    SD_File.println();

    // writing Air Quality (Gas) Data
    SD_File.println("Gas Data:");
    SD_File.println("CO2: " + gas.eCO2);
    SD_File.println("TOVC: " + gas.TOVC);
    SD_File.println("Temperature: " + gas.temp);

    // writing Multimeter Data
    SD_File.println("Multimeter Data:");
    SD_File.print("Bus Voltage: " + multi.bus_voltage); SD_File.println(" V");
    SD_File.print("Shunt Voltage: " + multi.shunt_voltage); SD_File.println(" mV");
    SD_File.print("Load Voltage: " + mult.load_voltage); SD_File.println(" V");
    SD_File.print("Current: " + multi.current_mA); SD_File.println(" mA");
    SD_File.print("Power: " + multi.power_mW); SD_File.println(" mW");

    // writing UV Sensor Data
    SD_File.println("UV Sensor Data:");
    SD_File.println("UV: " + UVreading + " V");

    SD_File.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}
*/
void SD_init() {
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(11, OUTPUT);
  if (!SD.begin(11)) {
    Serial.println("SD CARD FAIL");
    return;
  }
  Serial.println("SD init done.");
}

void write_to_SD() {
  SD_File = SD.open("TEST.txt", FILE_WRITE);
  Serial.println("r");
  SD_File.println("test");
  SD_File.close();
  /*
  if (SD_File) {
    SD_File.println("test");
    SD_File.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
    
  }
  */
}

//////////////////////////////////////////////// Communcation //////////////////////////////////////////////////////////////

void comm_init() {
  //xbee.setSerial(Serial);
  //Serial3.begin(9600);
}

void send_data() {
  /*
  uint8_t payload[] = {}; // can only send 8 bit
  Tx16Request tx = Tx16Request(0x1874, payload, sizeof(payload)); //TODO address of remote XBee
  xbee.send(tx);
  */
  //Serial.print("x :");
  //Serial.print(accel.accel_x);
  //Serial.print(accel.accel_y);
  //Serial.print(accel.accel_z);
  Serial.print("p: ");Serial.println(pat.pascals/3377);
  Serial.print("alt: ");Serial.println(pat.altm);
  Serial.print("tempC: ");Serial.println(pat.tempC);
  //Serial.print("sdiofjew'ds");
  //Serial.print("co2: ");Serial.println(gas.eCO2);
  //Serial.print("TVOC: ");Serial.println(gas.TVOC);
  //Serial.print("temp: ");Serial.println(gas.temp);
  //Serial.print("current: "); Serial.println(multi.current);
  /*
  Serial.print("shunt voltage(mV): ");Serial.println( multi.shunt_voltage);
  Serial.print("bus voltage(V): ");Serial.println( multi.bus_voltage);
  Serial.print("current(mA): ");Serial.println( multi.current_mA);
  Serial.print("power(mW): ");Serial.println( multi.power_mW);
  Serial.print("load voltage(V): ");Serial.println( multi.load_voltage);
  */
}


///////////////////////////////////////////////////  Main  ///////////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(9600);
  //SD_init();
  PAT_init();
  //GPS_init();
  //accel_init();
  //gas_init();
  //UV_init();
  //multi_init();
  comm_init();
  
  //buzzer_init();
}

uint32_t timer = millis();
bool dataSent = false;

void loop() {
  /*
  if (timer > millis())  timer = millis();
  if (millis() - timer > 500) {
    timer = millis();
    sensor_PAT();
    sensor_GPS();
    sensor_accel();
    sensor_gas();
    sensor_UV();
    if (!dataSent){
      send_data();
      dataSent = true;
    }
    else{
      dataSent = false;
    }
    buzzer();
  }
  */
  //sensor_GPS();
  sensor_PAT();
  //sensor_accel();
  //sensor_gas();
  //send_data();
  //sensor_multi();
  send_data();
  //Serial.print("hello12");
  //write_to_SD();
}
