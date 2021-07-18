#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include <SD.h>
#include<string> 

#define DHTPIN 2  
#define DHTTYPE    DHT22
const int MPU_addr_low = 0x68;
const int MPU_addr_high = 0x69;

DHT_Unified dht(DHTPIN, DHTTYPE);

SoftwareSerial gpsSerial(3,4);//rx,tx
SoftwareSerial sdCard(1,2);
TinyGPS gps;

uint32_t delayMS;

#include <MQUnifiedsensor.h>

/************************Hardware Related Macros************************************/
#define         Board                   "Arduino UNO"
#define         Pin                     A0  //Analog input 4 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    "MQ-9" //MQ9
#define         Voltage_Resolution      5
#define         ADC_Bit_Resolution      10 // For arduino UNO/MEGA/NANO
#define         RatioMQ9CleanAir        9.6 //RS / R0 = 60 ppm 
/*****************************Globals***********************************************/
//Declare Sensor
MQUnifiedsensor MQ9(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);
float co_conc = 0;
float temp = 0;
float humidity = 0;
float lat = 0;
float lon = 0;
float velocity = 0;
String latitude = 0;
String longitude = 0;
unsigned long age, fix_age;
int gpsyear;
byte gpsmonth, gpsday, gpshour, gpsminute, gpssecond, hundredths;
double pitch,roll;
double acceleration;

//SPI Settings
//MOSI,MISO,SCLK Set by default
int cs_pin = 10;
int pow_pin = 8;

char tmp_str[7]; // temporary variable used in convert function
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char* convert_int16_to_str(int16_t i)
{
  sprintf(tm,"%6d",i);
  return tmp_str;
}

void setup() {
  Serial.begin(9600); //Init serial port
  pinMode(cs_pin,OUTPUT);
  pinMode(pow_pin,OUTPUT);
  digitalWrite(pow_pin,HIGH);

  //Check if the sd card is ready
  if(!SD.begin(cs_pin))
  {
    Serial.println("Card Failed");
    return;
  }
  Serial.println("Card Ready");
  
  gpsSerial.begin(9600);
  
  Wire.begin();
  Wire.beginTransmission(MPU_addr_low); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  
  MQ9.setRegressionMethod(1);
  MQ9.setA(599.65); MQ9.setB(-2.244);

  MQ9.init();
  
  //SENSOR CALIBRATION 
  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ9.calibrate(RatioMQ9CleanAir);
    Serial.print(".");
  }
  MQ9.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply");} //while(1);}
  /*****************************  MQ CAlibration ********************************************/ 
  MQ9.serialDebug(true);
//***************************************************************************************************************************************************************************************************
  //DHT22 Calibration
  // Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;

    String dataString;
    dataString += "Velocity , Acceleration , ";
    dataString += "Latitude , Longityde , ";
    dataString += "Reason , ";
    dataString += "Time , ";
    File dataFile = SD.open("log.csv",FILE_WRITE);
    if(dataFile)
    {
      dataFile.println(dataString);
      dataFile.close();
      Serial.println(dataString);
    }
    else
    {
      Serial.println("Couldn't access file");
    }
}

void loop() {
  while(gpsSerial.available())
  {
    if(gps.encode(gpsSerial.read()))
    {
      gps.f_get_position(&lat,&lon);//Get latitude and longitude
      gps.crack_datetime(&gpsyear, &gpsmonth, &gpsday, &gpshour, &gpsminute, &gpssecond, &hundredths, &fix_age);
      gps.f_speed_kmph(&velocity);
    }
    latitude = String(lat,6);
    longitude = String(lon,6);
  }
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_addr_low, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  getAngle(accelerometer_x,accelerometer_y,accelerometer_z);
  
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  MQ9.update(); // Update data, the arduino will be read the voltage on the analog pin
  co_conc = MQ9.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ9.serialDebug(); // Will print the table on the serial port

  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    temp = event.temperature;
    Serial.print(F("Temperature: "));
    Serial.print(temp);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    humidity = event.relative_humidity;
    Serial.print(F("Humidity: "));
    Serial.print(humidity);
    Serial.println(F("%"));
  }
  String dataString;
  dataString += to_string(velocity) + " , " + to_string(acceleration) + " , ";
  dataString += to_string(latitude) + " , " + to_string(longitude) + " , ";
  int flag = 0;
  if(co_conc > 50) //If concentration of CO is greater than 50PPM
  {
    flag = 1
    dataString += "CO Poisoning";
  }
  if(temp > 40 || humidity > 60)
  {
    if(flag == 1){
      dataString += " or "
    }
    dataString += "Heat Stroke";
    flag = 2;
  }
  if(pitch != 0 || roll != 0)
  {
    if(flag == 2)
    {
      dataString += " or ";
    }
    dataString += "Car Imbalance";
  }
  if(flag != 0){
    dataString += ",";
  }

  dataString += gpshour + " : " + gpsminute + " : " + gpsseconds + " on " + gpsday + "/" + gpsmonth + "/" + gpsyear;
    File dataFile = SD.open("log.csv",FILE_WRITE);
    if(dataFile)
    {
      dataFile.println(dataString);
      dataFile.close();
      Serial.println(dataString);
    }
    else
    {
      Serial.println("Couldn't access file");
    }
   
  
  delay(delayMS); //Delay between measurements.
}

//convert the accel data to pitch/roll
void getAngle(int Vx,int Vy,int Vz) {
  double x = Vx;
  double y = Vy;
  double z = Vz;

  acceleration = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
  
  pitch = atan(x/sqrt((y*y) + (z*z)));
  roll = atan(y/sqrt((x*x) + (z*z)));
  //convert radians into degrees
  pitch = pitch * (180.0/3.14);
  roll = roll * (180.0/3.14) ;
}
