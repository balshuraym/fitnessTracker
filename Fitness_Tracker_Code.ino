#define BLYNK_TEMPLATE_ID "TMPLFUU_gZc7"
#define BLYNK_DEVICE_NAME "Wemos D1 Mini"
#define BLYNK_AUTH_TOKEN "rlnZYHlkR4JKTuM9eCUCZLE7H_k84cjR"

#include "math.h"
#include <DFRobot_LIS2DH12.h>
#include "DFRobot_BMP388.h"
#include "DFRobot_BMP388_I2C.h"
#include "Wire.h"
#include "SPI.h"
#include "bmp3_defs.h"

DFRobot_LIS2DH12 LIS; //Accelerometer

#define CALIBRATE_Altitude

/*Create a bmp388 object to communicate with IIC.*/
DFRobot_BMP388_I2C bmp388;

float seaLevel;
int steps = 0;
int set = 0;
int sign = -1;
int16_t a, b, l, m, r1, r2;
int flight = 0;
int currAlt = 0;
int pastAlt = 0;
float currflight;
float prevflight;
float difference = 0;


/*************************************************************
  Getting Started with Blynk Wifi Connection to WeMos D1
  You'll need:
   - Blynk IoT app (download from App Store or Google Play)
   - WeMos D1 board
   - Decide how to connect to Blynk
     (USB, Ethernet, Wi-Fi, Bluetooth, ...)
 *************************************************************/
 

#define BLYNK_PRINT Serial

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

char auth[] = BLYNK_AUTH_TOKEN;

char ssid[] = "Arduino"; //HOTSPOT NAME
char pass[] = "Fitness11"; //HOTSPOT PWD

void setup()
{
  // Debug console
  Wire.begin();
  Serial.begin(115200);

  //Initialize blynk wifi connection
  Blynk.begin(auth, ssid, pass, "blynk.cloud", 80);
  pinMode(LED_BUILTIN, OUTPUT);

  while(!Serial);
  delay(100);

  while(LIS.init(LIS2DH12_RANGE_16GA) == -1){  //Equipment connection exception or I2C address error
    Serial.println("No I2C devices found");
    delay(1000);
  }
  

  bmp388.set_iic_addr(BMP3_I2C_ADDR_SEC);
  /* Initialize bmp388*/
  while(bmp388.begin()){
    Serial.println("Initialize error!");
    delay(1000);
  }

  
  delay(100);
  seaLevel = bmp388.readSeaLevel(525.0);

  int16_t x, y, z;

  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);
  a = x;
  b = x;
  l = y;
  m = y;

  for(int i = 0; i< 50; i++){
      delay(100);
      LIS.readXYZ(x, y, z);
      LIS.mgScale(x, y, z);

      if(x > a){
        a = x;
      }
      else if(x < b){
        b = x;
      }      
  }

  Serial.print("a: " + (String)a + "b: "); //print a, b
  Serial.println(b);

  Serial.print("l: " + (String) l + "m: ");
  Serial.println(m);

  

}

BLYNK_WRITE(V1) // Executes when the value of virtual pin 0 changes
{
  if(param.asInt() == 1)
  {
    // execute this code if the switch widget is 1
    digitalWrite(LED_BUILTIN,LOW);
  }
  else
  {
    // execute this code if the switch widget is 0
    digitalWrite(LED_BUILTIN,HIGH); 
  }

  currflight = prevflight = bmp388.readCalibratedAltitude(seaLevel);
  
}

void loop()
{
  Blynk.run();

  float Temperature = bmp388.readTemperature();
  
    if((millis() % 10) == 0){
      Blynk.virtualWrite(V1, steps);
      Blynk.virtualWrite(V2, Temperature);
      Blynk.virtualWrite(V3, flight);
    }

Serial.println(Temperature);
acceleration();

}

void acceleration(void)
{
  #ifdef CALIBRATE_Altitude
  /* Read the calibrated altitude */
  float altitude = bmp388.readCalibratedAltitude(seaLevel);
 
  #else
  /* Read the altitude */
  float altitude = bmp388.readAltitude();

  #endif
  delay(100);
  currAlt = altitude;
  
  if(set == 0){
    
    pastAlt = currAlt;
    set = 1;
  }
  
 currflight = bmp388.readCalibratedAltitude(seaLevel);
  if (abs(currflight - prevflight) > 2.5){ //2.5 meters is roughly the heigh of a flight of stairs
     flight++; 
     prevflight = currflight;
  }
  Serial.println("currflight : ");
  Serial.println(currflight); 
  Serial.println("prevflight : ");
  Serial.println(prevflight); 
  Serial.println("Flights : ");
  Serial.println(flight);
  
  /* Read the atmospheric pressure, print data via serial port.*/
  float Pressure = bmp388.readPressure();

  /* Read the temperature, print data via serial port.*/
  float Temperature = bmp388.readTemperature();

  int16_t x, y, z, r;

  delay(100);
  LIS.readXYZ(x, y, z);
  LIS.mgScale(x, y, z);
  Serial.print("Acceleration x: "); //print acceleration
  Serial.println(x);

  if(x > a && sign == 1){
    steps++;
    sign = -1;
  }

  if(x < b && sign == -1){
    steps++;
    sign = 1;
  }  

  Serial.print("Steps: "); //print steps
  Serial.println(steps);

}
