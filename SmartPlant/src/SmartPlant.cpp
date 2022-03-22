/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Arjun/Documents/IOT/SmartPlant/SmartPlant/src/SmartPlant.ino"
/*
 * Project SmartPlant
 * Description: Smart watering system
 * Author: Arjun Bhakta
 * Date: 18-March-2022
 */

// libraries

// Particle library
#include "Particle.h"
void setup();
void loop();
void glowBlue();
void glowGreen();
void glowRed();
void airQuality();
void waterON();
void waterOFF();
void getBMEVal();
int SoilReading();
void screen();
#line 12 "c:/Users/Arjun/Documents/IOT/SmartPlant/SmartPlant/src/SmartPlant.ino"
SYSTEM_MODE(SEMI_AUTOMATIC);

// bme library
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>

Adafruit_BME280 bme; 

// I2C
                     // Adafruit_BME280 bme(BME_CS); // hardware SPI
// Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// dust sensor library

// Capacitive Soil Moisture Sensor

// air quality sensor library
#include "Air_Quality_Sensor.h"
AirQualitySensor sensor(A1);
int airQual;

// neo pixel library
#include "neopixel.h"

#define PIXEL_PIN D2
#define PIXEL_COUNT 60
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// OLED library
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);
String DateTime;
String TimeOnly;

// zapier library

// adafruit.io librarybn0.
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "credentials.h"
#include <Adafruit_MQTT.h>

// Global State (you don't need to change this!)

TCPClient TheClient;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Feeds(Publish and Subscibe) to adafruit.io
//  Setup Feeds to publish or subscribe
//  Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish mqttRandomNumber = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/randomNumber");
Adafruit_MQTT_Subscribe mqttButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonOnOff");
Adafruit_MQTT_Subscribe mqttSlider = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/LEDrange");
Adafruit_MQTT_Publish mqttlocation = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/location");

// Declare Global Variables

int moisture;
int temp;
int humidity;

// PINS
const int SOIL_SENSOR= A0;
const int AIR_QUALITY_SENSOR = A1;
const int DUST_SENSOR = A2;
const int WATER_PUMP = D11;
const int NEOPIXELS = D2;

//timers
unsigned long airQualityTimer;
unsigned long pumpTimer;
unsigned long bmeTimer;
unsigned long currentTime;
unsigned long lastTime;
bool needWater = true;
bool timerStart = true;

// Dust Sensor Variables 
unsigned long duration;
unsigned long dustStartTime;
unsigned long sampleTime_ms = 2000;//sampe 30s&nbsp;;
unsigned long lowPulseOccupancy = 0;
float ratio = 0;
float concentration = 0;

void setup() {

    //pin INPUT
    pinMode(SOIL_SENSOR, INPUT);
    pinMode(AIR_QUALITY_SENSOR,INPUT);
    pinMode(DUST_SENSOR, INPUT);

    //pin OUTPUT
    pinMode(NEOPIXELS, OUTPUT);
    pinMode (WATER_PUMP, OUTPUT);

    // OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
    display.clearDisplay();
    Time.zone(-6);
    Particle.syncTime();
    lastTime = millis();

    // airQuality Sensor
    Serial.begin(9600);
    while (!Serial)
        ;
    Serial.println("Waiting sensor to init...");
    delay(2000);
    if (sensor.init()) {
        Serial.println("Sensor ready.");
    } else {
        Serial.println("Sensor ERROR!");
    }

    // neopixel setup
    pixel.begin();
    pixel.show();

    // 
    bme.begin();

   

}

void loop() {
  // check Air Quality every 5 seconds
  if(millis()-airQualityTimer > 5000){
    airQuality();
    airQualityTimer = millis();
  }
  // print BME Values every 10 seconds
   if(millis()-bmeTimer > 10000){
    getBMEVal();
    bmeTimer = millis();
  }
 
 if(needWater==true){  
   waterON();
   if (timerStart == true){
     pumpTimer=millis();
     timerStart=false;
   }
    if(millis()-pumpTimer > 250){
      needWater=false;
      if(needWater==false){
        waterOFF();
      }  
    }
 }


  duration = pulseIn(DUST_SENSOR, LOW);
  lowPulseOccupancy = lowPulseOccupancy+duration;
 
  if ((millis()-dustStartTime) >= sampleTime_ms)//if the sampel time = = 30s
  {
    ratio = lowPulseOccupancy/(sampleTime_ms*10.0);  // Integer percentage 0=&gt;100
    concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.printf("concentration = %f pcs/0.01cf \n",concentration);
    lowPulseOccupancy = 0;
    dustStartTime = millis();
  }

 SoilReading();

 currentTime = millis();

    if ((currentTime - lastTime) > 2000) {
        DateTime = Time.timeStr();
        TimeOnly = DateTime.substring(11, 19);
        lastTime = millis();
        screen();
    }currentTime = millis();

    if ((currentTime - lastTime) > 2000) {
        DateTime = Time.timeStr();
        TimeOnly = DateTime.substring(11, 19);
        lastTime = millis();
        screen();
    }
}


// void readSoil() // function to read the current moisture level in the soil ( //empty cup 3478, submerged in water 1780, dry soil 3466, 2216 little bit of water- 1800damp)
// void waterPlant() // function that waters plant for .5 sec


// NEOPIXEL FUNCTIONS*******************************************************************//

// lights neo pixels with blue // utlize if plant has been watered
void glowBlue(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 0, 200));
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
    for (j=75; j >-1; j=j-2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 0, 200));
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
}
// utilize when 
void glowGreen(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 200, 0));
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
    for (j=75; j >-1; j=j-2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 200, 0));
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
}
// utlize this if air quality is bad
void glowRed(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(255, 0, 0));
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
    for (j=75; j >-1; j=j-2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(255, 0, 0));
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
}

void airQuality(){


  int quality = sensor.slope();
  Serial.print("Sensor value: ");
  Serial.println(sensor.getValue());

  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("High pollution! Force signal active.");
    glowRed();
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    Serial.println("High pollution!");
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    Serial.println("Low pollution!");
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    Serial.println("Fresh air.");
    glowGreen();
  }

}

void waterON(){
digitalWrite(WATER_PUMP, HIGH);
}

void waterOFF(){
digitalWrite(WATER_PUMP, LOW);
}

void getBMEVal(){

    temp = ((bme.readTemperature()*(1.8))+32);
    humidity = bme.readHumidity();
    Serial.printf("Temperature F = %i\ Humididty= %i %", temp, humidity);
    //
}

int SoilReading(){
  moisture=analogRead(SOIL_SENSOR);
  return moisture;
  //Serial.printf("%i \n", moisture);
}
// void greenPixels() // light when the plant has connected to the zapier
// void bmePixels() //

void screen() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.printf("Time is %s\n", TimeOnly.c_str());
    display.printf("RoomTemp= %i F\n", temp);
    display.printf("Humidity= %i\n", humidity);
    display.printf("AirPollution= %i\n", humidity);
    display.printf("SoilMoisture= %i\n", moisture);
    display.printf("dust = %f pcs/0.01cf \n",concentration);
    display.display();
}


// void zapier()// have Zapier connect to  an app
// void mqqt() // look back at example to start publishing data