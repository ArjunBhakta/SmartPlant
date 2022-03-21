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
void flashRed();
void glowBlue();
void glowGreen();
void glowRed();
void airQuality();
void waterON();
void waterOFF();
void getBMEVal();
int getSoilReading();
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
#define PIXEL_COUNT 12
#define PIXEL_TYPE WS2812B

Adafruit_NeoPixel pixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// OLED library

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

unsigned long airQualityTimer;
unsigned long pumpTimer;
unsigned long bmeTimer;

//
const int SOIL_SENSOR= A0;
const int AIR_QUALITY_SENSOR = A1;
const int DUST_SENSOR = A2;
const int WATER_PUMP = D11;
const int NEOPIXELS = D2;

int moisture;
bool needWater = true;

void setup() {

    //pin INPUT
    pinMode(SOIL_SENSOR, INPUT);
    pinMode(AIR_QUALITY_SENSOR,INPUT);
    pinMode(DUST_SENSOR, INPUT);
    //pin OUTPUT
    pinMode(NEOPIXELS, OUTPUT);
    pinMode (WATER_PUMP, OUTPUT);

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
  // check Air Quality 
  if(millis()-airQualityTimer > 5000){
    airQuality();
    airQualityTimer = millis();
  }
  // print BME Values
   if(millis()-bmeTimer > 10000){
    getBMEVal();
    bmeTimer = millis();
  }
 
 if(needWater==true){
    waterON();
    if(millis()-pumpTimer > 500){
      needWater=false;  
    }
 }
 else{
   waterOFF();
 }



    
   





 
  //  Serial.println(airQual);
   //glowBlue();

  //   flashRed();
  //   delay(500);
  //   glowBlue();
  //   delay(500);
  //   glowGreen();
  //   delay(500);
}

// void readSoil() // function to read the current moisture level in the soil ( //empty cup 3478, submerged in water 1780, dry soil 3466, 2216 little bit of water- 1800damp)
// void waterPlant() // function that waters plant for .5 sec

// void readDust()

// NEOPIXEL FUNCTIONS*******************************************************************//

// utlize this if air quality is bad
void flashRed() {
    int i;
    unsigned int redTimer;
    static unsigned int currentTime;
    redTimer= millis();
    for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(255, 0, 0));
        pixel.setBrightness(30);
    }
    pixel.show();

    if(currentTime-redTimer > 1000){

    }
    for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 0, 0));
        pixel.setBrightness(0);
    }
    pixel.show();
}
// lights neo pixels with blue // utlize if plant has been watered
void glowBlue() {
    int i;
    int j;
    for (j=0; j <50; j++){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 0, 200));
        pixel.setBrightness(j);
        pixel.show();
        delay(3);
    }
    }
    for (j=50; j >0; j--){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 0, 200));
        pixel.setBrightness(j);
        pixel.show();
        delay(3);
    }
    }

}
// utilize when 
void glowGreen(){
  int i;
  int j;
    for (j=0; j <50; j++){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 200, 0));
        pixel.setBrightness(j);
        pixel.show();

        delay(3);
    }
    }
    for (j=50; j >-1; j--){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(0, 200, 0));
        pixel.setBrightness(j);
        pixel.show();
        delay(3);
    }
    }

}

void glowRed() {
    int i;
    int j;
    for (j=0; j <50; j++){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(255, 0, 0));
        pixel.setBrightness(j);
        pixel.show();
        delay(3);
    }
    }
    for (j=50; j >0; j--){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, pixel.Color(255, 0, 0));
        pixel.setBrightness(j);
        pixel.show();
        delay(3);
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
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");
}

int getSoilReading(){
  int moisture;
  moisture=analogRead(SOIL_SENSOR);
  return moisture;
  //Serial.printf("%i \n", moisture);
}
// void greenPixels() // light when the plant has connected to the zapier
// void bmePixels() //
// void zapier()// have Zapier connect to  an app
// void OLED Display()
// void mqqt() // look back at example to start publishing data