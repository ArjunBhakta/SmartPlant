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
void airQuality();
void  waterPump();
#line 12 "c:/Users/Arjun/Documents/IOT/SmartPlant/SmartPlant/src/SmartPlant.ino"
SYSTEM_MODE(SEMI_AUTOMATIC);

// bme library
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; 
// I2C
                     // Adafruit_BME280 bme(BME_CS); // hardware SPI
// Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// dust sensor library

// Capacitive Soil Moisture Sensor

// air quality sensor library
#include "Air_Quality_Sensor.h"
AirQualitySensor sensor(A0);
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

void setup() {

    pinMode(A0, INPUT);
    pinMode(D2, OUTPUT);

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
    pinMode(D11,OUTPUT);
    waterPump();

}

void loop() {
  // if(millis()-airQualityTimer > 10000){
  //   airQuality();
  //   airQualityTimer = millis();
  // }


 
  //  Serial.println(airQual);
  //   flashRed();

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
// utilize when connected to Zapier
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

// void greenPixels() // light when the plant has connected to the zapier
// void bmePixels() //
// void zapier()// have Zapier connect to  an app
// void OLED Display()
// void mqqt() // look back at example to start publishing data


void airQuality(){

  int quality = sensor.slope();
  Serial.print("Sensor value: ");
  Serial.println(sensor.getValue());

  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("High pollution! Force signal active.");
    glowBlue();
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

void  waterPump(){
  digitalWrite(D11,HIGH);
  delay(500);
  digitalWrite(D11,LOW);
}