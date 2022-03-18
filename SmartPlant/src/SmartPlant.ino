/*
 * Project SmartPlant
 * Description: Smart watering system
 * Author: Arjun Bhakta
 * Date: 18-March-2022
 */



// libraries

// Particle library
#include "Particle.h"
SYSTEM_MODE (SEMI_AUTOMATIC);

// bme library
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BME_SCK D4
#define BME_MISO D3
#define BME_MOSI D2
#define BME_CS D5

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

// dust sensor library

// Capacitive Soil Moisture Sensor

// air quality sensor library
 #include "Air_Quality_Sensor.h"
AirQualitySensor sensor(A0);
int airQual;

// neo pixel library
  #include "neopixel.h"

// OLED library

// zapier library

// adafruit.io library
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

void setup() {

  pinMode(A0,INPUT);

  Serial.begin(9600);
  while (!Serial);

  Serial.println("Waiting sensor to init...");
  delay(20000);
  
  if (sensor.init()) {
    Serial.println("Sensor ready.");
  }
  else {
    Serial.println("Sensor ERROR!");
  }
}

void loop() {
  readAirQuality();
  Serial.println(airQual);
}

// void readSoil() // function to read the current moisture level in the soil ( //empty cup 3478, submerged in water 1780, dry soil 3466, 2216 little bit of water- 1800damp)
// void waterPlant() // function that waters plant for .5 sec
void readAirQuality() {
airQual= analogRead(A0);
}

// void readDust()
// void flashRed() // utlize this if air quality is bad
// void bluePixels() // lights neo pixels with blue feature// utlize if plant has been watered
// void greenPixels() // light when the plant has connected to the internet
// void bmePixels() //
// void zapier()// have Zapier connect to  an app
// void OLED Display()
// void mqqt() // look back at example to start publishing data
