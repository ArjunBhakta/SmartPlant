/*
 * Project SmartPlant
 * Description: Smart watering system
 * Author: Arjun Bhakta
 * Date: 18-March-2022
 */

// libraries

// Particle library
#include "Particle.h"
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
//  Setup Feeds to publish
//  Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Publish mqttSoilMoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartplant.soilmoisture");
Adafruit_MQTT_Publish mqttAirQuality = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartplant.airquality");
Adafruit_MQTT_Publish mqttTemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartplant.temp");
Adafruit_MQTT_Publish mqttHumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartplant.humidity");
Adafruit_MQTT_Publish mqttDust = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/smartplant.dust");
//  Setup Feeds to subscribe
Adafruit_MQTT_Subscribe mqttCheckAir = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/smartplant.manualcheckairquality");
Adafruit_MQTT_Subscribe mqttWater = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/smartplant.manualwater");
Adafruit_MQTT_Subscribe mqttLightPixels = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/smartplant.manuallightpixels");


// Declare Global Variables

int moisture;
int temp;
int humidity;
int quality;

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
unsigned long publishTime;
bool needWater = true;
bool timerStart = true;

// Dust Sensor Variables 
unsigned long duration;
unsigned long dustStartTime;
unsigned long sampleTime_ms = 2000;//sampe 30s&nbsp;;
unsigned long lowPulseOccupancy = 0;
unsigned long last;
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

    // wait for Serial Monitor to startup
    waitFor(Serial.isConnected, 15000); 

    // Connect to WiFi without going to Particle Cloud
    WiFi.connect();
    while (WiFi.connecting()) {
        Serial.printf(".");
    }

    // Setup MQTT subscription for onoff feed.
    mqtt.subscribe(&mqttCheckAir);
    mqtt.subscribe(&mqttWater);
    mqtt.subscribe(&mqttLightPixels);

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

    // OLED
    display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
    display.clearDisplay();
    Time.zone(-6);
    Particle.syncTime();
    lastTime = millis();

    // neopixel setup
    pixel.begin();
    pixel.show();

    // 
    bme.begin();

}

void loop() {
   // Validate connected to MQTT Broker
    MQTT_connect();
   // Ping MQTT Broker every 2 minutes to keep connection alive
    if ((millis() - last) > 120000) {
        Serial.printf("Pinging MQTT \n");
        if (!mqtt.ping()) {
            Serial.printf("Disconnecting \n");
            mqtt.disconnect();
        }
        last = millis();
    }
  

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
 
// WaterPlant

 if(needWater==true){  
   waterON();
   if (timerStart == true){
     pumpTimer=millis();
     timerStart=false;
   }
    if(millis()-pumpTimer > 300){
      needWater=false;
      if(needWater==false){
        waterOFF();
        glowBlue();
        timerStart=true;
      }  
    }
 }

 // Subscription Packages
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(100))) {
        if (subscription == &mqttWater) {
            needWater = atof((char *)mqttWater.lastread);
            Serial.printf("Received %i from Adafruit.io feed needWater \n", needWater);
        }
    }


 // publish to adafruit.io every 10 seconds
 
 if ((millis() - publishTime > 60000)) {
        if (mqtt.Update()) {

          mqttSoilMoisture.publish(moisture);
          mqttAirQuality.publish(quality);
          mqttTemp.publish(temp);
          mqttHumidity.publish(humidity);
          mqttDust.publish(concentration);
          
          Serial.printf("Publishing to adafruit.io");
        }
        publishTime = millis();
    }

// dust sensor
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
// get soil Reading
// check if soil is less than desired moisture level every 30 minutes
 if (SoilReading() <3000){
    //needWater= true;
 }

// Current Time
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
    for (j=75; j >-1; j--){
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


  quality = sensor.slope();
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
   // glowGreen();
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
    Serial.printf("Temperature F = %i\ Humididty= %i %",temp,humidity);
    //
}

int SoilReading(){
  moisture=analogRead(SOIL_SENSOR);
  return moisture;
  //Serial.printf("%i \n", moisture);
}

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

void MQTT_connect() {
    int8_t ret;

    // Stop if already connected.
    if (mqtt.connected()) {
        return;
    }

    Serial.print("Connecting to MQTT... ");

    while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
        Serial.printf("%s\n", (char *)mqtt.connectErrorString(ret));
        Serial.printf("Retrying MQTT connection in 5 seconds..\n");
        mqtt.disconnect();
        delay(5000); // wait 5 seconds
    }
    Serial.printf("MQTT Connected!\n");
}


// void zapier()// have Zapier connect to  an app
// void mqqt() // look back at example to start publishing data