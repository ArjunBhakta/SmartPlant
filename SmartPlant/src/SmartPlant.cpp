/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#line 1 "c:/Users/Arjun/Documents/IOT/SmartPlant/SmartPlant/src/SmartPlant.ino"
/*
 * Project SmartPlant
 * Description: Smart watering system
 * Author: Arjun Bhakta
 * Date: 23-March-2022
 */

// libraries

// Particle library
#include "Particle.h"
void setup();
void loop();
void glowColor();
void glowBlue();
void glowGreen();
void glowYellow();
void glowOrange();
void glowRed();
int getAirQuality();
int getBMEtemp();
int getBMEhumidity();
int getSoilReading(int _pin);
void screen();
void waterPlant();
void MQTT_connect();
void adafruitSubscribe();
void adafruitPublish();
#line 12 "c:/Users/Arjun/Documents/IOT/SmartPlant/SmartPlant/src/SmartPlant.ino"
SYSTEM_MODE(SEMI_AUTOMATIC);

// bme library
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Wire.h>
Adafruit_BME280 bme; 

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
Adafruit_MQTT_Subscribe mqttColor = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/smartplant.color");

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

unsigned long pumpTimer;

unsigned long currentTime;
unsigned long lastTime;
unsigned long publishTime;
bool needWater = false;
bool timerStart = false;
unsigned long getDustTimer;
unsigned long waterButtonTimer;
unsigned int waterTimer;


// Dust Sensor Variables 
unsigned long duration;
unsigned long dustStartTime;
unsigned long sampleTime_ms = 2000;//sampe 30s&nbsp;;
unsigned long lowPulseOccupancy = 0;
unsigned long last;
float ratio = 0;
float concentration = 0;

bool waterButtonState;
bool prevWaterButtonState;
bool LEDState;
bool CheckAirState;

// colorPicker 
int color;
byte buf [6];

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
    mqtt.subscribe(&mqttColor);

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

  quality = getAirQuality();
  temp= getBMEtemp();
  humidity= getBMEhumidity();
  moisture= getSoilReading(SOIL_SENSOR);
  
 

if (moisture>3000 && (millis() - waterTimer) > 5000){
      needWater= true;
      timerStart=true;
  waterTimer=millis();
}

 waterPlant();

 // Subscription Packages
  Adafruit_MQTT_Subscribe *subscription;
  // Water Plant Manually
    while ((subscription = mqtt.readSubscription(1000))) {
        if (subscription == &mqttWater) {
            prevWaterButtonState= waterButtonState;
          if (millis()-waterButtonTimer>500){
              waterButtonState = atof((char *)mqttWater.lastread);
              waterButtonTimer= millis();
                if (waterButtonState != prevWaterButtonState){
                  needWater= true;
                  timerStart= true;
                } 
                else{
                  needWater= false;
                } 
          }
        Serial.printf("Received %i from Adafruit.io feed needWater \n", waterButtonState);
        }     
  // Glow Pixels Manually
        if (subscription == &mqttColor) {
            Serial.printf (" Received from Adafruit : %s \n" ,( char *)mqttColor.lastread);
            memcpy(buf,&mqttColor.lastread [1] ,6); // strip off the ???#???
            Serial.printf(" Buffer : %s \n" ,( char *) buf );
            color = strtol((char *) buf ,NULL ,16) ; // convert string to int (hex)
            Serial.printf ("Buffer : 0x%02X \n",color);
        }
        if(subscription == &mqttLightPixels){
            LEDState = atof((char *)mqttLightPixels.lastread);  
            if (LEDState==true){
            glowColor();
            Serial.printf("Received %i from Adafruit.io feed pixel \n", LEDState);
            }
        }


  }
 // publish to adafruit.io every 60 seconds
 if ((millis() - publishTime > 15000)) {
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



// dust sensor every 90 seconds
if (millis() - getDustTimer > 90000){
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
  getDustTimer = millis();
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
// lights neo pixels with any color | Utilize with dashboard for fun
void glowColor(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, color);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
    for (j=75; j >-1; j--){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i,color);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
}

// lights neo pixels with blue | utlize if plant has been watered
void glowBlue(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, 0x0000FF);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
    for (j=75; j >-1; j--){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i,0x0000FF);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
}

// lights neo pixels green | utlize if airquality is GREAT
void glowGreen(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, 0x0000FF);
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

// lights neo pixels yellow | utlize if airquality is OK
void glowYellow(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, 0xfff600);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
    for (j=75; j >-1; j--){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i,0xfff600);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
}

// lights neo pixels yellow | utlize if airquality is BAD
void glowOrange(){
  int i;
  int j;
    for (j=0; j <75; j=j+2){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i, 0xff9100);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
    for (j=75; j >-1; j--){
      for (i = 0; i < pixel.numPixels(); i++) {
        pixel.setPixelColor(i,0xff9100);
        pixel.setBrightness(j);
        pixel.show(); 
      }
    }
}
// lights neo pixels yellow | utlize if airquality is HORRIBLE
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

int getAirQuality(){
  static int _airReading;
  static int _quality;
  static unsigned int _airQualityTimer;

  if(millis()-_airQualityTimer > 3000){
    
  _quality = sensor.slope();
  //Serial.print("Sensor value: ");
  //Serial.println(sensor.getValue());
  _airReading= sensor.getValue();
  if (_quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("High pollution! Force signal active.");
    //glowRed();
  }
  else if (_quality == AirQualitySensor::HIGH_POLLUTION) {
    Serial.println("High pollution!");
    //glowOrange();
  }
  else if (_quality == AirQualitySensor::LOW_POLLUTION) {
    Serial.println("Low pollution!");
    //glowYellow();  
    }
  else if (_quality == AirQualitySensor::FRESH_AIR) {
    Serial.println("Fresh air.");
    //glowGreen();
  }
   _airQualityTimer = millis();
  }
  return _airReading;
}

int getBMEtemp(){
  static unsigned int _bmeTimerTemp;
  static int _temp;
  unsigned int _updateTime = 30000;
    if(millis()-_bmeTimerTemp > _updateTime){
      _temp = ((bme.readTemperature()*(1.8))+32);
      _bmeTimerTemp= millis();
    }
    return _temp;
}

int getBMEhumidity(){
  static unsigned int _bmeTimerHumidity;
  static int _humidity;
  unsigned int _updateTime = 30000;

    if(millis()-_bmeTimerHumidity > _updateTime){
      _humidity = bme.readHumidity();
     _bmeTimerHumidity= millis();
    }
    return _humidity;
}

int getSoilReading(int _pin){
  static unsigned int _soilTimer;
  static int _moisture;
  unsigned int _updateTimeSoil = 3000;

  if (millis() - _soilTimer > _updateTimeSoil){
    _moisture=analogRead(_pin);
    _soilTimer=millis();
  }
  return _moisture;
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

void waterPlant(){

if(needWater==true){  
 digitalWrite(WATER_PUMP, HIGH);
   if (timerStart == true){
     pumpTimer=millis();
     timerStart=false;
   }
    if(millis()-pumpTimer > 300){
      needWater=false;
      if(needWater==false){
        digitalWrite(WATER_PUMP, LOW);
        glowBlue();
        timerStart=true;
      }  
    }
 }
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

void adafruitSubscribe(){
}

void adafruitPublish(){
}
