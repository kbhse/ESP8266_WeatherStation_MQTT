/*  src/main.cpp */

#define PROGNAM "ESP8266_WeatherStation_MQTT"                                                      // program name
#define VERSION "v04.01"                                                                           // program version (nb lowercase 'version' is keyword)
#define PGMFUNCT "Temperature, Humidity, Pressure, Light Intensity"                                // what the program does
#define HARDWARE "Wemos D1 mini, pro and Shields"                                                  // hardware version
#define AUTHOR "J Manson"                                                                          // created by
#define CREATION_DATE "December 2019"                                                              // date
//#define DEBUG_OUT

// NB number the ESP8266 devices and edit the next 2 #defines accordingly !
#define MQTT_DEVICE "esp08"                                                                        // MQTT requires unique device ID (see reconnect() function)
#define PUB_SUB_CLIENT esp08client                                                                 // and unique client ?
#define MQTT_LOCATION "test"                                                                     // location for MQTT topic
#define UPDATE_FREQ 60000L                                                                         // 60 seconds


// ------------------------------------------------------------------
// MQTT
const char* mqtt_server = "192.168.1.74";
const int mqttPort = 1883;
const char* mqttUser = "mqttUser";
const char* mqttPassword = "hTR7gxBY4";

// MQTT code from: https://randomnerdtutorials.com/raspberry-pi-publishing-mqtt-messages-to-esp8266/
//                 https://github.com/knolleary/pubsubclient
// ------------------------------------------------------------------

/* Changelog
04.00 average 10 samples
04.01 add routine to change update frequency from Node RED
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "SimpleTimer.h"                                                                           // https://playground.arduino.cc/Code/SimpleTimer/ https://github.com/marcelloromani/arduino/tree/master/SimpleTimer
#include "WEMOS_SHT3X.h"                                                                           // Wemos Temperature and Humidity shield library
#include "PubSubClient.h"                                                                          // https://github.com/knolleary/pubsubclient

// Comment out sensors not in use
// ------------------------------
#define WEMOS_SHT30                                                                                // Wemos Temperature and Humidity shield
//#define WEMOS_HP303                                                                                // Wemos Barometric Pressure Shield
//#define WEMOS_BH1750                                                                               // Wemos Ambient Light Shield
//#define WEMOS_BATTERY                                                                              // Wemos Battery Shield
//#define RSSI                                                                                        // measure RSSI

// define which WiFi network to connect to (only 1 should be active)
// -----------------------------------------------------------------
#define BTHUB
//#define LINKSYS

#ifdef BTHUB                                                                                       // Gym
    const char* ssid = "BTHub4-5H9P";                                                              // BTHub WiFi credentials
    const char* password = "nB67c3zuRlPrAVcZL5YN";
#endif

#ifdef LINKSYS                                                                                     // Shed
    const char* ssid = "linksys";                                                                  // BTHub WiFi credentials
    const char* password = "rhenigidale";
#endif

// Initializes the espClient. You should change the espClient name if you have multiple ESPs running in your home automation system
//WiFiClient espClient;
WiFiClient PUB_SUB_CLIENT;
PubSubClient client(PUB_SUB_CLIENT);

// Lamp - LED - GPIO 4 = D2 on ESP-12E NodeMCU board
const int lamp = LED_BUILTIN;

// ------------------------------------------------------------------

#ifdef WEMOS_SHT30
    #include <WEMOS_SHT3X.h>                                                                       // Wemos Temperature and Humidity shield
    SHT3X sht30(0x45);                                                                             // SHT30 shield has two user selectable I2C addresses
    //static char sht_temperature[7];
    //static char sht_humidity[7];
#endif

#ifdef WEMOS_HP303
    #include <LOLIN_HP303B.h>                                                                      // Wemos Barometric Pressure HP303B Shield https://github.com/wemos/LOLIN_HP303B_Library
    LOLIN_HP303B HP303B;
#endif

#ifdef WEMOS_BH1750                                                                                // Wemos Ambient Light Shield
    #include <BH1750.h>                                                                            // ambient light sensor https://github.com/claws/BH1750
    BH1750 lightMeter(0x23);                                                                       // BH1750 can be configured to use two I2C addresses, 0x23 (default)or 0x5C
#endif
// ------------------------------------------------------------------

SimpleTimer timer;

// ------------------------------------------------------------------
// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
// Change the function below to add logic to your program, so when a device publishes a message to a topic that 
// your ESP8266 is subscribed you can actually do something
void callback(String topic, byte* message, unsigned int length)
    {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String messageTemp;
  
    for (int i = 0; i < length; i++)
        {
        Serial.print((char)message[i]);
        messageTemp += (char)message[i];
        }
    Serial.println();

    // Feel free to add more if statements to control more GPIOs with MQTT

    // If a message is received on the topic room/lamp, you check if the message is either on or off. Turns the lamp GPIO according to the message
    if(topic == MQTT_LOCATION "/lamp")
        {
        Serial.print("Changing Room lamp to ");
        if(messageTemp == "on")
            {
            digitalWrite(lamp, LOW);                                                               // active LOW
            Serial.print("On");
            }
        else if(messageTemp == "off")
            {
            digitalWrite(lamp, HIGH);
            Serial.print("Off");
            }
        }
        Serial.println();
    }

// ------------------------------------------------------------------
// This functions reconnects your ESP8266 to your MQTT broker
// Change the function below if you want to subscribe to more topics with your ESP8266 
    void reconnect()
    {
    // Loop until we're reconnected
    while (!client.connected())
        {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        /*
        YOU MIGHT NEED TO CHANGE THIS LINE, IF YOU'RE HAVING PROBLEMS WITH MQTT MULTIPLE CONNECTIONS
        To change the ESP device ID, you will have to give a new name to the ESP8266.
        Here's how it looks:
            if (client.connect("ESP8266Client")) {
        You can do it like this:
            if (client.connect("ESP1_Office")) {
        Then, for the other ESP:
            if (client.connect("ESP2_Garage")) {
        That should solve your MQTT multiple connections problem
        */
        if (client.connect(MQTT_DEVICE, mqttUser, mqttPassword))
            {
            Serial.println("connected");  
            // Subscribe or resubscribe to a topic
            // You can subscribe to more topics (to control more LEDs in this example)
            client.subscribe(MQTT_LOCATION "/lamp");
            }
        else
            {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
            }
        }
    }

// ------------------------------------------------------------------
void setup_wifi()
    {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
        {
        delay(500);
        Serial.print(".");
        }
    Serial.println("");
    Serial.print("WiFi connected - ESP IP address: ");
    Serial.println(WiFi.localIP());
    }

// ------------------------------------------------------------------

void combSort11(float *ar, uint8_t n)                                                              // this was used for median when temp samples stored in array
{
  uint8_t i, j, gap, swapped = 1;
  float temp;

  gap = n;
  while (gap > 1 || swapped == 1)
  {
    gap = gap * 10 / 13;
    if (gap == 9 || gap == 10) gap = 11;
    if (gap < 1) gap = 1;
    swapped = 0;
    for (i = 0, j = gap; j < n; i++, j++)
    {
      if (ar[i] > ar[j])
      {
        temp = ar[i];
        ar[i] = ar[j];
        ar[j] = temp;
        swapped = 1;
      }
    }
  }
}
// ------------------------------------------------------------------

void readSensors()
    {                                                                                              // function: reads sensors and sends data to blynk app
	
    #ifdef WEMOS_SHT30                                                                             // temperature and humidity
        #ifdef DEBUG_OUT
            Serial.println(F("Reading the SHT30: "));
        #endif
        static char temperatureTemp[7];                                                            // client.publish() expects char array
        static char humidityTemp[7];
        float averageTemperature = 0;
        float averageHumidity = 0;
        // get average of 10 consecutive readings
        // there is a 500ms delay in call to sht30 so loop takes 5 seconds !!
        for(int i = 0; i < 10; i++)
            {
            sht30.get();
            averageTemperature += sht30.cTemp;
            averageHumidity += sht30.humidity;
            yield();
            }
        averageTemperature /= 10;
        averageHumidity /= 10;
        // publish average temperature to mqtt
        dtostrf(averageTemperature, 6, 2, temperatureTemp);                                        // convert float to char array
        dtostrf(averageHumidity, 6, 2, humidityTemp);
        client.publish(MQTT_LOCATION "/temperature", temperatureTemp);
        client.publish(MQTT_LOCATION "/humidity", humidityTemp);
        #ifdef DEBUG_OUT
            Serial.print("Temperature: ");
            Serial.println(temperatureTemp);
            Serial.print("Humidity: ");
            Serial.println(humidityTemp);
        #endif
    #endif

    #ifdef WEMOS_HP303
        #ifdef DEBUG_OUT
            Serial.println(F("Reading the HP303B: "));
        #endif
        int32_t pressure;
        float hPa;
        static char hPaTemp[7];
        HP303B.measurePressureOnce(pressure);                                                      // read the barometric pressure
        hPa = pressure / 100.0;                                                                    // convert to hPa (note 100.0 for cast to float)
        #ifdef DEBUG_OUT
            Serial.println(hPa);
        #endif
        dtostrf(hPa, 6, 1, hPaTemp);
        client.publish(MQTT_LOCATION "/pressure", hPaTemp);
    #endif

    #ifdef WEMOS_BH1750
        #ifdef DEBUG_OUT
            Serial.println(F("Reading the BH1750: "));
        #endif
        uint16_t lux = lightMeter.readLightLevel();                                                // read the light level
        static char luxStr[5];
        itoa(lux, luxStr, 10);
        client.publish(MQTT_LOCATION "/light", luxStr);
    #endif
/*
    #ifdef RSSI
        int rssi = WiFi.RSSI();                                                                        // get RSSI (received signal strength indicator) of WiFi connection
        static char rssiStr[5];
        itoa(rssi, rssiStr, 10);
        client.publish(MQTT_LOCATION "/rssi", rssiStr);
    #endif

    #ifdef WEMOS_BATTERY
        int adc = analogRead(0);                                                                   // read the ESP ADC (connected to battery)
        float v = (adc * 4.335) / 1024;                                                            // calculate the voltage (fsd 4.335v)
        Blynk.virtualWrite(V47, v);                                                                // send to app
    #endif
*/
  	} // end of readSensors()


void setup()
    {

    Serial.begin(115200);
    Serial.printf("\n\n%s_%s, %s, %s, %s\n\n", PROGNAM, VERSION, AUTHOR, CREATION_DATE, MQTT_DEVICE);

    pinMode(lamp, OUTPUT);

    setup_wifi();
    client.setServer(mqtt_server, mqttPort);
    client.setCallback(callback);

    #ifdef WEMOS_HP303                                                                             // Wemos Barometric Pressure HP303B Shield
        HP303B.begin();                                                                            // I2C address = 0x77 (or HP303B.begin(0x76); //I2C address = 0x76)
    #endif

    #ifdef WEMOS_BH1750
        if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))                                    // begin returns a boolean that can be used to detect setup problems
            {
            Serial.println(F("BH1750 Advanced begin"));
            }
        else
            {
            Serial.println(F("Error initialising BH1750"));
            }
    #endif

    timer.setInterval(UPDATE_FREQ, readSensors);                                                        // every x minutes, send data to MQTT

    } // end of setup()


void loop()
    {

    if (!client.connected())
        {
        reconnect();
        }
    if (!client.loop())
        {
        client.connect("ESP8266Client");
        }

    timer.run();

    } // end of loop()



/*
        // read 11 temperature measurements to array
        // there is a 500ms delay in call to sht30 so loop takes 5.5 seconds !!
        float temperature[11];
        //Serial.println(millis());
        for(int i = 0; i < 11; i++)
            {
            sht30.get();
            temperature[i] = sht30.cTemp;
            yield();
            }

        // use mid element of array as unsmoothed sample
        dtostrf(temperature[5], 6, 2, temperatureTemp);
        client.publish(MQTT_LOCATION "/temperature", temperatureTemp);  // and publish it to MQTT

        // print results array to console
        #ifdef DEBUG_OUT
            for(int i = 0; i < 11; i++)
                {
                Serial.print(i);
                Serial.print(": ");
                Serial.println(temperature[i]);
                }
        #endif

        // find highest and lowest values (outliers)
        int hi = 0, lo = 0;
        for(int i = 1; i <= 9; i++)
            {
            if(temperature[i] > temperature[hi])
                {
                hi = i;
                }
            if(temperature[i] < temperature[lo])
                {
                lo = i;
                }
            }
        // reject outliers
        temperature[hi] = 0;
        temperature[lo] = 0;

        // calc average of 11 values
        float averageTemperature = 0;
        for(int i = 0; i < 11; i++)
            {
            averageTemperature += temperature[i];
            }
        averageTemperature /= 11;

        #ifdef DEBUG_OUT
            Serial.println();
            Serial.print("average: ");
            Serial.println(averageTemperature);
            Serial.println();
        #endif

        combSort11(temperature, 11);
        // print results to console
        #ifdef DEBUG_OUT
            for(int i = 0; i < 11; i++)
                {
                Serial.print(i);
                Serial.print(": ");
                Serial.println(temperature[i]);
                }
        #endif
        float medianTemperature = temperature[5];

        // publish median temperature to mqtt
        dtostrf(medianTemperature, 6, 2, temperatureTemp);
        client.publish(MQTT_LOCATION "/medianTemperature", temperatureTemp);

        #ifdef DEBUG_OUT
            Serial.println();
            Serial.print("median: ");
            Serial.println(medianTemperature);
            Serial.println();
        #endif
*/