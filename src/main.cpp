#define MCU "Wemos Lolin D1 Mini Pro V2"                                                            // MCU Hardware
#define PROGNAM "ESP8266_WeatherStation_MQTT"                                                       // program name
#define VERSION "v04.19"                                                                            // program version (nb lowercase 'version' is keyword)
#define PGMFUNCT "Temperature, Humidity, Pressure, Light Intensity, Wind, CO2"                           // what the program does
#define HARDWARE "Wemos D1 mini or pro with sensors and shields"                                    // hardware version
#define AUTHOR "J Manson"                                                                           // created by
#define CREATION_DATE "31 August 2020"                                                                 // date
#define DEBUG_OUT
                                                                                                    // https://stackoverflow.com/questions/47346133/how-to-use-a-define-inside-a-format-string
#define _STRINGIFY(x) #x                                                                            // this converts to string
#define STRINGIFY(x) _STRINGIFY(x)                                                                  // this makes sure the argument is expanded before converting to string
                                                                                                    // used to convert #define PUB_SUB_CLIENT esp30client to a string in id() function

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include "NDIR_I2C.h"                                                                               // CO2 sensor library https://github.com/SandboxElectronics/NDIR http://sandboxelectronics.com/?product=mh-z16-ndir-co2-sensor-with-i2cuart-5v3-3v-interface-for-arduinoraspeberry-pi
#include "SimpleTimer.h"                                                                            // https://playground.arduino.cc/Code/SimpleTimer/ https://github.com/marcelloromani/arduino/tree/master/SimpleTimer
#include "WEMOS_SHT3X.h"                                                                            // Wemos Temperature and Humidity shield library
#include "PubSubClient.h"                                                                           // https://github.com/knolleary/pubsubclient
                                                                                                    // NB modified MQTT_MAX_PACKET_SIZE in PubSubClient.h from 128 to 384
                                                                                                    // see https://www.hivemq.com/blog/mqtt-client-library-encyclopedia-arduino-pubsubclient/
//#include "SoftwareSerial.cpp"                                                                       // https://github.com/plerup/espsoftwareserial
                                                                                                    // NB ESP needs this version of SoftwareSerial !! (and needs to be in PLatform IO lib)
#include "RBDdimmer.h"                                                                              // https://github.com/RobotDynOfficial/RBDDimmer
#include "ArduinoJson.h"                                                                            // https://github.com/bblanchon/ArduinoJson

// MQTT code from: https://randomnerdtutorials.com/raspberry-pi-publishing-mqtt-messages-to-esp8266/
//                 https://github.com/knolleary/pubsubclient

// Functions
//
// void reconnect()                                                     This functions reconnects your ESP8266 to your MQTT broker
// void setup_wifi()
// void combSort11(float *ar, uint8_t n)
// void readSensors()                                                   Read the sensors and publish to MQTT. called by timer @ updateFreq
// void callback(String topic, byte* message, unsigned int length)      This functions is executed when a device publishes a message to a topic that an ESP8266 is subscribed to
// void id()                                                            create Json object containing ID data and publish to mqtt broker
// void setup()
// void loop()



// ------------------------------------------------------------------
// Pins

// ------------------------------------------------------------------
// Calibrate CO2 Sensor (self-calibration disabled in setup())
// Re-calibration of the sensor should be done in fresh air. After power on the sensor and put it in fresh air for at least 5 minutes.
// press the calibration button for at least 10 seconds
// ------------------------------------------------------------------

/* Changelog
04.00 average 10 samples
04.02 add routines to change update frequency from Node RED via MQTT
04.03 temporarily disable the averaging routine
04.04 fix esp8266 broadcasting unwanted open wifi network (in setup_wifi())
04.05 add credentials for BTHub6-7N5K
04.06 fix routines to change update frequency from Node RED via MQTT
04.07 adding RS485 wind sensors
04.08 winds peed implemented, publishing to MQTT
04.09 wind direction implemented
04.10 refactor
04.11 add CRC-16/MODBUS checksum function
04.12 adding CO2 sensor
04.14 turn onboard LED OFF by default (in setup)
04.17 add mySensor.disableAutoCalibration() in setup() for CO2 sensor
04.17 add filter for outlier rejection
*/

/*
ToDo:
CRC checksum for wind direction
set limits for sensor updateFreqs based on response times
remove averaging and median filters. just send raw data at updateFreq
(raw data to database, smoothing in Node RED)
WiFi reconnect routine
*/

// ------------------------------------------------------------------
// unique number for each ESP8266 device and edit the next 2 #defines accordingly
#define MQTT_DEVICE "esp30"                                                                         // MQTT requires unique device ID (see reconnect() function)
#define PUB_SUB_CLIENT esp30client                                                                  // and unique client ?
#define MQTT_LOCATION "test"                                                                    // location for MQTT topic
#define LED_STARTS_OFF                                                                            // initial state of on-board LED
//#define UPDATE_FREQ 60000L                                                                          // 60 seconds

long updateFreq = 0;                                                                                // the update frequency for sensors and publish to MQTT
int timerID;

// ------------------------------------------------------------------
// MQTT
const char* mqtt_server = "192.168.1.100";
const int mqttPort = 1883;
const char* mqttUser = "mqttUser2";
const char* mqttPassword = "jR5b73Wklx";

/*
const char* mqtt_server = "192.168.1.74";
const int mqttPort = 1883;
const char* mqttUser = "mqttUser";
const char* mqttPassword = "hTR7gxBY4";
*/

// ------------------------------------------------------------------
// Comment out sensors not in use

#define WEMOS_SHT30 "SHT30 "                                                                              // Wemos Temperature and Humidity shield
//#define WEMOS_HP303 "HP303 "                                                                                // Wemos Barometric Pressure Shield
//#define WEMOS_BH1750 "BH1750 "                                                                               // Wemos Ambient Light Shield
//#define WEMOS_BATTERY "BATTERY "                                                                              // Wemos Battery Shield
//#define RSSI "RSSI "                                                                                       // ESP8266 WiFi RSSI
//#define RS485_WIND "WIND "                                                                                 // Anemometer and Wind Direction
//#define CO2 "CO2 "                                                                                        // sandbox electronics CO2 sensor - NB recalibrate in fresh air regularly
//#define ROBOTDYN_ACDIMMER "ACDIMMER "                                                                          // Robotdyn zero-crossing detector and AC dimmer module

// -----------------------------------------------------------------
// define which WiFi network to connect to (only 1 should be active)

#define BTHUB6
//#define BTHUBKBHSE              // NB not useable with this sketch because no MQTT Server on KBHSE Network
//#define LINKSYS

#ifdef BTHUB6                                                                                       // Weights Room
    const char* ssid = "BTHub6-7N5K";                                                               // new BTHub WiFi credentials
    const char* password = "QeC3RCJGeUvx";
#endif

#ifdef BTHUBKBHSE
    const char* ssid = "BT-HZA8HJ";                                                               // KBHSE BTHub WiFi credentials
    const char* password = "fyurGVup6qru3H";
#endif

#ifdef LINKSYS                                                                                      // Garage
    const char* ssid = "linksys";                                                                   // BTHub WiFi credentials
    const char* password = "rhenigidale";
#endif

// -----------------------------------------------------------------
// Initializes the espClient. You should change the espClient name if you have multiple ESPs running in your home automation system
//WiFiClient espClient;
WiFiClient PUB_SUB_CLIENT;
PubSubClient client(PUB_SUB_CLIENT);

// Lamp - LED - GPIO 4 = D2 on ESP-12E NodeMCU board
const int lamp = LED_BUILTIN;

// ------------------------------------------------------------------

#ifdef CO2
    #include <NDIR_I2C.h>
    NDIR_I2C mySensor(0x4D);                                  //Adaptor's I2C address (7-bit, default: 0x4D)
#endif

#ifdef WEMOS_SHT30
    #include "WEMOS_SHT3X.h"                                                                       // Wemos Temperature and Humidity shield
    SHT3X sht30(0x45);                                                                              // SHT30 shield has two user selectable I2C addresses
    //static char sht_temperature[7];
    //static char sht_humidity[7];
#endif

#ifdef WEMOS_HP303
    #include "LOLIN_HP303B.h"                                                                       // Wemos Barometric Pressure HP303B Shield https://github.com/wemos/LOLIN_HP303B_Library
    LOLIN_HP303B HP303B;
#endif

#ifdef WEMOS_BH1750                                                                                 // Wemos Ambient Light Shield
    #include "BH1750.h"                                                                             // ambient light sensor https://github.com/claws/BH1750
    BH1750 lightMeter(0x23);                                                                        // BH1750 can be configured to use two I2C addresses, 0x23 (default)or 0x5C
#endif

#ifdef RS485_WIND
    //const byte anemometerInquiryFrame[] = {0x01, 0x03, 0x00, 0x16, 0x00, 0x01, 0x65, 0xCE};            // Anemometer Inquiry frame
    //const byte windDirectionInquiryFrame[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39};         // Wind Direction Sensor Inquiry frame
    byte responseFrameBuff[7] = {};                                                                    // buffer for receiving response frame from sensors
    //long updateFreq;                                                                                   // sensor update frequency (millis)
    float windSpeed;

    #define rxPin D7                                                                                  // SoftwareSerial receive pin
    #define txPin D6                                                                                  // SoftwareSerial transmit pin
    #define rtsPin D5                                                                                 // RS485 RTS direction control
    #define transmit HIGH
    #define receive LOW
    #define ledPin D4

    SoftwareSerial RS485(rxPin, txPin);                                                                // instantiate SoftwareSerial object

    // anemometer type B - not used
    // const byte anemometerInquiryFrame[] = {0x01, 0x03, 0x00, 0x16, 0x00, 0x01, 0x65, 0xCE};            // Anemometer Inquiry frame

    /*
    Anemometer on address 0x03
                              bytes
    device address:             1   0x03
    function number:            1   0x03
    start register address:     2   0x0000
    number of registers:        2   0x0001
    CRC-16/MODBUS               2   0x85E8
    */
    const byte anemometerInquiryFrame[] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x01, 0x85, 0xE8};            // Wind Speed Sensor Inquiry frame
    /*
    the response frame should be:
                              bytes
    device address:             1   0x03
    function number:            1   0x03
    number of valid bytes       1   0x02
    data:                       2   0x0008
    CRC-16/MODBUS               2   0xC042          ** check this!

    data / 10 => speed m/s (0.8 m/s)

    range:              0 - 32.4 m/s
    precision:          0.3 m/s
    start speed:        0.8 m/s
    */

    /*
    Wind Direction Sensor on address 0x02
    */
    const byte windDirectionInquiryFrame[] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x39};         // Wind Direction Sensor Inquiry frame

    /*
    Modbus Device address can be set to 0-255. You can set a new address by transmitting a frame
    with the first byte (the device address) set to 0x00. **Only do this with 1 device on the bus!
    Eg to set a device to address 3:
                          bytes
    device address:             1   0x00
    function number:            1   0x16
    start register address:     2   0x1000
    number of registers:        2   0x0001
    number of valid bytes       1   0x02
    SET new device address:     2   0x0003
    CRC-16/MODBUS               2   0x7A2A

    const byte modifyDeviceAddressFrame[] = {0x00, 0x16, 0x10, 0x00, 0x00, 0x01, 0x02, 0x00, 0x03, 0x7A, 0x2A};

    CRC-16/MODBUS online calculator: https://crccalc.com/
    the CRC is sent little-endian (low byte first)
    https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16

    the response frame should be:
                              bytes
    device address:             1   0x00
    function number:            1   0x16
    start register address:     2   0x1000
    number of registers:        2   0x0001
    CRC-16/MODBUS               2   0x8CD8
    */

#endif

#ifdef ROBOTDYN_ACDIMMER
    #define dimmerPWMPin  D7
    #define zerocross  D6
    dimmerLamp dimmer(dimmerPWMPin, zerocross);                                                    // the class is dimmerLamp
    int outVal = 0;                                                                                // PWM Output value (%)
#endif

// ------------------------------------------------------------------

SimpleTimer timer;

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
            client.subscribe("update/Freq/" MQTT_LOCATION);
            client.subscribe("IDRequest");
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
    // the next line to fix esp8266 broadcasting unwanted open wifi network
    // it is placed AFTER WiFi.begin() !!
    // see: https://forum.arduino.cc/index.php?topic=501263.0 and https://github.com/esp8266/Arduino/issues/549
    WiFi.mode(WIFI_STA);
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

void combSort11(float *ar, uint8_t n)                                                               // this was used for median when temp samples stored in array
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
#ifdef RS485_WIND
    // CRC-16/MODBUS calc function
    // https://stackoverflow.com/questions/19347685/calculating-modbus-rtu-crc-16

    uint16_t CRC16 (const byte *nData, uint16_t wLength)
        {
        static const uint16_t wCRCTable[] = {
        0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
        0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
        0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
        0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
        0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
        0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
        0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
        0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
        0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
        0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
        0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
        0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
        0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
        0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
        0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
        0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
        0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
        0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
        0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
        0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
        0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
        0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
        0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
        0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
        0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
        0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
        0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
        0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
        0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
        0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
        0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
        0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040 };

        byte nTemp;
        uint16_t wCRCWord = 0xFFFF;

        while (wLength--)
            {
            nTemp = *nData++ ^ wCRCWord;
            wCRCWord >>= 8;
            wCRCWord  ^= wCRCTable[(nTemp & 0xFF)];
            }
        return wCRCWord;
        } // End: CRC16
#endif

// ------------------------------------------------------------------
// read the sensors and publish to MQTT
// called by timer @ updateFreq

void readSensors()
    {                                                                                               // function: reads sensors and sends data to blynk app
	
    #ifdef CO2
        //int ppmReading;                                           
        #ifdef DEBUG_OUT
            Serial.println(F("Reading the CO2 sensor: "));
        #endif
        mySensor.measure();
        uint16_t ppmReading = mySensor.ppm;                                                         // CO2 in ppm
        static char CO2Temp[7];                                                                     // client.publish() expects char array
        itoa(ppmReading, CO2Temp, 10);
        client.publish(MQTT_LOCATION "/CO2", CO2Temp);
        #ifdef DEBUG_OUT
            Serial.print("CO2: ");
            Serial.print(ppmReading);
            Serial.println(" ppm");
        #endif
    #endif

    #ifdef WEMOS_SHT30                                                                              // temperature and humidity
        #ifdef DEBUG_OUT
            Serial.println(F("Reading the SHT30: "));
        #endif
        static char temperatureTemp[7];                                                             // client.publish() expects char array
        static char humidityTemp[7];
        float temperature = 0;
        float humidity = 0;
        /*
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
        dtostrf(averageTemperature, 6, 2, temperatureTemp);                                         // convert float to char array
        dtostrf(averageHumidity, 6, 2, humidityTemp);
        */
        sht30.get();
        temperature += sht30.cTemp;
        humidity += sht30.humidity;
        dtostrf(temperature, 6, 2, temperatureTemp);                                                // convert float to char array
        dtostrf(humidity, 6, 2, humidityTemp);

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
        HP303B.measurePressureOnce(pressure);                                                       // read the barometric pressure
        hPa = pressure / 100.0;                                                                     // convert to hPa (note 100.0 for cast to float)
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
        uint16_t lux = lightMeter.readLightLevel();                                                 // read the light level
        static char luxStr[5];
        itoa(lux, luxStr, 10);
        client.publish(MQTT_LOCATION "/light", luxStr);
    #endif
/*
    #ifdef RSSI
        int rssi = WiFi.RSSI();                                                                     // get RSSI (received signal strength indicator) of WiFi connection
        static char rssiStr[5];
        itoa(rssi, rssiStr, 10);
        client.publish(MQTT_LOCATION "/rssi", rssiStr);
    #endif

    #ifdef WEMOS_BATTERY
        int adc = analogRead(0);                                                                    // read the ESP ADC (connected to battery)
        float v = (adc * 4.335) / 1024;                                                             // calculate the voltage (fsd 4.335v)
        Blynk.virtualWrite(V47, v);                                                                 // send to app
    #endif
*/
    #ifdef RS485_WIND
        word checksum;
        word crc;
        do
            {
            digitalWrite(rtsPin, transmit);                                        // init transmission
            RS485.write(anemometerInquiryFrame, sizeof(anemometerInquiryFrame));   // send the Inquiry
            RS485.flush();                                                         // Wait for the transmission to complete

            digitalWrite(rtsPin, receive);                                         // init receive
            RS485.readBytes(responseFrameBuff, 7);                                 // read the data

            checksum = CRC16 (responseFrameBuff, 5);                               // calculate CRC-16/MODBUS checksum
            crc = responseFrameBuff[6] * 256 + responseFrameBuff[5];               // get checksum from buffer
            Serial.print("+");
            }
        while (checksum != crc);                                                   // test if checksum is ok
        Serial.println();
        
        static char speedTemp[6];                                                  // client.publish() expects char array
        int speed = responseFrameBuff[3] * 256 + responseFrameBuff[4];             // high low bytes
        itoa(speed, speedTemp, 10);                                                                    // convert float to char array
        client.publish(MQTT_LOCATION "/windSpeed", speedTemp);                                         // publish to MQTT, topic /windSpeed

        #ifdef DEBUG_OUT
            Serial.print("wind speed: ");
            for(byte i = 0; i < 7; i++)
                {
                Serial.print(responseFrameBuff[i], HEX);
                Serial.print(" ");
                }
            Serial.print(" ==> ");
            windSpeed = responseFrameBuff[4] / 10.0;
            Serial.print(windSpeed);
            Serial.print(" m/s");
            Serial.print(" checksum = ");
            Serial.print(checksum);
            if (crc == checksum)
                {
                Serial.println(" - OK");
                }
            else
                {
                Serial.println(" - WRONG");
                }
            
        #endif

        delay(100);   // needs delay here - why? duration?

        digitalWrite(rtsPin, transmit);                                                            // init transmission
        RS485.write(windDirectionInquiryFrame, sizeof(windDirectionInquiryFrame));                 // send the Inquiry
        RS485.flush();                                                                             // Wait for the transmission to complete

        digitalWrite(rtsPin, receive);                                                             // init receive
        RS485.readBytes(responseFrameBuff, 7);                                                     // read the data

        static char directionTemp[2];                                                              // client.publish() expects char array
        int direction = responseFrameBuff[4];                                                      // low byte
        itoa(direction, directionTemp, 10);                                                        // convert float to char array
        client.publish(MQTT_LOCATION "/windDirection", directionTemp);                             // publish to MQTT, topic /windSpeed

        digitalWrite(ledPin, HIGH);                                                                // flash the LED
        delay(100);
        digitalWrite(ledPin, LOW);

    #endif

  	} // end of readSensors()

// ------------------------------------------------------------------
void id()                                                                                          // create Json object containing ID data and publish to mqtt broker
                                                                                                   // https://techtutorialsx.com/2017/04/29/esp32-sending-json-messages-over-mqtt/
                                                                                                   // https://github.com/bblanchon/ArduinoJson/blob/6.x/examples/JsonGeneratorExample/JsonGeneratorExample.ino
                                                                                                   // https://arduinojson.org/v6/doc/serialization/
                                                                                                   // PubSubClient: As part of minimising its footprint, it limits the size of any MQTT packet it can send or receive to 128 bytes.
                                                                                                   // If you want to send or receive messages larger than this, you must change the value of MQTT_MAX_PACKET_SIZE in PubSubClient.h
                                                                                                   // The library allocates this much memory in its internal buffer, which reduces the memory available to the sketch itself.
                                                                                                   // https://www.hivemq.com/blog/mqtt-client-library-encyclopedia-arduino-pubsubclient/
    {

    //StaticJsonBuffer<300> JSONbuffer;
    //JsonObject& JSONencoder = JSONbuffer.createObject();

    StaticJsonDocument<400> doc;

    doc["MCU"] = MCU;
    doc["PROGNAM"] = PROGNAM;
    doc["VERSION"] = VERSION;
    Serial.println(WiFi.localIP());
    doc["IP"] = WiFi.localIP().toString();
    doc["ROUTER"] = ssid;
    doc["MQTT_LOCATION"] = MQTT_LOCATION;
    doc["MQTT_DEVICE"] = MQTT_DEVICE;
    String pubSubClient = STRINGIFY(PUB_SUB_CLIENT);                                                        // used to convert PUB_SUB_CLIENT #define to a string  https://stackoverflow.com/questions/47346133/how-to-use-a-define-inside-a-format-string
    Serial.println(pubSubClient);
    doc["PUB_SUB_CLIENT"] = pubSubClient;
    doc["UPDATE_FREQ"] = updateFreq;

    String sensors;
    #ifdef WEMOS_SHT30
        sensors += WEMOS_SHT30;
    #endif
    #ifdef WEMOS_HP303
        //sensors += ", ";
        sensors += WEMOS_HP303;
    #endif
    #ifdef WEMOS_BH1750
        sensors += WEMOS_BH1750;
    #endif
    #ifdef BATTERY
        sensors += BATTERY;
    #endif
    #ifdef RSSI
        sensors += RSSI;
    #endif
    #ifdef RS485_WIND
        sensors += RS485_WIND;
    #endif
    #ifdef CO2
        sensors += CO2;
    #endif
    #ifdef ROBOTDYN_ACDIMMER
        sensors += ROBOTDYN_ACDIMMER;
    #endif
    doc["SENSORS"] = sensors;

    // Generate the minified JSON and send it to the Serial port
    serializeJson(doc, Serial);
    Serial.println();
    // Generate the prettified JSON and send it to the Serial port
    serializeJsonPretty(doc, Serial);
    Serial.println();

    // Declare a buffer to hold the result
    char jsonBuff[256];
    // Produce a minified JSON document
    serializeJson(doc, jsonBuff);
    Serial.println(jsonBuff);
    //publish to MQTT
    client.publish("ID", jsonBuff);

    }
    // end of id()

// ------------------------------------------------------------------
// This functions is executed when a device publishes a message to a topic that an ESP8266 is subscribed to
// (subscriptions are specified in reconnect() function)
// Change the function below to add logic to the program, so when a device publishes a message to a topic that 
// an ESP8266 is subscribed to it can do something
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
    // NB need to subscribe to topics in reconnect() !!

    //if(topic == MQTT_LOCATION "/freq")
    if(topic == "update/Freq/" MQTT_LOCATION)
        {
        updateFreq = messageTemp.toInt();
        Serial.print("freq: ");
        Serial.println(updateFreq);
        if(timer.isEnabled(timerID))                                                                // if timer has already been started (in setup())
            {
            timer.deleteTimer(timerID);
            timerID = timer.setInterval(updateFreq, readSensors);                                   // restart timer with new updateFreq
            }
        }

    if(topic == MQTT_LOCATION "/lamp")
        {
        Serial.print("Changing Room lamp to ");
        if(messageTemp == "on")
            {
            digitalWrite(lamp, LOW);                                                                // active LOW
            Serial.print("On");
            }
        else if(messageTemp == "off")
            {
            digitalWrite(lamp, HIGH);
            Serial.print("Off");
            }
        }

    if(topic == MQTT_LOCATION "/fanTempSetpoint")
        {
        //= messageTemp.toInt();


        }

    if(topic == "IDRequest")
        {
        id();
        }

    Serial.println();
    }   // end of callback()

// ------------------------------------------------------------------



// ------------------------------------------------------------------
void setup()
    {

    Serial.begin(115200);
    Serial.printf("\n\n%s_%s, %s, %s, %s, %s\n\n", PROGNAM, VERSION, AUTHOR, CREATION_DATE, MQTT_DEVICE, MQTT_LOCATION);

    pinMode(lamp, OUTPUT);
    #ifdef LED_STARTS_OFF
        digitalWrite(lamp, HIGH);                                                                      // turn onboard LED OFF
    #endif

    setup_wifi();
    client.setServer(mqtt_server, mqttPort);
    client.setCallback(callback);

    #ifdef CO2
        if (mySensor.begin())
            {
            Serial.println();
            Serial.println("Wait 10 seconds for CO2 sensor initialization...");
            delay(10000);
            }
        else
            {
            Serial.println("ERROR: Failed to connect to the sensor.");
            while(1);
            }
        mySensor.disableAutoCalibration();                                                         // stop the sensor self-calibration function (it expects sensor to be in fresh air at some point in a 24 hour period - which it isn't)
    #endif

    #ifdef WEMOS_HP303                                                                              // Wemos Barometric Pressure HP303B Shield
        HP303B.begin();                                                                             // I2C address = 0x77 (or HP303B.begin(0x76); //I2C address = 0x76)
    #endif

    #ifdef WEMOS_BH1750
        if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE))                                     // begin returns a boolean that can be used to detect setup problems
            {
            Serial.println(F("BH1750 Advanced begin"));
            }
        else
            {
            Serial.println(F("Error initialising BH1750"));
            }
    #endif

    #ifdef RS485_WIND
        pinMode(rtsPin, OUTPUT);
        pinMode(ledPin, OUTPUT);
        RS485.begin(9600);
        delay(1000);
    #endif

    #ifdef ROBOTDYN_ACDIMMER
        dimmer.begin(NORMAL_MODE, ON);                      //dimmer initialisation: name.begin(MODE, STATE) 
    #endif
    
    if (!client.connected())
        {
        reconnect();
        }

    // request the update frequency (from Node RED)
    //client.publish(MQTT_LOCATION "/update", "freq");
    client.publish( "update/getFreq", MQTT_LOCATION);

    // wait for MQTT to publish the update frequency
    Serial.println("waiting for MQTT to publish the update frequency");
    while (updateFreq == 0) 
        {
        Serial.print(".");
        delay(500);
        client.loop();
        //yield();
        }
    //Serial.print("freq: ");
    //Serial.println(updateFreq);
 
    // start the timer
    timerID = timer.setInterval(updateFreq, readSensors);                                           // every x milliseconds, send data to MQTT

    Serial.print("timerID: ");
    Serial.println(timerID);

    id();

    } // end of setup()

// ------------------------------------------------------------------
void loop()
    {

    if (!client.connected())
        {
        reconnect();
        }
    if (!client.loop())
        {
        client.connect(MQTT_DEVICE);
        }

    timer.run();

    //outVal = map(analogRead(A0), 1, 1024, 20, 100);                                     // read potentiometer, analogRead(analog_pin), min_analog, max_analog, 20%, 100%);
    //dimmer.setPower(outVal); 

    } // end of loop()

// ------------------------------------------------------------------


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