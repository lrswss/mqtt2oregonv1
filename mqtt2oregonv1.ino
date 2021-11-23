/*
 * mqtt2oregonv1 gateway for ESP8266 or ESP32
 * 
 * (c) 2021 Lars Wessels <software@bytebox.org>
 *   
 * Arduino sketch to emulate an Oregon V1 sensor (433 MHz) 
 * to transmit temperature readings received via mqtt to be
 * displayed on an Oregon compatible weather station device.
 * 
 * Published under MIT license.
 * 
 */

#include <PubSubClient.h>
#include <ArduinoJson.h>
#if defined(ESP32)
#include <WiFi.h>
#else
#include <ESP8266WiFi.h>
#endif


#define MQTT_BROKER "<mqtt-broker-ip>"
#define MQTT_TOPIC "oregonv1"

#define WIFI_SSID "<your-ssid>"
#define WIFI_PASS "<wlan-password>"

// set output pin for 433MHz transmitter (e.g. FS1000A)
#define TX_PIN 4


// macros to set bits in payload based on 8 nibbles (32 bit)
// http://wmrx00.sourceforge.net/Arduino/OregonScientific-RF-Protocols.pdf
#define SET_ADDR(x)  (x & 0x07) << 0
#define SET_CH(x)    ((x - 1) & 0x03) << 6
#define SET_TEMP(x)  (((uint32_t)x / 100) % 10) << 16 | (((uint32_t)x / 10) % 10) << 12 | (x % 10) << 8
#define SET_CRC(x)   (uint32_t)x << 24


// struct for Oregon V1 data frame (excl. checksum)
typedef struct {
    uint8_t addr;
    uint8_t ch;
    float temp;
    bool lowbat;
} thn128_t;


// setup wifi and mqtt client
WiFiClient espClient;
PubSubClient mqtt(espClient);


// default data update in mqtt callback function
thn128_t sensor_data = {
    .addr = 4,        // rolling address
    .ch = 1,          // channels 1,2,3
    .temp = 21.0,     // temperature in degrees celcius
    .lowbat = false,  // true if battery is low
};


// (re)connect to wifi
void connect_wifi() {
    delay(100);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(F("."));
    }
    Serial.print(F("IP "));
    Serial.print(WiFi.localIP());
    Serial.print(F(" (RSSI "));
    Serial.print(WiFi.RSSI());
    Serial.println(F(" dBm)."));
}


// called if mqtt messages arrive we've previously subscribed to
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<256> json;  // https://arduinojson.org/v6/assistant/
   
    Serial.print(F("Received message on /"));
    Serial.print(topic);
    Serial.print(" ");
    for (uint8_t i = 0; i < length; i++)
        Serial.print((char)payload[i]);
    Serial.println();

    DeserializationError error = deserializeJson(json, payload);
    if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
    } else {
        sensor_data.ch = json["channel"];
        sensor_data.addr = json["address"];
        sensor_data.temp = json["temperature"];
        sensor_data.lowbat = json["lowbattery"];
    }
}


// connect to mqtt broker with random client id and subscribe 
// to sensor topic we want to display on 433MHz weather display
void connect_mqtt() {
    static char clientid[16];
    static uint8_t mqtt_error = 0;
    
    while (!mqtt.connected()) {
        // generate pseudo random client id
        snprintf(clientid, sizeof(clientid), "THN128_%x", random(0xffff));
    
        Serial.print(F("Connecting to MQTT Broker ")); 
        Serial.print(MQTT_BROKER); Serial.print(F(" as client "));
        Serial.print(clientid); Serial.print(F("..."));
    
        if (mqtt.connect(clientid)) {
            Serial.println(F("OK."));
            mqtt.subscribe(MQTT_TOPIC);
            mqtt_error = 0;      
        } else {
            Serial.print(F("failed("));
            Serial.print(mqtt.state());
            Serial.println(F("), try again in 5 seconds."));   
    
            // no mqtt connection within a minute => restart system  
            if (mqtt_error++ >= 12) {
                Serial.println(F("Auto reboot system..."));
                delay(3000);
                ESP.restart();
            }
            delay(5000);
        }
        Serial.println();
    }
} 


// calculate checksum for Oregon V1
uint8_t calc_crc(uint32_t data) {
    uint16_t crc;
    
    crc = ((data >> 16) & 0xFF) + ((data >> 8) & 0xff) + ((data >> 0) & 0xFF);
    crc = (crc >> 8) + (crc & 0xFF);
    return (uint8_t)crc;
}


// create payload (8 nibbles)
// (1) rolling address
// (1) channel
// (4) data - temperature
// (2) checksum
uint32_t encode_data(thn128_t *data) {
    uint32_t payload;

    payload = SET_ADDR(data->addr);  // rolling address (4bit)
    payload |= SET_CH(data->ch); // channel 1,2,3
    if (data->temp > -99 && data->temp < 99) {
        if (data->temp < 0) {
            payload |= (1UL << 21);
            payload |= SET_TEMP((int)(data->temp * -10));
        } else {
            payload |= SET_TEMP((int)(data->temp * 10));
        }
    }
    if (data->lowbat) // set low battery bit
        payload |= (1UL << 23);
    payload |= SET_CRC(calc_crc(payload)); // add checksum
    
    return payload;
}


// transmit sync pulse
// leading sync off 4200 ms
// sync pulse 5700 ms
// trailing sync off 5200 ms
void tx_sync() {
    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(4200);
    digitalWrite(TX_PIN, HIGH);
    delayMicroseconds(5700);
    digitalWrite(TX_PIN, LOW);
    delayMicroseconds(5200);
}


// send manchester bit pulse (342 Hz = 2923 micros)
void tx_bit(bool bit) {
    digitalWrite(TX_PIN, bit);
    delayMicroseconds(1450);
    digitalWrite(TX_PIN, !bit);
    delayMicroseconds(1475);
}


// send 12 bit preamble
void tx_preamble() {
    for (uint8_t i = 0; i < 12; i++) {
        delayMicroseconds(1250);
        digitalWrite(TX_PIN, HIGH);
        delayMicroseconds(1750);
        digitalWrite(TX_PIN, LOW);
    }
    delayMicroseconds(3000);
}


// encode and send 32 bit payload
void tx_data(uint32_t payload) {
    for (uint8_t i = 0; i < 32; i++) {
        if (payload & (1UL << i))
            tx_bit(true);
        else
            tx_bit(false);
    }
}


// transmit data twice including preamble and sync frame
void send_data(thn128_t *data) {
  
    Serial.print(F("Transmit RF433 "));
    Serial.printf("(Ch: %d, Addr: %d, Temp: ", data->ch, data->addr);
    Serial.print(data->temp);
    Serial.println("°C)...");

    for (uint8_t i = 0; i < 2; i++) {
        tx_preamble();
        tx_sync();
        tx_data(encode_data(data));
        digitalWrite(TX_PIN, LOW);
        delay(100);
    }
}


void setup() {
    pinMode(TX_PIN, OUTPUT);
    Serial.begin(115200);
    delay(250);
    
    Serial.println();
    Serial.print(F("Starting mqtt2oregonv1 gateway..."));
    Serial.println();
    
    Serial.print(F("Connecting to SSID "));
    Serial.print(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    connect_wifi();
    mqtt.setServer(MQTT_BROKER, 1883);
}


void loop() {
    static uint32_t lastLoopRun = 0;
    static uint32_t loopSecs = 0;
  
    if (WiFi.status() == WL_CONNECTED) { // check wifi connection status
        if (!mqtt.connected())
            connect_mqtt();
        mqtt.setCallback(mqtt_callback);
        mqtt.loop();
    } else {
        Serial.print(F("Reconnecting to SSID "));
        Serial.print(WIFI_SSID);
        connect_wifi();
    }

    if ((millis() - lastLoopRun) >= 1000) {
        lastLoopRun = millis();
        if ((loopSecs % 30) == 0) {  // transmit data every 30 secs.
            send_data(&sensor_data);
            //sensor_data.temp += 1;  // for debugging
        } else {
            Serial.printf("Waiting %d secs for next RF433 TX...\n", (loopSecs % 30));
        }
        loopSecs++;
    }
}
