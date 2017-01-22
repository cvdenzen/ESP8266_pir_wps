// program WiFi_pir_wps.ino, Arduino 1.6.12
/*
  WiFiTelnetToSerial - Example Transparent UART to Telnet Server for esp8266
  Copyright (c) 2015 Hristo Gochkov. All rights reserved.
  This file is part of the ESP8266WiFi library for Arduino environment.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
    2017-01-17 Carl van Denzen, pir sensor added
    Scan wps button to allow an already connected esp-01 to a new network, while
    the old network is still available.
    pir sensor added
    pull-up resistors reviewed

    2016-10-17 Rudolf Reuter, push button switch for WPS function added.
    http://www.wi-fi.org/knowledge-center/faq/how-does-wi-fi-protected-setup-work

    Once the WiFi credentials are WPS fetched, they are stored,
    for following use (firmware function).

    Works on ESP-01 module - Arduino Tools:
        Board: "Generic ESP8266 Module"
        Flash Mode: "DIO"
        Flash Fequency: "40 MHz"
        CPU Frequency: "80 MHz"
        Flash Size: "512K (64K SPIFFS)"
        Debug port: "Disabled"
        Reset Method: "ck"
        Upload Speed: "115200"

    Connection of ESP8266 GPIO0 and GPIO2 for WPS button and LED indicator:
                          Vcc = 3.3 V
                           |
                          1k0  pull up resistors (3K3 to 10K Ohm)
                           |
                     +-|<|-+   LED red
                     |
                     |
                     |
   GPIO0  - +--------+-------o |
                               | -> | push button switch for WPS function
   GPIO2  --+            +---o |
            |            |
            |           GND
            |
            |              +--------+
            +--------------| pir    | (with internal 1k resistor)
                           +--------+





   from http://www.forward.com.au/pfod/ESP8266/GPIOpins/index.html

   /!\ There is a '''5 minute''' timeout in the ESP8266 '''!WiFi stack'''.
   That means, if you have after the first WLAN connection a data traffic pause
   longer than '''5 minutes''', you to have to resync the connection
   by sending some dummy characters.
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Timer.h"

bool debug = false;  // enable either one
//bool debug = true:

// Timer
Timer timer;
// timer id's
int timer_wps_button = -1;
int timer_pir_sensor = -1;
// mqtt
WiFiClient wifiClient;
char* mqtt_server = "mqtt_server";
char* mqtt_topic = "bg/wc/pir";
// see https://github.com/arduino/arduino-builder/issues/85
void mqtt_callback(char* topic, uint8_t* payload, unsigned int length);
PubSubClient mqtt_client(mqtt_server, 1883, mqtt_callback, wifiClient);


// The callback method for received mqtt messages
void mqtt_callback(char* topic, uint8_t* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    char receivedChar = (char)payload[i];
    Serial.print(receivedChar);
    // Carl if (receivedChar == '0')
    // ESP8266 Huzzah outputs are "reversed"
    // Carl digitalWrite(ledPin, HIGH);
    // Carl if (receivedChar == '1')
    // Carl digitalWrite(ledPin, LOW);
  }
  Serial.println();
}

bool startWPSPBC() {
  // from https://gist.github.com/copa2/fcc718c6549721c210d614a325271389
  // wpstest.ino
  Serial.println("WPS config start");
  bool wpsSuccess = WiFi.beginWPSConfig();
  if (wpsSuccess) {
    // Well this means not always success :-/ in case of a timeout we have an empty ssid
    String newSSID = WiFi.SSID();
    if (newSSID.length() > 0) {
      // WPSConfig has already connected in STA mode successfully to the new station.
      Serial.printf("WPS finished. Connected successfull to SSID '%s'\n", newSSID.c_str());
    } else {
      wpsSuccess = false;
    }
  }
  return wpsSuccess;
}
void setup() {
  Serial.begin(9600);        // adopt baud rate to your needs
  //Serial.begin(115200);
  delay(1000);
  if (debug) {
    Serial.println("\n WPS with push button on GPIO2 input, LOW = active");
    Serial.printf("\nTry connecting to WiFi with SSID '%s'\n", WiFi.SSID().c_str());
  }

  pinMode(0, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  // Start polling wps
  timer_wps_button = timer.every(10, poll_wps);


  WiFi.mode(WIFI_STA); // Station, not AP Access Point
  WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str()); // reading data from EPROM,
  while (WiFi.status() == WL_DISCONNECTED) {          // last saved credentials
    delay(500);
    if (debug) Serial.print(".");
  }

  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    if (debug) Serial.printf("\nConnected successful to SSID '%s'\n", WiFi.SSID().c_str());
  } else {
    Serial.printf("\nCould not connect to WiFi. state='%d'\n", status);
    Serial.println("Please press WPS button on your router, until mode is indicated.");
    Serial.println("next press the ESP module WPS button, router WPS timeout = 2 minutes");

    //
    // connect to mqtt server
    //
    mqtt_setup();


    // Start polling pir sensor
    timer_pir_sensor = timer.every(10, poll_pir);
  }
  // Try mqtt.<localdomain> as server name, see https://github.com/mqtt/mqtt.github.io/wiki/broker_auto-discovery

  mqtt_client.setServer(mqtt_server, 1883);
  mqtt_client.setCallback(mqtt_callback);
}

// from Arduino/hardware/esp8266com/esp8266/libraries/ESP8266WiFi/examples/
// (2015-11-15)

void loop() {
  timer.update(); // check to see whether there is anything to do
  delay(1); // give other processes (WiFi) a chance to run, and potentially reduce power consumption
}
/*
   Start polling sequence:

*/
// Exposed values
boolean wps_button = false;
boolean pir_sensor = false;
// Internal exposed values (pin readings)
int wps_button_internal = HIGH; // initially not active
int pir_button_internal = LOW; // initially not active
// variables used to implement debounce
int wps_debounce_count = 0;
int pir_debounce_count = 0;
//
// Poll sequence
//
void poll_wps() {
  // Low is active
  int raw_button;
  raw_button = digitalRead(0); // wps_button
  if (raw_button != wps_button_internal) {
    if (++wps_debounce_count > 5) {
      // definitely a change, 5 times same value, so expose new value
      wps_button_internal = raw_button;
      if (wps_button_internal == LOW) {
        // active
        wps_button = true;
      } else {
        wps_button = false;
      }
      wps_changed();
    }
  } else {
    // same value, reset debounce
    wps_debounce_count = 0;
  }
}
//
// Poll sequence
//
void poll_pir() {
  // pir sensor
  int raw_pir;
  raw_pir = digitalRead(2); // pir_sensor
  if (raw_pir == HIGH) {
    // active
    pir_sensor = true;
  } else {
    pir_sensor = false;
  }
}

void wps_changed() {
  if (wps_button) {
    // wps_button is active
    if (!startWPSPBC()) {
      Serial.println("Failed to connect with WPS :-(");
    } else {
      WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str()); // reading data from EPROM,
      while (WiFi.status() == WL_DISCONNECTED) {          // last saved credentials
        delay(500);
        Serial.print("."); // show wait for connect to AP
      }
      pinMode(0, INPUT);   // GPIO0, LED OFF, show WPS & connect OK
    }
  }
}
//
// mqtt part
//

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

void mqtt_setup() {
  // Generate client name based on MAC address and last 8 bits of microsecond counter
  String clientName;
  clientName += "esp8266-";
  uint8_t mac[6];
  WiFi.macAddress(mac);
  clientName += macToStr(mac);
  clientName += "-";
  clientName += String(micros() & 0xff, 16);

  Serial.print("Connecting to ");
  Serial.print(mqtt_server);
  Serial.print(" as ");
  Serial.println(clientName);

  if (mqtt_client.connect((char*) clientName.c_str())) {
    Serial.println("Connected to MQTT broker");
    Serial.print("Topic is: ");
    Serial.println(mqtt_topic);

    if (mqtt_client.publish(mqtt_topic, "hello from ESP8266")) {
      Serial.println("Publish ok");
    }
    else {
      Serial.println("Publish failed");
    }
  }
  else {
    Serial.println("MQTT connect failed");
    Serial.println("Will reset and try again...");
    abort();
  }
}

void callback(char* topic, byte * payload, unsigned int length) {
  // handle message arrived
}


