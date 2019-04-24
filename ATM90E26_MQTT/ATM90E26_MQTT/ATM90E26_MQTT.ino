#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <energyic_UART.h>
#include <stdio.h>
#include <stdlib.h>

const char* ssid = "xxxxxxx";
const char* password =  "xxxxxxxxx";

const char* mqttServer = "xxxxxxxx";
const int mqttPort = 1883;
const char* mqttUser = "YourMqttUser";
const char* mqttPassword = "YourMqttUserPassword";

WiFiClient espClient;
PubSubClient client(espClient);

#if !defined(ARDUINO_ARCH_SAMD) && !defined(ESP32)
#include <SoftwareSerial.h>
#else

#endif

#if defined(ESP8266)
SoftwareSerial ATMSerial(D4, D3, false, 256);
#endif 

#ifdef __AVR_ATmega32U4__
SoftwareSerial ATMSerial(11, 13);
#endif 

#if defined(ARDUINO_ARCH_SAMD)
#include "wiring_private.h"
//Feather M0 
#define PIN_SerialATM_RX       12ul
#define PIN_SerialATM_TX       11ul
#define PAD_SerialATM_RX       (SERCOM_RX_PAD_3)
#define PAD_SerialATM_TX       (UART_TX_PAD_0)

Uart ATMSerial(&sercom1, PIN_SerialATM_RX, PIN_SerialATM_TX, PAD_SerialATM_RX, PAD_SerialATM_TX);
#endif

#if defined(ESP32)
#define PIN_SerialATM_RX       19   
#define PIN_SerialATM_TX       23 
HardwareSerial ATMSerial(1);
#endif

ATM90E26_UART eic(&ATMSerial);

void WifiAndMqtt()
{
   WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  digitalWrite(D7,1);
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
 
  while (!client.connected()) { 
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
      digitalWrite(D8,1);
 
    } else {
      digitalWrite(D8,0);
      delay(2000);
 
    }
  }
client.subscribe("esp/test");
  
  }


void setup() {
  Serial.begin(115200);
  #if defined(ARDUINO_ARCH_SAMD)
  pinPeripheral(PIN_SerialATM_RX, PIO_SERCOM);
  pinPeripheral(PIN_SerialATM_TX, PIO_SERCOM);
  #endif

  #if defined(ESP32)
  ATMSerial.begin(9600, SERIAL_8N1, PIN_SerialATM_RX, PIN_SerialATM_TX);
  #else
  ATMSerial.begin(9600);
  #endif
  
  eic.InitEnergyIC();
  delay(1000);
  pinMode(D8,OUTPUT);
  pinMode(D7,OUTPUT);
  WifiAndMqtt();
}


void callback(char* topic, byte* payload, unsigned int length) {
  
  for (int i = 0; i < length; i++) {
  }
}



void loop() {

  client.loop();
  char buffer [20];
  unsigned short s_status = eic.GetSysStatus();
  if(s_status == 0xFFFF)
  {
	#if defined(ESP8266)
     client.publish("READ", "0");
    ESP.restart();
	#endif
  }
  else{client.publish("READ", "1");}
  
  //Serial.println(eic.GetSysStatus(),HEX);
  //delay(10);
  //Serial.print("Meter Status:");
  //Serial.println(eic.GetMeterStatus(),HEX);
  //delay(10);
  
  #if defined(ESP32)
  sprintf(buffer, "%f", eic.GetLineVoltage());
  client.publish("VOLTAGE",buffer );
  #else
  sprintf(buffer, "%f", eic.GetLineVoltage());
  client.publish("VOLTAGE",buffer );
  #endif
  delay(10);
  #if defined(ESP32)
  sprintf(buffer, "%f", eic.GetLineCurrent());
  client.publish("CURRENT",buffer );
  #else
  sprintf(buffer, "%f", eic.GetLineCurrent());
  client.publish("CURRENT",buffer );
  #endif
  delay(10);
  sprintf(buffer, "%f", eic.GetActivePower());
  client.publish("AP",buffer );
  delay(10);
  sprintf(buffer, "%f", eic.GetPowerFactor());
  client.publish("PF",buffer );
  delay(1000);
}
