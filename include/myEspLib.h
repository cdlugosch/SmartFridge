#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"

/* defined in config.h */
/*
// WiFi 
const char* ssid            = "ssid";
const char* WiFi_password   = "pass";

// MQTT 
const char* mqttServer      = "1.2.3.4";
const int mqttPort          = 1883;
const char* mqtt_user       = "";
const char* mqtt_password   = "";
const char* topic_root      = "project/";
const char* mqtt_ClientName = "ESP32Client_4_Project";
*/ 

long int start_seconds = 0;

int max_connect_attempts = 3;
int connect_attempt_count = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

/* ############################ */
boolean connectNetwork(){
  

  /* try a few times to connecto to WiFi */
  while((!mqttClient.connected()) && (connect_attempt_count < max_connect_attempts)){
      
    start_seconds = millis();
    // attempt to connect to Wifi network:
    Serial.print("Attempting to connect to Wifi network, SSID: ");
    Serial.println(ssid);
    WiFi.mode(WIFI_STA);
  
    if(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, WiFi_password);
      Serial.println("Try to Connect to WiFI ");
    }  

    while ((WiFi.status() != WL_CONNECTED) && (start_seconds+5000 > millis())) {
      //Serial.print(".");
    }  
    /* DEBUG MESSAGES */  
    switch(WiFi.status()) {
      case WL_IDLE_STATUS: Serial.println(" WL_IDLE_STATUS"); break;
      case WL_NO_SSID_AVAIL: Serial.println(" WL_NO_SSID_AVAIL"); break;
      case WL_SCAN_COMPLETED: Serial.println(" WL_SCAN_COMPLETED"); break;
      case WL_CONNECTED: 
        Serial.println(" WL_CONNECTED - Connected to the WiFi network");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());      
        break;
      case WL_CONNECT_FAILED: Serial.println(" WL_CONNECT_FAILED"); break;
      case WL_CONNECTION_LOST: Serial.println(" WL_CONNECTION_LOST"); break;
      case WL_DISCONNECTED: Serial.println(" WL_DISCONNECTED"); break;
      case WL_NO_SHIELD: Serial.println(" WL_NO_SHIELD"); break;

      default: Serial.println(" UNKNOWN WL STATUS:" + WiFi.status()); break;
    }  

    if(WiFi.status() == WL_CONNECTED){
      Serial.println("Connecting to MQTT...");    
      mqttClient.setServer(mqttServer, mqttPort);
      mqttClient.connect(mqtt_ClientName, NULL, NULL );
      
      start_seconds = millis();
      while ((!mqttClient.connected()) && (start_seconds+5000 > millis())) {
          //Serial.print(".");
      }

      if (mqttClient.connect(mqtt_ClientName, NULL, NULL )) {
        Serial.println("MQTT connected");  
        return true;
      } else {
        Serial.print("MQTT failed with state ");
        Serial.println(mqttClient.state());
        return false;
      }
    }else{
      return false;
    }

    connect_attempt_count +=1;
  }

  return false;
}