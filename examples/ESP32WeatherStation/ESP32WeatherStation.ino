/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           ESP32CANBUS Master script
%   Description:    Script for a ESP32 Weather station program base for 
%                   gathering the information of the weather station and 
%                   uploading the information with the ESP32 directly to the cloud, 
%                   without the master using HTTP protocol.
%   Date:           15/10/2024
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
% V02:      Implemented offset for the temperature to make the correct reading when the temperature is engative. This was needed, since if not, the -1 value of temperature, was 65535.
%	V01:			Weather station program base for gathering the information
%           of the weather station and uploading the information with the 
%           ESP32 directly to the cloud, without the master.
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//  Board Check
#ifndef ARDUINO_ARCH_ESP32
#error "Select an ESP32 board"
#endif
//   Include libraries
#include <core_version.h>  // For ARDUINO_ESP32_RELEASE
#include "time.h"          // Library for the EPOCH TIME
#include <TimeLib.h>       // Library for time functions
#include <WiFi.h>          //Library for the WiFi Modul
#include <HTTPClient.h>    //Library for the http functions
#include <Arduino_JSON.h>  //Library for working with JSON syntax in Arduino
#include <Preferences.h>   //Library for storing and reading values from the EEPROM
#include <ModbusMaster.h>  //Library for the MODBUS 485 IC in order to communicate with the Weather station
//Setup-Global Variables
//Pins
//#define PIN_IOEXPANDER 16  //GPIO PIN für die RESET PIN_IOEXPANDER, es muss HIGH sein, sonst startet der IC sich die ganze Zeit neu und kommuniziert nicht.
#define MAX485_DE_RE 16    //Pin for toggling the MAX485 Board //PIN_IOEXPANDER muss gelöscht werden, bei der nächsten Platine.
#define MAX485_RX 18       //Pin for the RX communication RO RS485MAX
#define MAX485_TX 19       //Pin for the TX communication DI RS485MAX

//Flags
bool weatherStationFlag = false;   //Flag for knowing if data from the weather station was received or not
bool InfoPrint = false;
//Internet access // Replace with network credentials
const char* ssid = "ShellyPro1-EC626091346C";
const char* password = "Demonstrator";
//const char* ssid = "Mexicano";
//const char* password = "Mexicano";


//Weather station variables
float weatherStationHumidity = 0;
float weatherStationTemperature = 0;
int weatherStationLux = 0;
float tempOffset = 65536;
ModbusMaster node;  //object node for class ModbusMaster
// NTP SERVER - API variables
const char* ntpServer = "pool.ntp.org";                                                                         //NTP server for getting the time online
const char* serverName = "https://gruenfacade.web.app/api/facade";  //Your Domain name with URL path or IP address with path for HTTP Posting
int offsetUTC = 0;          //Offset für die Zeitzonen Uhrzeit ist in UTC, dann soll die Zeitzone hier inkrementiert oder dekrementiert werden
const char* facadeID = "demonstrator";  //Id for the facade,  this plays a role for the backend
Preferences preferences;                //Variables and instances for the EEPROM rewrite/read procedure
const char* spaceEEPROM = "timestamp";  //variable for the EEPROM procedure
const char* paramEEPROM = "stamp";      //variable for the EEPROM procedure

String timeStamp = "";

void setup() {
  Serial.begin(115200);                // Start serial in case it is desired to debug or display any information
  initWiFi();                          // Init WiFi
  getEEPROMTimestamp();                //Get the last timestamp stored in the EEPROM in case that the NTP server cannot establish connecion.
  configTime(0, 0, ntpServer);         //Start NTP server connection for the UNIX TIME
  delay(100);
  //Display ESP32 Chip and BUS Settings Info if the InfoPrint flag is active. Possible section for further debugging
  if (InfoPrint) {
    esp_chip_info_t chip_info;
    chipInfo(chip_info);
  }
  weatherStationSetUp(); //Configure RS485 Modbus channel for the weather station
  Serial.println("Setup done");  //Info
}
void loop() {
  delay(1000*5);
  //Main Program
  weatherStationFlag = weatherStationGetData();
  timeStamp = getTime();
  if (weatherStationFlag) {  
    Serial.print("Humidity: ");
    Serial.print(weatherStationHumidity);
    Serial.println("%");
    Serial.print("Temp: ");
    Serial.print(weatherStationTemperature);
    Serial.println("°C");
    Serial.print("Illuminance: ");
    Serial.print(weatherStationLux);
    Serial.println("Lux");
    processDataWeatherstation();
    Serial.println("All information posted");
    delay(1000*60*5);
  }
  else{

  }

}
//Display ESP32 Chip Info
void chipInfo(esp_chip_info_t chip_info) {
  esp_chip_info(&chip_info);
  Serial.print("ESP32 Arduino Release: ");
  Serial.println(ARDUINO_ESP32_RELEASE);
  Serial.print("ESP32 Chip Revision: ");
  Serial.println(chip_info.revision);
  Serial.print("ESP32 SDK: ");
  Serial.println(ESP.getSdkVersion());
  Serial.print("ESP32 Flash: ");
  Serial.print(spi_flash_get_chip_size() / (1024 * 1024));
  Serial.print(" MB ");
  Serial.println(((chip_info.features & CHIP_FEATURE_EMB_FLASH) != 0) ? "(embeded)" : "(external)");
  Serial.print("APB CLOCK: ");
  Serial.print(APB_CLK_FREQ);
  Serial.println(" Hz");
}
//Process the queued messages
void processDataWeatherstation() {
  if (WiFi.status() == WL_CONNECTED) {
    //Initialize the JSON object for the humidity package
    //Post the sensor's information and the position in the first FRONTEND
    JSONVar myObject;
    char* sensorType = "weatherStationHumidity";
    float dataWS = weatherStationHumidity;
    //Set every attribute of the JSONVar
    myObject["id"] = "4357"; //ID equal to 0x1105
    myObject["type"] = sensorType;
    myObject["x"] = 3;                               //Set the row value to the object using the CANBUSlines parameter given with row, + 1 since it starts in 0
    myObject["y"] = 1;
    myObject["z"] = 0;
    int response = httpPOSTRequest(serverName, myObject,0);  //POST the JSON object
    if(response == 200){
      //Serial.println("posted");
    }
    else{
        int counter = 0;
        while(response != 200){
          //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
          response = httpPOSTRequest(serverName, myObject,0);  //POST the JSON object
          counter++;
          if(counter <= 4 && response == 200){
            Serial.println(" reposting succesfull."); //Print the response code if desired
          }
          else if(counter >= 5){
            Serial.println(" reposting timeout."); //Print the response code if desired
            response = 200;
          }
        }
    }
    //Post the timestamp and value in the second FRONTEND
    String serverName1 = String(serverName) + "/" + String(facadeID) + "/sensor/4357/measurement"; //ID equal to 0x1105
    JSONVar myObject1;
    myObject1["time"] = timeStamp;
    myObject1["value"] = (weatherStationHumidity,0);
    response = httpPOSTRequest(serverName1.c_str(), myObject1,1);  //POST the JSON object
    if(response == 200){
      //Serial.println("posted in the second backend");
      //Serial.println(response);
    }
    else{
        int counter = 0;
        while(response != 200){
          //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
          response = httpPOSTRequest(serverName1.c_str(), myObject1,1);  //POST the JSON object
          counter++;
          if(counter <= 4 && response == 200){
            Serial.println(" reposting succesfull."); //Print the response code if desired
          }
          else if(counter >= 5){
            Serial.println(" reposting timeout."); //Print the response code if desired
            response = 200;
          }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Initialize the JSON object for the humidity package
    //Post the sensor's information and the position in the first FRONTEND
    JSONVar myObject2;
    sensorType = "weatherStationTemperature";
    //Set every attribute of the JSONVar
    myObject2["id"] = "4358"; //ID equal to 0x1106
    myObject2["type"] = sensorType;
    myObject2["x"] = 3;                               //Set the row value to the object using the CANBUSlines parameter given with row, + 1 since it starts in 0
    myObject2["y"] = 1;
    myObject2["z"] = 0;
    response = httpPOSTRequest(serverName, myObject2,0);  //POST the JSON object
    if(response == 200){
      //Serial.println("posted");
    }
    else{
        int counter = 0;
        while(response != 200){
          //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
          response = httpPOSTRequest(serverName, myObject2,0);  //POST the JSON object
          counter++;
          if(counter <= 4 && response == 200){
            Serial.println(" reposting succesfull."); //Print the response code if desired
          }
          else if(counter >= 5){
            Serial.println(" reposting timeout."); //Print the response code if desired
            response = 200;
          }
        }
    }
    //Post the timestamp and value in the second FRONTEND
    serverName1 = String(serverName) + "/" + String(facadeID) + "/sensor/4358/measurement";//ID equal to 0x1106
    JSONVar myObject3;
    myObject3["time"] = timeStamp;
    myObject3["value"] = (weatherStationTemperature,1);
    response = httpPOSTRequest(serverName1.c_str(), myObject3,1);  //POST the JSON object
    if(response == 200){
      //Serial.println("posted in the second backend");
      //Serial.println(response);
    }
    else{
        int counter = 0;
        while(response != 200){
          //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
          response = httpPOSTRequest(serverName1.c_str(), myObject3,1);  //POST the JSON object
          counter++;
          if(counter <= 4 && response == 200){
            Serial.println(" reposting succesfull."); //Print the response code if desired
          }
          else if(counter >= 5){
            Serial.println(" reposting timeout."); //Print the response code if desired
            response = 200;
          }
        }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Initialize the JSON object for the humidity package
    //Post the sensor's information and the position in the first FRONTEND
    JSONVar myObject4;
    sensorType = "weatherStationLux";
    //Set every attribute of the JSONVar
    myObject4["id"] = "4359"; //ID equal to 0x1107
    myObject4["type"] = sensorType;
    myObject4["x"] = 3;                               //Set the row value to the object using the CANBUSlines parameter given with row, + 1 since it starts in 0
    myObject4["y"] = 1;
    myObject4["z"] = 0;
    response = httpPOSTRequest(serverName, myObject4,0);  //POST the JSON object
    if(response == 200){
      //Serial.println("posted");
    }
    else{
        int counter = 0;
        while(response != 200){
          //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
          response = httpPOSTRequest(serverName, myObject4,0);  //POST the JSON object
          counter++;
          if(counter <= 4 && response == 200){
            Serial.println(" reposting succesfull."); //Print the response code if desired
          }
          else if(counter >= 5){
            Serial.println(" reposting timeout."); //Print the response code if desired
            response = 200;
          }
        }
    }
    //Post the timestamp and value in the second FRONTEND
    serverName1 = String(serverName) + "/" + String(facadeID) + "/sensor/4359/measurement"; //ID equal to 0x1107
    JSONVar myObject5;
    myObject5["time"] = timeStamp;
    myObject5["value"] = weatherStationLux;
    response = httpPOSTRequest(serverName1.c_str(), myObject5,1);  //POST the JSON object
    if(response == 200){
      //Serial.println("posted in the second backend");
      //Serial.println(response);
    }
    else{
        int counter = 0;
        while(response != 200){
          //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
          response = httpPOSTRequest(serverName1.c_str(), myObject5,1);  //POST the JSON object
          counter++;
          if(counter <= 4 && response == 200){
            Serial.println(" reposting succesfull."); //Print the response code if desired
          }
          else if(counter >= 5){
            Serial.println(" reposting timeout."); //Print the response code if desired
            response = 200;
          }
        }
    }
  }
  else {
    checkWiFi();  //Check for the WiFi connection before posting the messages
    Serial.println("WiFi Disconnected");
  }
}
//Send a GETRequest to the server to check for availability
int httpGETRequest(const char* serverName) {
  //Start the http connection
  HTTPClient http;
  http.begin(serverName);
  // Send HTTP GET request
  int httpResponseCode = http.GET();
  // String payload = "{}"; //Variable to store the GET package
  // JSONVar keys = "Failed";
  // //Negative response codes are an error. 200 means OK
  // if (httpResponseCode>0) {
  //   //Serial.print("HTTP Response code: "); //Display response code if desired
  //   //Serial.println(httpResponseCode);
  //   payload = http.getString();
  //   //Parse payload to JSON object in order to be able to use the data
  //   JSONVar myObject = JSON.parse(payload);
  //   // JSON.typeof(jsonVar) can be used to get the type of the var
  //   if (JSON.typeof(myObject) == "undefined") {
  //     Serial.println("Parsing input failed!");
  //     keys = "Failed";
  //   }
  //   else{
  //     // myObject.keys() can be used to get an array of all the keys in the object
  //     keys = myObject[0].keys();
  //     //Loop for printing the JSON elements if desired
  //     // for (int j = 0; j < myObject.length();j++){
  //     //   for (int i = 0; i < keys.length(); i++){
  //     //     Serial.print(keys[i]);
  //     //     Serial.print(": ");
  //     //     Serial.print(myObject[j][keys[i]]);
  //     //     Serial.print(" / ");
  //     //   }
  //     //   Serial.print("\n");
  //     // }
  //   }
  // }
  // else {
  //   //Serial.print("Error code: ");
  //   //Serial.println(httpResponseCode);
  //   keys = "Failed";
  // }
  // Free resources
  http.end();
  return httpResponseCode;
}
//Post a message in the server
int httpPOSTRequest(const char* serverName, JSONVar myObject,int backend) {
  //Get the JSON elements descriptions in case it is necessary to create a new object.
  //jsonElements = httpGETRequest(serverName);
  // Your Domain name with URL path or IP address with path
  HTTPClient http;
  http.begin(serverName);
  int httpResponseCode = 0;
  // If you need Node-RED/server authentication, insert user and password below
  //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
  int responseGET = httpGETRequest(serverName);
  if (responseGET > 0) {
    //Serial.print("HTTP Response code: "); //Display response code if desired
    //Serial.println(httpResponseCode);
    // If you need an HTTP request with a content type: application/json, use the following:
    http.addHeader("Content-Type", "application/json");
    //httpResponseCode = http.POST("{ \"sensordata\":[ { \"id\": \"45\", \"type\": \"tempreture\", \"time\": \"2023-11-02T11:50:50+00:00\", \"value\": 99, \"row\": 1, \"column\": 2 }, { \"id\": \"45\", \"type\": \"tempreture\", \"time\": \"2023-11-02T11:50:50+00:00\", \"value\": 99, \"row\": 1, \"column\": 2 } ] }");
    //String json = "{ \"sensordata\":[" + JSON.stringify(myObject) + "]}"; Altes String für den alten Backend
    String json = "";
    if(backend == 1){
      json = JSON.stringify(myObject);
    }
    else{
      json = "{\"id\":\"" + String(facadeID) + "\", \"sensors\":[" + JSON.stringify(myObject) + "]}"; //Falls dimensions noch hinzugefügt wird, dann muss ich hier wahrscheinlich das am Ende noch hinzufügen
    }
    //Serial.println(json);
    httpResponseCode = http.POST(json);
    //"{ \"sensordata\":[]}"
  } else {
    Serial.print(", Error code: ");
    Serial.print(httpResponseCode);
  }
  // Free resources
  http.end();
  return httpResponseCode;
}
// Function that gets current epoch time
String getTime() {
  struct tm timeinfo;
  char timestamp[26];
  //Get the current time in the device since the NTP connection failed. after getting the current time in the device, store the current time in the EEPROM for future processing in case the device goes off
  if (!getLocalTime(&timeinfo)) {
    getEEPROMTimestamp();                                                                                           //Set the time before reading it, according to the clock IC if available, if not then from the EEPROM
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", year(), month(), day(), hour(), minute(), second());  //Get the time stamp in the ISO8601 format YYYY-MM-DDTHH:MM:SS+00:00
    return (timestamp);
  } else {                                                                                                                                                                              //If it was possible to get the information from the NTP server, then use this information and store it in the EEPROM and sync the IC Clock Get the time stamp in the ISO8601 format YYYY-MM-DDTHH:MM:SS+00:00
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour + offsetUTC, timeinfo.tm_min, timeinfo.tm_sec);  //Store the NTP data in the timestamp variable
    setTime(timeinfo.tm_hour + offsetUTC, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);                                                    //Overwrite the internal clock of the ESP32 with the current value from the NTP server to refresh it with new data
    overwriteEEPROMTimestamp(now());  //Overwrite the EEPROMtimeStamp with the current value from the NTP server to refresh it with new data
    return timestamp;
  }
}
// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  delay(2000);
  checkWiFi();  //Check for the WiFi connection before posting the messages
  //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
}
// Check WiFi connection
void checkWiFi() {
  //Try to connect if the WL_CONNECTED is false
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, Trying to connect");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    delay(2000);
  } else {
    //WiFi was already connected
    Serial.println("WiFi connected");
  }
  //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
}
//Read EEPROM-Timestamp value
void getEEPROMTimestamp() {
  if (true) {
  } else {
    //init preference
    preferences.begin(spaceEEPROM, false);
    //preferences.clear(); // remove all preferences in namespace myfile
    //preferences.remove("timeStampID");// remove varname in the namespace
    long readstamp = preferences.getULong(paramEEPROM, 0);  // Rewrite the readtstamp value with the EEPROM value
    preferences.end();
    setTime(readstamp);
  }
}
//Overwrite the EEPROM-Timestamp value
void overwriteEEPROMTimestamp(long stamp) {
  //init preference
  preferences.begin(spaceEEPROM, false);
  //preferences.clear(); // remove all preferences in namespace myfile
  //preferences.remove("varname");// remove varname in the namespace
  preferences.putULong(paramEEPROM, stamp);
  preferences.end();
}
//Method for the set up of the weather station configuration
void weatherStationSetUp() {
  pinMode(MAX485_DE_RE, OUTPUT);                          //Set the MAX485_DE_RE as output
  digitalWrite(MAX485_DE_RE, LOW);                        //Set the MAX485_DE_RE low
  Serial1.begin(4800, SERIAL_8N1, MAX485_RX, MAX485_TX);  // Transmission mode: MODBUS-RTU, Baud rate: 4800bps, Data bits: 8, Stop bit: 1, Check bit: no
  node.begin(1, Serial1);                                 // The weather station has a default communication baudrate of 4800 bps and 0x01 address.
  node.preTransmission(preTransmission);                  //Set the transmision values to default
  node.postTransmission(postTransmission);
}
//Method for starting the transmition when getting information from the weather station
void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH);
}
//Method for finishing the transmition when getting information from the weather station
void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW);
}
//Method for reading the values from the sensors in the weather station and storing them in the global values.
bool weatherStationGetData() {
  // The 03H Register Code Example: Read The Atmospheric Humidity, Temperature & Illuminance
  // Host Scan Order (Slave addr:0x01): 0x01 0x03 0x00 0x00 0x00 0x07 0x04 0x08 (Check the sensor documentation for further commands.)
  // Slave Response: 0x01 0x03 0x0E 0x01 0xCB 0x00 0xC0 0x00 0x74 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x74 0x5F 0x78
  // Address of the first register Lo bytes is 00
  // Number of registers of Lo bytes is 03
  uint8_t result = node.readHoldingRegisters(0, 3);
  delay(100);
  //Printing the information in the console/procesing of the information
  if (result == node.ku8MBSuccess) {  //If the Message was received, then store the values in the corresponding variables
    weatherStationHumidity = node.getResponseBuffer(0) / 10.0;
    weatherStationTemperature = node.getResponseBuffer(1) / 10.0;
    weatherStationLux = node.getResponseBuffer(2);
    if(weatherStationTemperature > 2000)
    {
      Serial.println(weatherStationTemperature);
      weatherStationTemperature = (node.getResponseBuffer(1) - tempOffset)/10.0;
      Serial.println("value was negative");
      Serial.println(weatherStationTemperature);
    }
    return true;
  } else {  //If no data received, set the weather station flag to error.
    weatherStationHumidity = 0;
    weatherStationTemperature = 0;
    weatherStationLux = 0;
    return false;
  }
}