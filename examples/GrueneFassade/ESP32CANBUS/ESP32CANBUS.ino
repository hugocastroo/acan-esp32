/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           ESP32CANBUS Master script
%   Description:    Script for a ESP32 Master in a CANBUS line. The CANBUS
%					line is equiped with many Rasbperry Pi Pico modules
%					that send temperatur and soil moisture measurements
%					using the CANBUS protocol. After sampling the information
%					in the CANBUS line, the script uploads the measurements
%					using HTTP protocol.
%   Date:           06/12/2023      
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
% V12:    millis() overflow problem was solved with a second variable and using ULONG variables.
% V11:    CANBUS data scaling implementation for more precise data.
%	V01:			Program uploads the measurements samplings to a test
%					BACKEND.
%					Measurments are readed from different Raspberry Pi Pico
%					boards and sorted out.
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//  Board Check
#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif
//   Include libraries
#include <ACAN_ESP32.h>
#include <core_version.h> // For ARDUINO_ESP32_RELEASE
#include "time.h"         // Library for the EPOCH TIME
#include <TimeLib.h>      // Library for time functions
#include <WiFi.h>         //Library for the WiFi Modul
#include <HTTPClient.h>   //Library for the http functions
#include <Arduino_JSON.h> //Library for working with JSON syntax in Arduino
#include <Preferences.h>  //Library for storing and reading values from the EEPROM
//   SETUP and Variables
#define LED_BUILTIN 2 //Change LED Position if using different board
#define PIN_POWERON 32 //Change PIN Position if using other PIN
#define InfoPrint 0 //Change flag to show chip and BUS settings in the serial monitor 0-1
#define ErrorInfoPrint 1 //Change flag to show ErrorStatics of the CANBUS in the serial monitor 0-1
#define RemoteFrameID 99 //ID for the remote frames to ask for information from the slaves
#define SlavesTurnOnDelay 5000 // (ms) Wait for the slaves to warm up and be able to send information
#define ArrayLimit 99 //Limit for the array storage of the queued messages, THIS IS LIMITED BY THE RAM MEMORY, so it cannot be too big, still need to check what are the limits.
#define Samplings 100
static const uint32_t DESIRED_BIT_RATE = 1000UL * 125UL ; // 125 Kb/s ESP32 Desired Bit Rate
//Counter for debugging and statistics of the BUS
uint32_t ReceivedFrameCount = 0;
uint32_t SentFrameCount = 0;
unsigned long referenzMillis = 0;
unsigned long TimeInterval = 1000*1*1;
uint32_t samplingCounter = 0;
uint32_t currentMessagesQueued = 0;
//Array for Queuing the received messages
CANMessage queuedMessages[ArrayLimit];
String timeStamps[ArrayLimit];
//static const char* const timeStamps[ArrayLimit]; //Check if it works, this way and if not then find a way to do not use STRING
// Replace with network credentials
//Zuhause
//const char* ssid = "nordisch-Box-1";
//const char* password = "87654321";
//Büro
//const char* ssid = "castro";
//const char* password = "msnc5000";
//Labor
const char* ssid = "Mexicano";
const char* password = "Mexicano";
// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";
//Your Domain name with URL path or IP address with path for HTTP Posting
const char* serverName = "https://europe-west1-gruenfacade.cloudfunctions.net/app/api/facade/test/sensordata";
//JSONVar jsonElements;
//Variables and instances for the EEPROM rewrite/read procedure
Preferences preferences;
const char* spaceEEPROM = "timestamp";
const char* paramEEPROM = "stamp";

//----------------------------------------------------------------------------------------
void setup() {
  //Switch on builtin led to know that the program started
  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite (LED_BUILTIN, HIGH);
  //Configure the OUTPUT PIN for turning on and off the slaves. Set it OFF
  pinMode (PIN_POWERON, OUTPUT);
  digitalWrite (PIN_POWERON, LOW);
  Serial.begin (115200); // Start serial 
  initWiFi(); // Init WiFi
  getEEPROMTimestamp(); //Get the last timestamp stored in the EEPROM in case that the NTP server cannot establish connecion.
  configTime(0, 0, ntpServer); //Start NTP server connection for the UNIX TIME
  delay (1000);
  //Configure ESP32 CAN
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE);
  settings.mRxPin = GPIO_NUM_27; // Optional, default Rx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_14; // Optional, default Tx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings);
  //Display ESP32 Chip and BUS Settings Info if the InfoPrint flag is active. Possible section for further debugging
  if(InfoPrint){
    esp_chip_info_t chip_info;
    chipInfo(chip_info);
    busInfo(errorCode,settings);
    }
  }
//----------------------------------------------------------------------------------------
void loop () {
  //CANMessages initialization
  CANMessage remoteFrame; //Message for requesting data from the slaves
  remoteFrame.ext = false;
  remoteFrame.id = RemoteFrameID;
  remoteFrame.rtr = true;
  CANMessage frame; //No initialization needed, since the message will be read from the slaves
  int arrayIndex = 0; //Variable for the elements position in the arrays
  if (millis() - referenzMillis > TimeInterval) { // Overflow solution for the time problem.
    referenzMillis = millis();
    digitalWrite (PIN_POWERON, HIGH);//Turn Power ON for the slaves in order to come online
    delay(SlavesTurnOnDelay);
    //Sent remote frame to the BUS in order to get information from the Slaves
    const bool okRemoteFrame = ACAN_ESP32::can.tryToSend (remoteFrame);
    if (okRemoteFrame){
      SentFrameCount += 1;
      samplingCounter = 0;
      arrayIndex = 0;
      currentMessagesQueued = 0;
      //Serial.print("Remote message sent succ. \n"); //Check if the message was sent succ.
    }
    displayBUSstatistics(); //If flag ErrorInfoPrint is active, then statics information will be displayed in the serial monitor
    //Sample for a certain time for the slave's answers
    while(samplingCounter <= Samplings){
      if (ACAN_ESP32::can.receive (frame)){
        queuedMessages[arrayIndex] = frame; //Add the received frames to a Queue in order to process them later
        timeStamps[arrayIndex] = getTime();
        ReceivedFrameCount += 1;
        arrayIndex += 1;
        currentMessagesQueued = arrayIndex;
        //framePrinting(frame); //Print the data of the received frames if desired
      }
      samplingCounter += 1;
      delay(100); //Check how small this can get
    }
    digitalWrite (PIN_POWERON, LOW);//Turn Power off for the slaves in order to spare energy
    checkWiFi(); //Check for the WiFi connection before posting the messages
    processQueuedMessages(queuedMessages); //Once the messages has been queued, they will be send
  }
  delay(1000);
  }
//----------------------------------------------------------------------------------------
//Help/Debug functions
  //Display ESP32 Chip Info
  void chipInfo(esp_chip_info_t chip_info){
    esp_chip_info (&chip_info);
    Serial.print ("ESP32 Arduino Release: ");
    Serial.println (ARDUINO_ESP32_RELEASE);
    Serial.print ("ESP32 Chip Revision: ");
    Serial.println (chip_info.revision);
    Serial.print ("ESP32 SDK: ");
    Serial.println (ESP.getSdkVersion ());
    Serial.print ("ESP32 Flash: ");
    Serial.print (spi_flash_get_chip_size () / (1024 * 1024));
    Serial.print (" MB ");
    Serial.println (((chip_info.features & CHIP_FEATURE_EMB_FLASH) != 0) ? "(embeded)" : "(external)");
    Serial.print ("APB CLOCK: ");
    Serial.print (APB_CLK_FREQ);
    Serial.println (" Hz");
    }
  //Display BUS settings and or error codes
  void busInfo(uint32_t errorCode,ACAN_ESP32_Settings settings){
    if (errorCode == 0) {
      Serial.print ("Bit Rate prescaler: ");
      Serial.println (settings.mBitRatePrescaler);
      Serial.print ("Time Segment 1:     ");
      Serial.println (settings.mTimeSegment1);
      Serial.print ("Time Segment 2:     ");
      Serial.println (settings.mTimeSegment2);
      Serial.print ("RJW:                ");
      Serial.println (settings.mRJW);
      Serial.print ("Triple Sampling:    ");
      Serial.println (settings.mTripleSampling ? "yes" : "no");
      Serial.print ("Actual bit rate:    ");
      Serial.print (settings.actualBitRate ());
      Serial.println (" bit/s");
      Serial.print ("Exact bit rate ?    ");
      Serial.println (settings.exactBitRate () ? "yes" : "no");
      Serial.print ("Distance            ");
      Serial.print (settings.ppmFromDesiredBitRate ());
      Serial.println (" ppm");
      Serial.print ("Sample point:       ");
      Serial.print (settings.samplePointFromBitStart ());
      Serial.println ("%");
      Serial.println ("Configuration OK!");
    } 
    //If there is an error code, then it should be displayed, In case an error persist, then the LED should blink
    else {
      Serial.print ("Configuration error 0x");
      Serial.println (errorCode, HEX);
      digitalWrite (LED_BUILTIN, LOW);
      delay(250);
      digitalWrite (LED_BUILTIN, HIGH);
    }
    }
  //Statics information of the BUS will be displayed in the serial monitor
  void displayBUSstatistics(){
    if (ErrorInfoPrint){
      Serial.print ("Sent: ") ;
      Serial.print (SentFrameCount) ;
      Serial.print (" ") ;
      Serial.print ("Receive: ") ;
      Serial.print (ReceivedFrameCount) ;
      Serial.print (" ") ;
      Serial.print (" STATUS 0x") ;
      Serial.print (TWAI_STATUS_REG, HEX) ;
      Serial.print (" RXERR ") ;
      Serial.print (TWAI_RX_ERR_CNT_REG) ;
      Serial.print (" TXERR ") ;
      Serial.println (TWAI_TX_ERR_CNT_REG) ;
    }
    }
  //Print the frame data in the serial monitor
  void framePrinting(CANMessage frame){
    Serial.print ("**** Received ");
    Serial.print (frame.ext ? "extended " : "standard ");
    Serial.print (frame.rtr ? "remote " : "data ");
    Serial.print ("frame, id 0x");
    Serial.print (frame.id, HEX);
    Serial.print (", Data: ");
    for(int i = 0; i < frame.len;i++){
      Serial.print(frame.data[i]);
      Serial.print(", ");
    }
    Serial.print("\n");
    }
  //Process the queued messages
  void processQueuedMessages(CANMessage queuedMessages[]){
    if(WiFi.status()== WL_CONNECTED){
      //Initialize the JSON object from the queuedMessages Array
      for (int i = 0; i<currentMessagesQueued;i++){
        JSONVar myObject;
        char* sensorType = "";
        float dataCANBUS = -99.99;
        //According to the id of the CANMessage, sort the message to the corresponding sensor
        if(queuedMessages[i].id >= 0xff00){
          sensorType = "tempreture";
          dataCANBUS = scaleCANBUStemperature(queuedMessages[i].data32[0]);
        }
        else if(queuedMessages[i].id <= 0xee00){
          sensorType = "humidity";
          dataCANBUS = scaleCANBUShumidity(queuedMessages[i].data32[0]);
        }
        //Set every attribute of the JSONVar
        myObject["id"] = String(queuedMessages[i].id);
        myObject["type"] = sensorType;
        myObject["time"] = timeStamps[i];
        myObject["value"] = dataCANBUS; //Implement the sensors in the boards and then use the data field of the currentMessagesQueued
        myObject["row"] = (long) queuedMessages[i].id;//Ask jeschke how we are gonna do this positioning
        myObject["column"] = (long) queuedMessages[i].id;
        //POST the JSON object
        int response = httpPOSTRequest(serverName,myObject);
        //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
        //Serial.println(response); //Print the response code if desired
      }
    }
    else {
    checkWiFi(); //Check for the WiFi connection before posting the messages
    //Serial.println("WiFi Disconnected");
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
  int httpPOSTRequest(const char* serverName, JSONVar myObject){
    //Get the JSON elements descriptions in case it is necessary to create a new object.
    //jsonElements = httpGETRequest(serverName);
    // Your Domain name with URL path or IP address with path
    HTTPClient http;
    http.begin(serverName);
    int httpResponseCode = 0;
    // If you need Node-RED/server authentication, insert user and password below
    //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
    int responseGET = httpGETRequest(serverName);
    if (responseGET>0) {
      //Serial.print("HTTP Response code: "); //Display response code if desired
      //Serial.println(httpResponseCode);
      // If you need an HTTP request with a content type: application/json, use the following:
      http.addHeader("Content-Type", "application/json");
      //httpResponseCode = http.POST("{ \"sensordata\":[ { \"id\": \"45\", \"type\": \"tempreture\", \"time\": \"2023-11-02T11:50:50+00:00\", \"value\": 99, \"row\": 1, \"column\": 2 }, { \"id\": \"45\", \"type\": \"tempreture\", \"time\": \"2023-11-02T11:50:50+00:00\", \"value\": 99, \"row\": 1, \"column\": 2 } ] }");
      String json = "{ \"sensordata\":[" + JSON.stringify(myObject) + "]}";
      //Serial.println(json);
      httpResponseCode = http.POST(json);
      //"{ \"sensordata\":[]}"
    }
    else {
      //Serial.print("Error code: ");
      //Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
    return httpResponseCode;
    }
  // Function that gets current epoch time
  String getTime() {
    struct tm timeinfo;
    char timestamp[26];
    //This would not give the exact date, but would keep the data chronologically at least.
    //The last time from NTP or working time from the ESP32 will be stored in the EEPROM
    //getEEPROMTimestamp(); //We do not use it here, because if so, then it will set the time everytime according to the value in the the EEPROM, that is why it should be done just once the program starts and not later.
    //Get the current time in the device since the NTP connection failed. after getting the current time in the device, store the current time in the EEPROM for future processing in case the device goes off
    if (!getLocalTime(&timeinfo)) {
      //Get the time stamp in the ISO8601 format
      //YYYY-MM-DDTHH:MM:SS+00:00
      sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", year(), month(), day(),hour(), minute(), second());
      overwriteEEPROMTimestamp(now());
      return(timestamp);
      }
    else{
      //Get the time stamp in the ISO8601 format
      //YYYY-MM-DDTHH:MM:SS+00:00
      sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday,timeinfo.tm_hour+1, timeinfo.tm_min, timeinfo.tm_sec);
      //Overwrite the EEPROMtimeStamp with the current value from the NTP server to refresh it with new data
      setTime(timeinfo.tm_hour+1, timeinfo.tm_min, timeinfo.tm_sec,timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900);
      overwriteEEPROMTimestamp(now());
      return timestamp;
      }
    }
  // Initialize WiFi
  void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    delay(2000);
    checkWiFi(); //Check for the WiFi connection before posting the messages
    //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
    }
  // Check WiFi connection
  void checkWiFi() {
    //Try to connect if the WL_CONNECTED is false
    if(WiFi.status() != WL_CONNECTED){
      //Serial.println("Not connected, Trying to connect");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      delay(2000);
    }
    else{
      //WiFi was already connected
    }
    //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
    }

  //Read EEPROM-Timestamp value
  void getEEPROMTimestamp() {
    //init preference
    preferences.begin(spaceEEPROM, false);
    //preferences.clear(); // remove all preferences in namespace myfile
    //preferences.remove("timeStampID");// remove varname in the namespace
    long readstamp = preferences.getULong(paramEEPROM, 0); //
    preferences.end();
    setTime(readstamp);
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
  //Scaling for the CANBUS temperature Data
  float scaleCANBUStemperature(uint32_t data32){
    //Variables for the CANBUS Data scaling
    uint32_t maxScaling = (2<<15)-1;
    int minScaling = 0;
    int tempA = -100;
    int tempB = 100;
    float result = ((((tempB-tempA)*(float(data32)-minScaling))/(maxScaling-minScaling))+tempA);
    return result;
    }
  //Scaling for the CANBUS humidity Data
  float scaleCANBUShumidity(uint32_t data32){
    //Variables for the CANBUS Data scaling
    uint32_t maxScaling = (2<<15)-1;
    int minScaling = 0;
    int humidA = 0;
    int humidB = 100;
    float result = ((((humidB-humidA)*(float(data32)-minScaling))/(maxScaling-minScaling))+humidA);
    return result;
    }