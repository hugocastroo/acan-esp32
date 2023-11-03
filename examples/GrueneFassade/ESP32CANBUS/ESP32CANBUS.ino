//----------------------------------------------------------------------------------------
//  Board Check
//----------------------------------------------------------------------------------------
#ifndef ARDUINO_ARCH_ESP32
  #error "Select an ESP32 board"
#endif

//----------------------------------------------------------------------------------------
//   Include files
//----------------------------------------------------------------------------------------
#include <ACAN_ESP32.h>
#include <core_version.h> // For ARDUINO_ESP32_RELEASE
#include "time.h"         // Library for the EPOCH TIME
#include <TimeLib.h>      //Library for processing the timestamp
#include <WiFi.h>         //Library for the WiFi Modul
#include <HTTPClient.h>   //Library for the http functions
#include <Arduino_JSON.h> //Library for working with JSON syntax in Arduino

//----------------------------------------------------------------------------------------
//  ESP32 Desired Bit Rate
//----------------------------------------------------------------------------------------
static const uint32_t DESIRED_BIT_RATE = 1000UL * 125UL ; // 125 Kb/s

//----------------------------------------------------------------------------------------
//   SETUP and Variables
//----------------------------------------------------------------------------------------
//Solution for no LED_BUILTIN
#define LED_BUILTIN 2 //Change LED Position if using different board
#define InfoPrint 0 //Change flag to show chip and BUS settings in the serial monitor 0-1
#define ErrorInfoPrint 1 //Change flag to show ErrorStatics of the CANBUS in the serial monitor 0-1
#define TimeInterval 30000 //Time interval for the system to trigger the bus scan process
#define RemoteFrameID 99 //ID for the remote frames to ask for information from the slaves
#define SlavesTurnOnDelay 5000 //Wait for the slaves to warm up and be able to send information
#define ArrayLimit 99 //Limit for the array storage of the queued messages, THIS IS LIMITED BY THE RAM MEMORY

//Counter for debugging and statistics of the BUS
static uint32_t ReceivedFrameCount = 0;
static uint32_t SentFrameCount = 0;
static uint32_t samplingTimeInterval = 0;
static uint32_t samplingTimeFreq = 100;
static uint32_t samplingCounter = 0;
int currentMessagesQueued = 0;

//Array for Queuing the received messages
CANMessage queuedMessages[ArrayLimit];
String timeStamps[ArrayLimit];

// Replace with network credentials
const char* ssid = "castro";
const char* password = "msnc5000";

// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";

//Your Domain name with URL path or IP address with path for HTTP Posting
const char* serverName = "https://europe-west1-gruenfacade.cloudfunctions.net/app/api/facade/test/sensordata";
//JSONVar jsonElements;

void setup() {
 //--- Switch on builtin led to know that the program is running correctly
  pinMode (LED_BUILTIN, OUTPUT);
  digitalWrite (LED_BUILTIN, HIGH);
  //--- Start serial and WiFi connection and well as NTP connection for EPOCH time
  Serial.begin (115200);
  initWiFi();
  configTime(0, 0, ntpServer);
  delay (100);
  //--- Configure ESP32 CAN
  ACAN_ESP32_Settings settings (DESIRED_BIT_RATE);
  settings.mRxPin = GPIO_NUM_4; // Optional, default Tx pin is GPIO_NUM_4
  settings.mTxPin = GPIO_NUM_5; // Optional, default Rx pin is GPIO_NUM_5
  const uint32_t errorCode = ACAN_ESP32::can.begin (settings);
  //Display ESP32 Chip and BUS Settings Info if the InfoPrint flag is active. Possible section for further debugging
  if(InfoPrint){
    esp_chip_info_t chip_info;
    chipInfo(chip_info);
    busInfo(errorCode,settings);
  }
}

//----------------------------------------------------------------------------------------
//   LOOP
//----------------------------------------------------------------------------------------
void loop () {
  //CANMessages initialization
  CANMessage remoteFrame; //Message for requesting data from the slaves
  remoteFrame.ext = false;
  remoteFrame.id = RemoteFrameID;
  remoteFrame.rtr = true;
  CANMessage frame; //No initialization needed, since the message will be read from the slaves
  int arrayIndex = 0; //Variable for the elements position in the arrays
  //Turn Power off for the slaves in order to come online
  if (samplingTimeInterval < millis ()) {
    samplingTimeInterval += TimeInterval;
                                            //Turn Power ON for the slaves in order to come online
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
    while(samplingCounter <= samplingTimeFreq){
      if (ACAN_ESP32::can.receive (frame)){
      queuedMessages[arrayIndex] = frame; //Add the received frames to a Queue in order to process them later
      timeStamps[arrayIndex] = getTime();
      ReceivedFrameCount += 1;
      arrayIndex += 1;
      currentMessagesQueued = arrayIndex;
      //framePrinting(frame); //Print the data of the received frames if desired
      }
      samplingCounter += 1;
      delay(100);
    }
                                                  //Turn Power off for the slaves in order to come online
    processQueuedMessages(queuedMessages); //Once the messages has been queued, they will be send
  }
}
//----------------------------------------------------------------------------------------
//Help/Debug functions

  //--- Display ESP32 Chip Info
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
        String sensorType = "";
        //According to the id of the CANMessage, sort the message to the corresponding sensor
        if(false){
          sensorType = "tempreture";
        }
        else if(true){
          sensorType = "humidity";
        }
        //Set every attribute of the JSONVar
        myObject["id"] = String(queuedMessages[i].id);
        myObject["type"] = sensorType;
        myObject["time"] = timeStamps[i];
        myObject["value"] = (float)55; //Implement the sensors in the boards and then use the data field of the currentMessagesQueued
        myObject["row"] = (long) queuedMessages[i].id;//Ask jeschke how we are gonna do this positioning
        myObject["column"] = (long) queuedMessages[i].id;
        //POST the JSON object
        int response = httpPOSTRequest(serverName,myObject);
        //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
        //Serial.println(response); //Print the response code if desired
      }
    }
    else {
    //Serial.println("WiFi Disconnected");
    }
  }

  int httpGETRequest(const char* serverName) {
    //Start the http connection
    HTTPClient http;
    http.begin(serverName);
    // Send HTTP GET request
    int httpResponseCode = http.GET();
    String payload = "{}"; //Variable to store the GET package
    JSONVar keys = "Failed";
    //Negative response codes are an error. 200 means OK
    if (httpResponseCode>0) {
      //Serial.print("HTTP Response code: "); //Display response code if desired
      //Serial.println(httpResponseCode);
      payload = http.getString();
      //Parse payload to JSON object in order to be able to use the data
      JSONVar myObject = JSON.parse(payload);
      // JSON.typeof(jsonVar) can be used to get the type of the var
      if (JSON.typeof(myObject) == "undefined") {
        Serial.println("Parsing input failed!");
        keys = "Failed";
      }
      else{
        // myObject.keys() can be used to get an array of all the keys in the object
        keys = myObject[0].keys();
        //Loop for printing the JSON elements if desired
        // for (int j = 0; j < myObject.length();j++){
        //   for (int i = 0; i < keys.length(); i++){
        //     Serial.print(keys[i]);
        //     Serial.print(": ");
        //     Serial.print(myObject[j][keys[i]]);
        //     Serial.print(" / ");
        //   }
        //   Serial.print("\n");
        // }
      }
    }
    else {
      //Serial.print("Error code: ");
      //Serial.println(httpResponseCode);
      keys = "Failed";
    }
    // Free resources
    http.end();
    return httpResponseCode;
  }

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
      Serial.println(json);
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
    time_t now;
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      return(String(0));
    }
    time(&now);
    //Convertion from UNIX seconds to ISO8601
    String timestamp = String(year(now)) + "-" + String(month(now)) + "-" + String(day(now)) + "T" + String(hour(now)) + ":" + String(minute(now)) + ":" + String(second(now)) + "+00:00";
    //Serial.println(timestamp);
    return timestamp;
  }

  // Initialize WiFi
  void initWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED){
      delay(1000);
    }
    //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
  }