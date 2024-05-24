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
% V21:    Changes in the programm to adapt the program to the ZSE PCB
%         Add library wire.h and methods for the PCAL6408A I2C I/O expander
%         Add methods to get the time from the PCF8523T IC Clock
%         findSlaves method was added as well as the eeprom writing/reading of the slaves matrix
%         canbus method was added to be able to turn different canbus channels
%         add the identifier for the row and column in the processquededmessages method for the system to sort out where the pots are located
%
%         -
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
#include <core_version.h>  // For ARDUINO_ESP32_RELEASE
#include "time.h"          // Library for the EPOCH TIME
#include <TimeLib.h>       // Library for time functions
#include <WiFi.h>          //Library for the WiFi Modul
#include <HTTPClient.h>    //Library for the http functions
#include <Arduino_JSON.h>  //Library for working with JSON syntax in Arduino
#include <Preferences.h>   //Library for storing and reading values from the EEPROM
#include <Wire.h>          //Library for the I2C communication protocol

//Setup-Global Variables
//Pins
#define PIN_IOEXPANDER 16           //GPIO PIN für die RESET PIN_IOEXPANDER, es muss HIGH sein, sonst startet der IC sich die ganze Zeit neu und kommuniziert nicht.
#define PIN_IOEXPANDERINTERRUPT 34  //GPIO PIN for the interrupt signals from the PCAL6408A expander This signal is defined to set the PIN as INPUT, but is it not really used, since the inputs of the PCAL6408A are read during the set up and NEVER again after that.
#define I2C_SDA 33                  //SDA GPIO PIN für die I2C Kommunikation
#define I2C_SCL 32                  //SCL GPIO PIN für die I2C Kommunikation
//Delays
#define I2CDelay 10             //Delay für die I2C Befehle
#define SlavesTurnOnDelay 5000  // (ms) Wait for the slaves to warm up and be able to send information
//Misc.
#define RemoteFrameID 99      //ID for the remote frames to ask for information from the slaves
#define findSlavesFrameID 69  //ID for the findSlaves frame to start the find slaves process
#define ArrayLimit 99         //Limit for the array storage of the queued messages, THIS IS LIMITED BY THE RAM MEMORY, so it cannot be too big, still need to check what are the limits.
#define Samplings 100

//Flags
bool InfoPrint = false;       //Change flag to show chip and BUS settings in the serial monitor 0-1
bool ErrorInfoPrint = false;  //Change flag to show ErrorStatics of the CANBUS in the serial monitor 0-1
bool findFlag = false;        //Find Flag for running the find method to find all the CAN devices in the line. Matrix principle
bool PCAL6408AFlag = false;   //Flag for the PCAL IC Loop, the flag is rewritten with the checkForI2CDevices method after establishing communication with the IC
bool CLOCKFlag = false;       //Flag for the CLOCK IC
bool nextSlaveFlag = false;   //Flag for a new found slave that should be added to the matrix

//Declaration of the variables
//Internet access // Replace with network credentials
//Zuhause
//const char* ssid = "nordisch-Box-1";
//const char* password = "87654321";
//Rechner - Büro
//const char* ssid = "castro";
//const char* password = "msnc5000";
//Labor
const char* ssid = "Mexicano";
const char* password = "Mexicano";

//I2C Parameters
int ADDRESS_PCF8523T = 104;                                  //Address in int, for some reason, if I do it in Byte and hex, it does not work  for the I2C CLOCK aquired with the SensorFindIC2Adress.ino script 0x68 = 104
int ADDRESS_PCAL6408A = 32;                                  //Address for the I/O expander aquired with the SensorFindIC2Adress.ino script 0x20 = 32
unsigned char PCAL6408ARegister = 0x03;                      //PCAL6408A register array for setting the configuration to turn on/off one I/O //Original unsigned char PCAL6408ARegister[] = {0x4F, 0x03, 0x01, 0x43};
unsigned char Konfiguration[] = { 0x7F, 0xBF, 0xDF, 0xEF };  //PCAL6408A configuration array for turning on the different I/O //Original { 0x80, 0x40, 0x20, 0x10 }
const uint32_t CANBUSlines = sizeof(Konfiguration);          //Count of the different CANBUS lines/states that should be turned on

//CANBUS variables
static const uint32_t DESIRED_BIT_RATE = 1000UL * 125UL;  // 125 Kb/s ESP32 Desired Bit Rate
unsigned long referenzMillis = 0;                         //Counter for the time loop
unsigned long TimeInterval = 1000 * 1 * 1;                //Time that should elapse between every loop
uint32_t samplingCounter = 0;                             //Counter for sampling the CANBUS line, this is needed in case that no message is received
uint32_t currentMessagesQueued = 0;                       //Counter for the queued messages in the array before processing them
CANMessage queuedMessages[ArrayLimit];                    //Array for Queuing the received messages
String timeStamps[ArrayLimit];                            //Array for storing the timestamp of every CANBUS message when they are received.
//static const char* const timeStamps[ArrayLimit]; //Check if it works, this way and if not then find a way to do not use STRING
//Counter for debugging and statistics of the BUS
uint32_t ReceivedFrameCount = 0;  //Counter for the received frames  from the slavesin the CANBUS line
uint32_t SentFrameCount = 0;      //Counter for the sent frames from the Master in the CANBUS line
const uint32_t maxSlavesInLine = 10;
uint32_t potsMatrix[CANBUSlines][maxSlavesInLine] = { { 0 } };  //Array for storing the matrix of the system pots

// NTP SERVER - API variables
const char* ntpServer = "pool.ntp.org";                                                                         //NTP server for getting the time online
const char* serverName = "https://europe-west1-gruenfacade.cloudfunctions.net/app/api/facade/test/sensordata";  //Your Domain name with URL path or IP address with path for HTTP Posting
//JSONVar jsonElements; //Used in an old programm, probably not needed anymore.
Preferences preferences;                //Variables and instances for the EEPROM rewrite/read procedure
const char* spaceEEPROM = "timestamp";  //variable for the EEPROM procedure
const char* paramEEPROM = "stamp";      //variable for the EEPROM procedure
char* CANBUSdata = "CANBUSdata";
char* CANBUSKeys[] = { "CB1Elements", "CB2Elements", "CB3Elements", "CB4Elements" };
unsigned long long CANBUSValues[] = { 0, 0, 0, 0 };

//----------------------------------------------------------------------------------------
void setup() {
  //Configure the OUTPUT PINs and signals for the board to work properly
  pinMode(PIN_IOEXPANDER, OUTPUT);          //Set the PIN for the PCAL6408A as OUTPUT
  pinMode(PIN_IOEXPANDERINTERRUPT, INPUT);  //Set the PIN for the PCAL6408A as INPUT
  digitalWrite(PIN_IOEXPANDER, HIGH);       //Set the PIN for the PCAL6408A HIGH, IF THIS IS NOT DONE; IT WILL NOT COMMUNICATE.
  Serial.begin(115200);                     // Start serial in case it is desired to debug or display any information
  //Serial.println("Setup started");  //Info
  initWiFi();                   // Init WiFi
  getEEPROMTimestamp();         //Get the last timestamp stored in the EEPROM in case that the NTP server cannot establish connecion.
  configTime(0, 0, ntpServer);  //Start NTP server connection for the UNIX TIME
  delay(100);
  //Display ESP32 Chip and BUS Settings Info if the InfoPrint flag is active. Possible section for further debugging
  if (InfoPrint) {
    esp_chip_info_t chip_info;
    chipInfo(chip_info);
  }
  //Start the I2C line on the ESP32 for the Clock and I/O expander
  Wire.begin(I2C_SDA, I2C_SCL);  //Begin the I2C transmition with the desired PINs
  delay(I2CDelay);
  //If the PCAL6408AFlag then the INPUTs will be checked for setting the right SW Version and the find funktion of the system.
  PCAL6408AFlag = checkForI2CDevices(ADDRESS_PCAL6408A);  //Scan for the PCAL6408A IC and proof communication
  if (PCAL6408AFlag) {
    readFindFlag();
    if (findFlag) {
      findSlaves();  //Call the findSlaves method and store the information in the ESP32 RAM for future starts
    } else {         //Read the Pflanfgefäße Matrix from the EEPROM
      //readSlavesMatrix();
    }
  } else {  //Read the Pflanfgefäße Matrix from the EEPROM
    //readSlavesMatrix();
  }
  Serial.println("Setup done");  //Info
}
//----------------------------------------------------------------------------------------
void loop() {
  //CANMessages initialization
  CANMessage remoteFrame;  //Message for requesting data from the slaves
  remoteFrame.ext = false;
  remoteFrame.id = RemoteFrameID;
  remoteFrame.rtr = true;
  CANMessage frame;                                //No initialization needed, since the message will be read from the slaves
  int arrayIndex = 0;                              //Variable for the elements position in the arrays
  Serial.println("Frames configuration done");     //Info
  if (millis() - referenzMillis > TimeInterval) {  // Overflow solution for the time problem.
    referenzMillis = millis();
    //If the PCAL6408AFlag then all different channels will be turned on respectively
    if (PCAL6408AFlag) {
      //For loop for starting the four different outputs in the Konfiguration array
      for (int i = 0; i < CANBUSlines; i++) {
        AusgangEinschalten(ADDRESS_PCAL6408A, Konfiguration[i]);            //Turn the desired output ON/OFF
        delay(SlavesTurnOnDelay);                                           //Wait for the slaves to start
        // const uint32_t errorCodeCANBUS = startCANBUSline(i);                //Start the CANBUS line, there are 4 different transceivers, so there are 4 different channels 1-4
        // const bool okRemoteFrame = ACAN_ESP32::can.tryToSend(remoteFrame);  //Sent remote frame to the BUS in order to get information from the Slaves
        // if (okRemoteFrame) {
        //   SentFrameCount += 1;
        //   samplingCounter = 0;
        //   arrayIndex = 0;
        //   currentMessagesQueued = 0;
        //   //Serial.print("Remote message sent succ. \n"); //Check if the message was sent succ.
        // }
        // displayBUSstatistics();                 //If flag ErrorInfoPrint is active, then statics information will be displayed in the serial monitor
        // while (samplingCounter <= Samplings) {  //Sample for a certain time for the slave's answers
        //   if (ACAN_ESP32::can.receive(frame)) {
        //     queuedMessages[arrayIndex] = frame;  //Add the received frames to a Queue in order to process them later
        //     timeStamps[arrayIndex] = getTime();
        //     ReceivedFrameCount += 1;
        //     arrayIndex += 1;
        //     currentMessagesQueued = arrayIndex;
        //     //framePrinting(frame); //Print the data of the received frames if desired
        //   }
        //   samplingCounter += 1;
        //   delay(100);  //Check how small this can get
        // }
        // checkWiFi();                               //Check for the WiFi connection before posting the messages
        // processQueuedMessages(queuedMessages, i);  //Once the messages has been queued, they will be send
        // delay(5000);
      }
    } else {
      //Brauchen wir nix machen, weil der IC nicht da ist, und die Kanäle nicht mit Strom versorgt werden können
    }
  }
  delay(1000);
}
//----------------------------------------------------------------------------------------
//Help/Debug functions
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
//Display BUS settings and or error codes
void busInfo(uint32_t errorCode, ACAN_ESP32_Settings settings) {
  if (errorCode == 0) {
    Serial.print("Bit Rate prescaler: ");
    Serial.println(settings.mBitRatePrescaler);
    Serial.print("Time Segment 1:     ");
    Serial.println(settings.mTimeSegment1);
    Serial.print("Time Segment 2:     ");
    Serial.println(settings.mTimeSegment2);
    Serial.print("RJW:                ");
    Serial.println(settings.mRJW);
    Serial.print("Triple Sampling:    ");
    Serial.println(settings.mTripleSampling ? "yes" : "no");
    Serial.print("Actual bit rate:    ");
    Serial.print(settings.actualBitRate());
    Serial.println(" bit/s");
    Serial.print("Exact bit rate ?    ");
    Serial.println(settings.exactBitRate() ? "yes" : "no");
    Serial.print("Distance            ");
    Serial.print(settings.ppmFromDesiredBitRate());
    Serial.println(" ppm");
    Serial.print("Sample point:       ");
    Serial.print(settings.samplePointFromBitStart());
    Serial.println("%");
    Serial.println("Configuration OK!");
  }
  //If there is an error code, then it should be displayed, In case an error persist, then the LED should blink
  else {
    Serial.print("Configuration error 0x");
    Serial.println(errorCode, HEX);
    //digitalWrite (LED_BUILTIN, LOW);
    delay(250);
    //digitalWrite (LED_BUILTIN, HIGH);
  }
}
//Statics information of the BUS will be displayed in the serial monitor
void displayBUSstatistics() {
  if (ErrorInfoPrint) {
    Serial.print("Sent: ");
    Serial.print(SentFrameCount);
    Serial.print(" ");
    Serial.print("Receive: ");
    Serial.print(ReceivedFrameCount);
    Serial.print(" ");
    Serial.print(" STATUS 0x");
    Serial.print(TWAI_STATUS_REG, HEX);
    Serial.print(" RXERR ");
    Serial.print(TWAI_RX_ERR_CNT_REG);
    Serial.print(" TXERR ");
    Serial.println(TWAI_TX_ERR_CNT_REG);
  }
}
//Print the frame data in the serial monitor
void framePrinting(CANMessage frame) {
  Serial.print("**** Received ");
  Serial.print(frame.ext ? "extended " : "standard ");
  Serial.print(frame.rtr ? "remote " : "data ");
  Serial.print("frame, id 0x");
  Serial.print(frame.id, HEX);
  Serial.print(", Data: ");
  for (int i = 0; i < frame.len; i++) {
    Serial.print(frame.data[i]);
    Serial.print(", ");
  }
  Serial.print("\n");
}
//Process the queued messages
void processQueuedMessages(CANMessage queuedMessages[], int row) {
  if (WiFi.status() == WL_CONNECTED) {
    //Initialize the JSON object from the queuedMessages Array
    for (int i = 0; i < currentMessagesQueued; i++) {
      JSONVar myObject;
      char* sensorType = "";
      float dataCANBUS = -99.99;
      //According to the id of the CANMessage, sort the message to the corresponding sensor
      if (queuedMessages[i].id >= 0xff00) {
        sensorType = "tempreture";
        dataCANBUS = scaleCANBUStemperature(queuedMessages[i].data32[0]);
      } else if (queuedMessages[i].id <= 0xee00) {
        sensorType = "humidity";
        dataCANBUS = scaleCANBUShumidity(queuedMessages[i].data32[0]);
      }
      //Set every attribute of the JSONVar
      myObject["id"] = String(queuedMessages[i].id);
      myObject["type"] = sensorType;
      myObject["time"] = timeStamps[i];
      myObject["value"] = dataCANBUS;                          //Implement the sensors in the boards and then use the data field of the currentMessagesQueued
      myObject["row"] = row;                                   //Ask jeschke how we are gonna do this positioning
      int column = getSlaveColumn(row, queuedMessages[i].id);  //Method for getting the column of the device, according to the frame.id
      myObject["column"] = column;
      //POST the JSON object
      int response = httpPOSTRequest(serverName, myObject);
      //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
      //Serial.println(response); //Print the response code if desired
    }
  } else {
    checkWiFi();  //Check for the WiFi connection before posting the messages
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
int httpPOSTRequest(const char* serverName, JSONVar myObject) {
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
    String json = "{ \"sensordata\":[" + JSON.stringify(myObject) + "]}";
    //Serial.println(json);
    httpResponseCode = http.POST(json);
    //"{ \"sensordata\":[]}"
  } else {
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
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", year(), month(), day(), hour(), minute(), second());
    //Clock procedure for rewriting the timestmap in the IC Clock
    CLOCKFlag = checkForI2CDevices(ADDRESS_PCF8523T);  ////Scan for the Clock IC and proof communication
    if (CLOCKFlag) {                                   //If flag is active, the the time from the clock should be overwritten
      //register call
      Wire.write(0x03);  //It is just needed to call the first register, after that, all other registers are auto incremented
      //First overwrite register
      Wire.write(second());    //Seconds in BCD format
      Wire.write(minute());    //Minutes in BCD format
      Wire.write(hour());      //hours in BCD format
      Wire.write(day());       //days in BCD format
      Wire.write(0x00);        //week days  in BCD format
      Wire.write(month());     //month in BCD format
      Wire.write(year());      //2024 Year in HEX according to the BCD format, check the data sheet for more information.
      Wire.endTransmission();  // "Hang up the line" (can have multiple slaves & masters connected)^
      delay(I2CDelay);
    }
    overwriteEEPROMTimestamp(now());
    return (timestamp);
  } else {
    //Get the time stamp in the ISO8601 format
    //YYYY-MM-DDTHH:MM:SS+00:00
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour + 1, timeinfo.tm_min, timeinfo.tm_sec);
    //Overwrite the EEPROMtimeStamp with the current value from the NTP server to refresh it with new data
    setTime(timeinfo.tm_hour + 1, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);
    //Clock procedure for rewriting the timestmap in the IC Clock
    CLOCKFlag = checkForI2CDevices(ADDRESS_PCF8523T);  ////Scan for the Clock IC and proof communication
    if (CLOCKFlag) {                                   //If flag is active, the the time from the clock should be overwritten
      //register call
      Wire.write(0x03);  //It is just needed to call the first register, after that, all other registers are auto incremented
      //First overwrite register
      Wire.write(second());    //Seconds in BCD format
      Wire.write(minute());    //Minutes in BCD format
      Wire.write(hour());      //hours in BCD format
      Wire.write(day());       //days in BCD format
      Wire.write(0x00);        //week days  in BCD format
      Wire.write(month());     //month in BCD format
      Wire.write(year());      //2024 Year in HEX according to the BCD format, check the data sheet for more information.
      Wire.endTransmission();  // "Hang up the line" (can have multiple slaves & masters connected)^
      delay(I2CDelay);
    }
    overwriteEEPROMTimestamp(now());
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
    //Serial.println("Not connected, Trying to connect");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    delay(2000);
  } else {
    //WiFi was already connected
  }
  //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
}
//Read EEPROM-Timestamp value
void getEEPROMTimestamp() {
  CLOCKFlag = checkForI2CDevices(ADDRESS_PCF8523T);  ////Scan for the Clock IC and proof communication
  if (CLOCKFlag) {                                   //If Flag is active, then get the time from the IC - CLOCK PCF8523T, if it is not active, then get the time from the EEPROM
    //Get seconds
    Wire.beginTransmission(ADDRESS_PCF8523T);  //"notify" the matching device
    Wire.write(0x03);                          //  The command byte, sets pointer to register with address of 0xXX
    Wire.endTransmission();                    // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
    delay(I2CDelay);
    Wire.requestFrom(ADDRESS_PCF8523T, 1);  // Tell slave we need to read 1byte from the current register
    byte slaveByte2 = Wire.read();          // read that byte into 'slaveByte2' variable
    int date[13];
    date[0] = slaveByte2 & 0b1111;
    date[1] = (slaveByte2 >> 4) & 0b111;
    delay(I2CDelay);
    Wire.requestFrom(ADDRESS_PCF8523T, 1);  // Tell slave we need to read 1byte from the current register
    slaveByte2 = Wire.read();               // read that byte into 'slaveByte2' variable
    date[2] = slaveByte2 & 0b1111;
    date[3] = (slaveByte2 >> 4) & 0b111;
    delay(I2CDelay);
    Wire.requestFrom(ADDRESS_PCF8523T, 1);  // Tell slave we need to read 1byte from the current register
    slaveByte2 = Wire.read();               // read that byte into 'slaveByte2' variable
    date[4] = slaveByte2 & 0b1111;
    date[5] = (slaveByte2 >> 4) & 0b11;
    delay(I2CDelay);
    Wire.requestFrom(ADDRESS_PCF8523T, 1);  // Tell slave we need to read 1byte from the current register
    slaveByte2 = Wire.read();               // read that byte into 'slaveByte2' variable
    date[6] = slaveByte2 & 0b1111;
    date[7] = (slaveByte2 >> 4) & 0b11;
    delay(I2CDelay);
    Wire.requestFrom(ADDRESS_PCF8523T, 1);  // Tell slave we need to read 1byte from the current register
    slaveByte2 = Wire.read();               // read that byte into 'slaveByte2' variable
    date[8] = slaveByte2 & 0b111;
    delay(I2CDelay);
    Wire.requestFrom(ADDRESS_PCF8523T, 1);  // Tell slave we need to read 1byte from the current register
    slaveByte2 = Wire.read();               // read that byte into 'slaveByte2' variable
    date[9] = slaveByte2 & 0b1111;
    date[10] = (slaveByte2 >> 4) & 0b1;
    delay(I2CDelay);
    Wire.requestFrom(ADDRESS_PCF8523T, 1);  // Tell slave we need to read 1byte from the current register
    slaveByte2 = Wire.read();               // read that byte into 'slaveByte2' variable
    date[11] = slaveByte2 & 0b1111;
    date[12] = (slaveByte2 >> 4) & 0b1111;
    Wire.endTransmission();  // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
    delay(I2CDelay);
    setTime((date[5] * 10 + date[4]), (date[3] * 10 + date[2]), (date[1] * 10 + date[0]), (date[7] * 10 + date[6]), (date[10] * 10 + date[9]), (date[12] * 10 + date[11] + 2000));
    delay(980);
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
//Scaling for the CANBUS temperature Data
float scaleCANBUStemperature(uint32_t data32) {
  //Variables for the CANBUS Data scaling
  uint32_t maxScaling = (2 << 15) - 1;
  int minScaling = 0;
  int tempA = -100;
  int tempB = 100;
  float result = ((((tempB - tempA) * (float(data32) - minScaling)) / (maxScaling - minScaling)) + tempA);
  return result;
}
//Scaling for the CANBUS humidity Data
float scaleCANBUShumidity(uint32_t data32) {
  //Variables for the CANBUS Data scaling
  uint32_t maxScaling = (2 << 15) - 1;
  int minScaling = 0;
  int humidA = 0;
  int humidB = 100;
  float result = ((((humidB - humidA) * (float(data32) - minScaling)) / (maxScaling - minScaling)) + humidA);
  return result;
}
//Method for turning the I/O PCAL6408A ON/OFF
void AusgangEinschalten(byte ADDRESS_PCAL6408A, byte data) {
  Wire.beginTransmission(ADDRESS_PCAL6408A);  //"notify" the matching device
  Wire.write(PCAL6408ARegister);              //  The command byte, sets pointer to register with address of 0xXX
  Wire.write(data);                           //Je nach Konfiguration und oder Wünsch, muss das hier angepasst werden
  Wire.endTransmission();                     // "Hang up the line" (can have multiple slaves & masters connected)^
  delay(I2CDelay);                            //SCHAUEN WIE KLEIN/GROß DER DELAY WERDEN KANN
}
//Method scanning for a I2C device with specific adress in the line
bool checkForI2CDevices(byte address) {
  bool flag = false;                    //function Flag
  Wire.beginTransmission(address);      //Start the transmision
  byte error = Wire.endTransmission();  //Check if the transmition ended with errors
  if (error == 0) {                     //0 means no error
    flag = true;
  } else {  //If any error, then check the PCAL6408A
    flag = false;
  }
  return flag;
}
//Method for finding the different slaves on the line, the 4 ON/OFF outputs of the IC Expander should be switched, as well as the CANBUS lines, in order to communicate with the devices.
void findSlaves() {
  Serial.println("Starting find slaves");  //Info
  for (int i = 0; i < CANBUSlines; i++) {                     //Loop for starting the 4 different channels
    Serial.println("Turning on slave");  //Info
    AusgangEinschalten(ADDRESS_PCAL6408A, Konfiguration[i]);  //Turn the desired output ON/OFF
    Serial.println("Slave was turned on");  //Info
    delay(SlavesTurnOnDelay);                                 //Wait for the slaves to start
    // const uint32_t errorCodeCANBUS = startCANBUSline(i);      //Start the CANBUS line, there are 4 different transceivers, so there are 4 different channels 1-4
    // CANMessage findSlavesFrame;                               //Frame configuration for the find process from the slaves
    // findSlavesFrame.ext = false;
    // findSlavesFrame.id = findSlavesFrameID;
    // findSlavesFrame.rtr = true;
    // for (int j = 0; j < maxSlavesInLine; j++) {                                   //Loop for doing the process as many times as maxSlavesInLine, since the slaves are turned on, one by one.
    //   const bool okfindSlavesFrame = ACAN_ESP32::can.tryToSend(findSlavesFrame);  //Sent remote frame to the BUS in order to start the find process
    //   CANMessage findSlavesAnswer;                                                //No initialization needed, since the message will be read from the slaves
    //   while (samplingCounter <= Samplings) {                                      //Sample for a certain time for the slave's answers
    //     if (ACAN_ESP32::can.receive(findSlavesAnswer)) {                          //If got reponse from the slave then process the information
    //       int tempArray[maxSlavesInLine] = { 0 };                                 //Initialize array to store the current values of the current row i-CANBUSlines
    //       for (int k = 0; k < maxSlavesInLine; k++) {
    //         tempArray[k] = potsMatrix[i][k];  //Store the current row i-CANBUSlines in the temp array.
    //       }
    //       nextSlaveFlag = valueInArray(findSlavesAnswer.id >> 8, tempArray, maxSlavesInLine);  //Check if the id of the received message is contained int the temp array or not
    //       if (nextSlaveFlag) {                                                                 //If the id of the received message is new, then store the id in the potsMatrix array
    //         potsMatrix[i][j] = findSlavesAnswer.id >> 8;                                       //Store the answer in a 2D Matrix for mapping the Pflanzgefäße
    //         samplingCounter = 100;                                                             //Set the counter to 100, since a new element was added and we can start again.
    //       }
    //     }
    //     samplingCounter += 1;  //Increment the counter to get out of the loop after Samplings is over. This is actually not needed, but it helps to avoid to stay in this while inc ase something goes wrong.
    //     delay(100);            //Check how small this can get
    //   }
    //   delay(SlavesTurnOnDelay);  //Once a message was received, wait for the next device to start. Then start all over again.
    // }
  }
  Serial.println("trying to write on eeprom");  //Info
  // for (int m = 0; m < CANBUSlines; m++) {  //Loop for starting the 4 different channels
  //   char buffer[20];
  //   Serial.println("generating buffer");  //Info
  //   sprintf(buffer, "%02d%02d%02d%02d%02d%02d%02d%02d%02d%02d", potsMatrix[m][0], potsMatrix[m][1], potsMatrix[m][2], potsMatrix[m][3], potsMatrix[m][4], potsMatrix[m][5], potsMatrix[m][6], potsMatrix[m][7], potsMatrix[m][8], potsMatrix[m][9]);  //Der potsMatrix[i][j] muss nach dem Vorgang in den EEPROM gespeichert werden
  //   Serial.println("buffer generated");  //Info
  //   unsigned long long bufferint = atoll(buffer);
  //   Serial.println("buffer converted");  //Info
  //   CANBUSValues[m] = bufferint;                       //Store the buffer in the matrix
  //   Serial.println("buffer set");  //Info
  //   preferences.begin(CANBUSdata, false);              //init preference to overwrite EEPROM values with the new matrix
  //   Serial.println("write the keys in the eepprom");  //Info
  //   preferences.putULong64(CANBUSKeys[m], bufferint);  // Write the parameter with the following value.
  //   Serial.println("written");  //Info
  //   preferences.end();
  //   Serial.println("preferences were closed");  //Info
  // }
}
//Method for reading the matrix configuration from the EEPROM memory and storing it in the potsMatrix[][]
void readSlavesMatrix() {
  for (int m = 0; m < CANBUSlines; m++) {                                 //Loop for starting the 4 different channels
    preferences.begin(CANBUSdata, false);                                 //init preference to overwrite EEPROM values with the new matrix
    unsigned long long value = preferences.getULong64(CANBUSKeys[m], 0);  // Write the parameter with the following value.
    preferences.end();
    CANBUSValues[m] = value;  //Store the buffer in the CANBUS EEPROM variable
    char buffer[20];
    sprintf(buffer, "%llu", value);  //Get the readed value from the EEPROM into string to divide it and store it in the matrix.
    for (int n = 0; n < maxSlavesInLine; n++) {
      char minibuffer[2];
      sprintf(minibuffer, "%s%s", buffer[n * 2], buffer[(n * 2) + 1]);  //Get the readed value from the EEPROM into string to divide it and store it in the matrix.
      potsMatrix[m][n] = atoi(minibuffer);                              //Convert the string to int and store it in the matrix.
    }
  }
}
//Method for checking if a value is already in an array.
bool valueInArray(int id, int* arr, int elements) {
  for (int l = 0; l < elements; l++) {
    if (arr[l] == id) {
      return true;
    }
  }
  return false;
}
//Method for reading the findFlag of the ZSE unit
void readFindFlag() {
  //The Interrupt mask register 0x45 should be set after start up with the configuration for the pins where we want to receive interrupts.
  //Mask for the four inputs in this system is 0xF8(0B11111000), for more information reffer to the data sheet.
  //Not needed, since the interrupt will not be used. It will just be read the initial conditions of the inputs.
  Wire.beginTransmission(ADDRESS_PCAL6408A);  //"notify" the matching device
  Wire.write(0x00);                           //  Interrupt status register (46h)
  Wire.endTransmission();                     // "Hang up the line" so others can use it (can have multiple slaves & masters connected)
  delay(I2CDelay);
  Wire.requestFrom(ADDRESS_PCAL6408A, 1);  // Tell slave we need to read 1byte from the current register
  byte slaveByte2 = Wire.read();           // read that byte into 'slaveByte2' variable
  delay(I2CDelay);
  findFlag = slaveByte2 & 0B1;  //findFlag stores the value from the LSB from the register 0x00
  //int swVersion = slaveByte2 & 0B110;  //hwVersion variable, stores the version of the SW that should be run. There are 2 bits for that, which means 4 possible values (0,2,4,6);
}
//Method for starting one of the CANBUS channels 1-4
int startCANBUSline(int numberCANBUS) {
  ACAN_ESP32_Settings settings(DESIRED_BIT_RATE);
  switch (numberCANBUS) {             //Accordung to numberCANBUS switch the correct CANBUS line.
    case 1:                           //CANBUS1 configuration pins for the different channels
      settings.mRxPin = GPIO_NUM_25;  // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_26;  // Optional, default Tx pin is GPIO_NUM_5
      break;
    case 2:                           //CANBUS2
      settings.mRxPin = GPIO_NUM_27;  // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_14;  // Optional, default Tx pin is GPIO_NUM_5
      break;
    case 3:                           //CANBUS3
      settings.mRxPin = GPIO_NUM_12;  // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_13;  // Optional, default Tx pin is GPIO_NUM_5
      break;
    case 4:                           //CANBUS4
      settings.mRxPin = GPIO_NUM_15;  // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_4;   // Optional, default Tx pin is GPIO_NUM_5
      break;
  }
  const uint32_t errorCode = ACAN_ESP32::can.begin(settings);
  //Display CANBUS Settings Info if the InfoPrint flag is active. Possible section for further debugging
  if (InfoPrint) {
    busInfo(errorCode, settings);
  }
  return errorCode;
}
//Method for finding the column of an slave in the matrix, according to its ID and row
int getSlaveColumn(int row, int slaveID) {
  int generalID = slaveID >> 8;  //nur die zwei linke Bytes vom ID berücksichtigen
  for (int j = 0; j < maxSlavesInLine; j++) {
    if (generalID == potsMatrix[row][j]) {
      return j + 1;
    }
  }
  return 99;
}