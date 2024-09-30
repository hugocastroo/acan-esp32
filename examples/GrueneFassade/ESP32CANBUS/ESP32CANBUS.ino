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
%         -Add library wire.h and methods for the PCAL6408A I2C I/O expander
%         -Add methods to get the time from the PCF8523T IC Clock
%         -FindSlaves method was added as well as the eeprom writing/reading of the slaves matrix
%         -Canbus method was added to be able to turn different canbus channels
%         -Add the identifier for the row and column in the processquededmessages method for the system to sort out where the pots are located
%         -Add the <ModbusMaster.h> library and configuration needed for the RSMAX485 - weather station Adapter.
%         -Reorganize the gettime Method and the timestamp logic.
%         -Testing the code with the first slaves, was succesfull
%
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
#include <ModbusMaster.h>  //Library for the MODBUS 485 IC in order to communicate with the Weather station
//Setup-Global Variables
//Pins
//#define PIN_IOEXPANDER 16  //GPIO PIN für die RESET PIN_IOEXPANDER, es muss HIGH sein, sonst startet der IC sich die ganze Zeit neu und kommuniziert nicht.
#define I2C_SDA 33         //SDA GPIO PIN für die I2C Kommunikation
#define I2C_SCL 32         //SCL GPIO PIN für die I2C Kommunikation
#define MAX485_DE_RE 16    //Pin for toggling the MAX485 Board //PIN_IOEXPANDER muss gelöscht werden, bei der nächsten Platine.
#define MAX485_RX 18       //Pin for the RX communication RO RS485MAX
#define MAX485_TX 19       //Pin for the TX communication DI RS485MAX
//Delays
#define I2CDelay 10             //Delay in ms für die I2C Befehle
#define SlavesTurnOnDelay 1000  // (ms) Wait for the slaves to warm up and be able to send information
#define wateringDelay 15000     //Delay in ms for waiting for the valve to water the pot
#define overflowTimer 3000      //Delay in ms for the overflow while looking for new slaves.
//Misc.
int infoFrameID = 90;        //ID for the remote frames to ask for information from the slaves DEC 90 - HEX 5A
int findSlavesFrameID = 91;  //ID for the findSlaves frame to start the find slaves process    DEC 91 - HEX 5B
int requestFrameID = 92;     //ID for the findSlaves frame to start the find slaves process		DEC 92 - HEX 5C
#define ArrayLimit 99        //Limit for the array storage of the queued messages, THIS IS LIMITED BY THE RAM MEMORY, so it cannot be too big, still need to check what are the limits.
#define Samplings 100        //Number of samplings that should be done in order to scan for CANBUS messages
#define humidityTreshold 30  //Treshold for the humidity to start the ventils or not.
//Flags
bool InfoPrint = false;            //Change flag to show chip and BUS settings in the serial monitor 0-1
bool ErrorInfoPrint = true;        //Change flag to show ErrorStatics of the CANBUS in the serial monitor 0-1
bool findFlag = false;             //Find Flag for running the find method to find all the CAN devices in the line. Matrix principle
bool PCAL6408AFlag = false;        //Flag for the PCAL IC Loop, the flag is rewritten with the checkForI2CDevices method after establishing communication with the IC
bool CLOCKFlag = false;            //Flag for the CLOCK IC
bool nextSlaveFlag = false;        //Flag for a new found slave that should be added to the matrix
bool wateringFlag = false;         //Flag for the system to know if it is needed to trigger and extra delay in order to wait for the channel to finish watering
bool findMessageReceived = false;  //Flag for identifing when a find message was received and stop scaning for a message
bool weatherStationFlag = false;   //Flag for knowing if data from the weather station was received or not
//Internet access // Replace with network credentials
const char* ssid = "nordisch-Box-1";
const char* password = "87654321";
//const char* ssid = "castro";
//const char* password = "msnc5000";
//const char* ssid = "Mexicano";
//const char* password = "Mexicano";
//I2C Parameters
int ADDRESS_PCF8523T = 104;                          //Address in int, for some reason, if I do it in Byte and hex, it does not work  for the I2C CLOCK aquired with the SensorFindIC2Adress.ino script 0x68 = 104
int ADDRESS_PCAL6408A = 32;                          //Address for the I/O expander aquired with the SensorFindIC2Adress.ino script 0x20 = 32
unsigned char PCAL6408ARegister = 0x03;              //PCAL6408A register array for setting the configuration to turn on/off one I/O //Original unsigned char PCAL6408ARegister[] = {0x4F, 0x03, 0x01, 0x43};
unsigned char Konfiguration[] = { 0x7F, 0xBF };      //PCAL6408A configuration array for turning on the different I/O //Original { 0x80, 0x40, 0x20, 0x10 } { 0x7F, 0xBF, 0xDF, 0xEF }
const uint32_t CANBUSlines = sizeof(Konfiguration);  //Count of the different CANBUS lines/states that should be turned on
//CANBUS variables
static const uint32_t DESIRED_BIT_RATE = 1000UL * 125UL;  // 125 Kb/s ESP32 Desired Bit Rate
unsigned long referenzMillis = 1000 * 60 * 59;             //Counter for the time loop
unsigned long TimeInterval = 1000 * 60 * 60;               //Time that should elapse between every loop
uint32_t currentMessagesQueued = 0;                       //Counter for the queued messages in the array before processing them
CANMessage queuedMessages[ArrayLimit];                    //Array for Queuing the received messages
String timeStamps[ArrayLimit];                            //Array for storing the timestamp of every CANBUS message when they are received.
//static const char* const timeStamps[ArrayLimit]; //Check if it works, this way and if not then find a way to do not use STRING
//Counter for debugging and statistics of the BUS
uint32_t ReceivedFrameCount = 0;  //Counter for the received frames  from the slavesin the CANBUS line
uint32_t SentFrameCount = 0;      //Counter for the sent frames from the Master in the CANBUS line
const uint32_t maxSlavesInLine = 3;
uint32_t potsMatrix[CANBUSlines][maxSlavesInLine] = { { 0 } };  //Array for storing the matrix of the system pots
//Weather station variables
float weatherStationHumidity = 0;
float weatherStationTemperature = 0;
int weatherStationLux = 0;
ModbusMaster node;  //object node for class ModbusMaster
// NTP SERVER - API variables
const char* ntpServer = "pool.ntp.org";                                                                         //NTP server for getting the time online
const char* serverName = "https://europe-west1-gruenfacade.cloudfunctions.net/app/api/facade/test/sensordata";  //Your Domain name with URL path or IP address with path for HTTP Posting
Preferences preferences;                //Variables and instances for the EEPROM rewrite/read procedure
const char* spaceEEPROM = "timestamp";  //variable for the EEPROM procedure
const char* paramEEPROM = "stamp";      //variable for the EEPROM procedure
char* CANBUSdata = "CANBUSdata";
char* CANBUSKeys[] = { "CB1Elements", "CB2Elements", "CB3Elements", "CB4Elements" };
unsigned long long CANBUSValues[] = { 0, 0, 0, 0 };

void setup() {
  //Configure the OUTPUT PINs and signals for the board to work properly
  //pinMode(PIN_IOEXPANDER, OUTPUT);     //Set the PIN for the PCAL6408A as OUTPUT
  //digitalWrite(PIN_IOEXPANDER, HIGH);  //Set the PIN for the PCAL6408A HIGH, IF THIS IS NOT DONE; IT WILL NOT COMMUNICATE.
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
  //Start the I2C line on the ESP32 for the Clock and I/O expander
  Wire.begin(I2C_SDA, I2C_SCL);  //Begin the I2C transmition with the desired PINs
  delay(I2CDelay);
  //weatherStationSetUp(); //Configure RS485 Modbus channel for the weather station
  //If the PCAL6408AFlag then the INPUTs will be checked for the find function of the system.
  PCAL6408AFlag = checkForI2CDevices(ADDRESS_PCAL6408A);  //Scan for the PCAL6408A IC and proof communication
  if (PCAL6408AFlag) {
    readFindFlag();
    if (findFlag) {
      findSlaves();  //Call the findSlaves method and store the information in the ESP32 RAM for future starts
    } else {         //Read the Pflanfgefäße Matrix from the EEPROM
      readSlavesMatrix();
    }
  } else {  //Read the Pflanfgefäße Matrix from the EEPROM
    readSlavesMatrix();
  }
  Serial.println("Setup done");  //Info
}
void loop() {
  //CANMessages initialization
  CANMessage infoFrame;  //Message for requesting data from the slaves
  infoFrame.ext = false;
  infoFrame.id = infoFrameID;
  infoFrame.rtr = true;
  CANMessage frame;                                //No initialization needed, since the message will be read from the slaves
  int arrayIndex = 0;                              //Variable for the elements position in the arrays
  if (millis() - referenzMillis > TimeInterval) {  // Overflow solution for the time problem.
    Serial.println("Time to sample");              //Info
    referenzMillis = millis();
    if (PCAL6408AFlag) {                                          //If the PCAL6408AFlag then all different channels will be turned on respectively
      for (int i = 0; i < CANBUSlines; i++) {                     //For loop for starting the four different outputs in the Konfiguration array
        AusgangEinschalten(ADDRESS_PCAL6408A, Konfiguration[i]);  //Turn the desired output ON/OFF
        const uint32_t errorCodeCANBUS = startCANBUSline(i + 1);  //Start the CANBUS line, there are 4 different transceivers, so there are 4 different channels 1-4
        delay(SlavesTurnOnDelay);                                 //Wait for the slaves to start
        arrayIndex = 0;
        currentMessagesQueued = 0;
        for (int j = 0; j < maxSlavesInLine; j++) {
          const bool okinfoFrame = ACAN_ESP32::can.tryToSend(infoFrame);  //Sent remote frame to the BUS in order to get information from the Slaves
          if (okinfoFrame) {                                              //If the message was sent, then set the counters or increment them.
            SentFrameCount += 1;
            delay(500);  //It can be that this delay needs to be bigger if the CANBUS lines gets really long
            while (ACAN_ESP32::can.receive(frame)) {
              queuedMessages[arrayIndex] = frame;  //Add the received frames to a Queue in order to process them later
              timeStamps[arrayIndex] = getTime();
              ReceivedFrameCount += 1;
              arrayIndex += 1;
              currentMessagesQueued = arrayIndex;
              //framePrinting(frame); //Print the data of the received frames if desired
              Serial.print(frame.id, HEX);
              Serial.print(", ");
            }
          }
        }
        Serial.print("\n");
        displayBUSstatistics();                    //If flag ErrorInfoPrint is active, then statics information will be displayed in the serial monitor
        checkWiFi();                               //Check for the WiFi connection before posting the messages
        Serial.println("Processing the data");     //Info
        processQueuedMessages(queuedMessages, i);  //Once the messages has been queued, they will be send
        Serial.println("Data processed");          //Info
        if (wateringFlag) {                        //If the flag was set while processing the queuedMessages, then wait for the valves to close, before shutting down the slaves.
          Serial.println("Watering");              //Info
          delay(wateringDelay);
          wateringFlag = false;  //Set the wateringFlag to false for the next cycle.
        }
        AusgangEinschalten(ADDRESS_PCAL6408A, 0xff);  //Shut all channels down after the cycle.
        Serial.println("Finished the cycle");              //Info
      }
    }
  }
  delay(1000);
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
      //According to the id of the CANMessage, sort the message to the corresponding sensor 0xXX01 is humidity sensor, 0xXX02 is temperature sensor. XX is the Slave ID and should be different for every slave.
      int currentID = queuedMessages[i].id & 0B11111111;  //Take just the 8  first LSB, since the other bits are the Slave ID
      switch (currentID) {
        case 1:
          sensorType = "humidity";
          dataCANBUS = scaleCANBUShumidity(queuedMessages[i].data32[0]);
          Serial.print("Humidity: ");
          Serial.println(dataCANBUS);
        break;
        case 2:
          sensorType = "tempreture";
          dataCANBUS = scaleCANBUStemperature(queuedMessages[i].data32[0]);
          Serial.print("Temperature: ");
          Serial.println(dataCANBUS);
        break;
        case 3:
          sensorType = "internTempreture";
          dataCANBUS = scaleCANBUStemperature(queuedMessages[i].data32[0]);
          Serial.print("internTemperature: ");
          Serial.println(dataCANBUS);
        break;
        default:
          sensorType = "humidity";
          dataCANBUS = 99;
          break;
      }
      //Set every attribute of the JSONVar
      myObject["id"] = String(queuedMessages[i].id);
      myObject["type"] = sensorType;
      myObject["time"] = timeStamps[i];
      myObject["value"] = dataCANBUS;                          //Implement the sensors in the boards and then use the data field of the currentMessagesQueued
      myObject["row"] = row + 1;                               //Set the row value to the object using the CANBUSlines parameter given with row, + 1 since it starts in 0
      int column = getSlaveColumn(row, queuedMessages[i].id);  //Method for getting the column of the device, according to the frame.id
      myObject["column"] = column;
      int response = httpPOSTRequest(serverName, myObject);  //POST the JSON object
      if (dataCANBUS < humidityTreshold) {
        wateringFlag = true;
      }
      if(response == 200){
        Serial.println("posted");
      }
      else{
        //while(response != 200){
          //response = httpPOSTRequest(serverName, myObject);  //POST the JSON object
          Serial.print("Not posted error code: ");
          Serial.println(response);
        //}
      }

      // Serial.print("Row: ");
      // Serial.print(row + 1);
      // Serial.print(" Column: ");
      // Serial.println(column);
      //Find a solution for a negative response in case that the message has not been posted, in case that it is needed to store them somewhere, then think about expanding the memory
      //Serial.println(response); //Print the response code if desired
    }
  } else {
    checkWiFi();  //Check for the WiFi connection before posting the messages
    Serial.println("WiFi Disconnected");
  }
  //Before finishing this method, the information from the weather station should be gather and posted also in the backend. We will need new variables and parameters in the backend to acomplish this.
  // weatherStationFlag = weatherStationGetData();
  // if (weatherStationFlag) {  //Ask Jeschke and Robert which variables will be used and after the variables are set in the backend, then I can upload them. For now, this will be uncommented.
  //   // JSONVar myObject;
  //   // char* sensorType = "humidityWS";
  //   // float dataWS = weatherStationHumidity;
  //   // //Set every attribute of the JSONVar
  //   // myObject["id"] = 10;
  //   // myObject["type"] = sensorType;
  //   // myObject["time"] = getTime();
  //   // myObject["value"] = dataWS;
  //   // int response = httpPOSTRequest(serverName, myObject);  //POST the JSON object
  //   Serial.print("Humidity: ");
  //   Serial.print(weatherStationHumidity);
  //   Serial.println("%");
  //   Serial.print("Temp: ");
  //   Serial.print(weatherStationTemperature);
  //   Serial.println("°C");
  //   Serial.print("Illuminance: ");
  //   Serial.print(weatherStationLux);
  //   Serial.println("Lux");
  // }
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
  //Get the current time in the device since the NTP connection failed. after getting the current time in the device, store the current time in the EEPROM for future processing in case the device goes off
  if (!getLocalTime(&timeinfo)) {
    getEEPROMTimestamp();                                                                                           //Set the time before reading it, according to the clock IC if available, if not then from the EEPROM
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", year(), month(), day(), hour(), minute(), second());  //Get the time stamp in the ISO8601 format YYYY-MM-DDTHH:MM:SS+00:00
    return (timestamp);
  } else {                                                                                                                                                                              //If it was possible to get the information from the NTP server, then use this information and store it in the EEPROM and sync the IC Clock Get the time stamp in the ISO8601 format YYYY-MM-DDTHH:MM:SS+00:00
    sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour + 2, timeinfo.tm_min, timeinfo.tm_sec);  //Store the NTP data in the timestamp variable
    setTime(timeinfo.tm_hour + 2, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900);                                                    //Overwrite the internal clock of the ESP32 with the current value from the NTP server to refresh it with new data
    CLOCKFlag = checkForI2CDevices(ADDRESS_PCF8523T);                                                                                                                                   ////Scan for the Clock IC and proof communication
    if (CLOCKFlag) {                                                                                                                                                                    //If flag is active, the the time from the clock should be overwritten
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
  float maxScaling = (2 << 14) - 1;
  float minScaling = 0.0;
  float tempA = 0.0;  //Check if these values are correct or not, before it was apparently working with -100 and 100
  float tempB = 150.0; //Check if these values would work for the extern and intern temperature sensor or just with one sensor
  float result = ((((tempB - tempA) * (float(data32) - minScaling)) / (maxScaling - minScaling)) + tempA);
  return result;
}
//Scaling for the CANBUS humidity Data
float scaleCANBUShumidity(uint32_t data32) {
  //Variables for the CANBUS Data scaling
  float maxScaling = (2 << 14) - 1;
  float minScaling = 0.0;
  float humidA = 0.0;
  float humidB = 100.0;
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
  Serial.println("Starting find slaves");                     //Info
  for (int i = 0; i < CANBUSlines; i++) {                     //Loop for starting the 4 different channels
    AusgangEinschalten(ADDRESS_PCAL6408A, Konfiguration[i]);  //Turn the desired output ON/OFF
    const uint32_t errorCodeCANBUS = startCANBUSline(i + 1);  //Start the CANBUS line, there are 4 different transceivers, so there are 4 different channels 1-4
    delay(SlavesTurnOnDelay);                                 //Wait for the slaves to start, around two seconds should be fine, if not then 5 seconds
    CANMessage findSlavesFrame;                               //Frame configuration for the find process from the slaves
    findSlavesFrame.ext = false;
    findSlavesFrame.id = findSlavesFrameID;
    findSlavesFrame.rtr = true;
    for (int j = 0; j < maxSlavesInLine; j++) {  //Loop for doing the process as many times as maxSlavesInLine, since the slaves are turned on, one by one.
      int overflow = 0;
      findMessageReceived = false;
      CANMessage findSlavesAnswer;  //No initialization needed, since the message will be read from the slaves
      for (int m = 0; m < maxSlavesInLine; m++) {
        const bool okfindSlavesFrame = ACAN_ESP32::can.tryToSend(findSlavesFrame);  //Sent remote frame to the BUS in order to start the find process
        Serial.println("CAN message was sent");                     //Info
        delay(30);
      }
      while (!findMessageReceived) {                      //Wait for an answer
        if (ACAN_ESP32::can.receive(findSlavesAnswer)) {  //If got reponse from the slave then process the information
          int tempArray[maxSlavesInLine] = { 0 };         //Initialize array to store the current values of the current row i-CANBUSlines
          for (int k = 0; k < maxSlavesInLine; k++) {
            tempArray[k] = potsMatrix[i][k];  //Store the current row i-CANBUSlines in the temp array.
          }
          nextSlaveFlag = valueInArray(findSlavesAnswer.id >> 8, tempArray, maxSlavesInLine);  //Check if the id of the received message is contained int the temp array or not
          if (!nextSlaveFlag) {                                                                //If the id of the received message is new, then store the id in the potsMatrix array
            potsMatrix[i][j] = findSlavesAnswer.id >> 8;                                       //Store the answer in a 2D Matrix for mapping the Pflanzgefäße
            Serial.println(potsMatrix[i][j]);                                                  //Info
          } else {
            potsMatrix[i][j] = 11;  //error address for the slaves
          }
          findMessageReceived = true;
        } else if (overflow > overflowTimer) {
          findMessageReceived = true;
          potsMatrix[i][j] = 11;  //error address for the slaves
        }
        overflow += 1;
        delay(1);
      }
    }
    AusgangEinschalten(ADDRESS_PCAL6408A, 0xff);  //Shut all channels down after the loop.
  }
  for (int m = 0; m < CANBUSlines; m++) {  //Loop for writing the matrix configuration in the ESP32 EEPROM
    char miniBuffer[2];
    char buffer[21];
    for (int n = 0; n < maxSlavesInLine; n++) {       //Loop for slicing the pots Matrix and store the result in the EEPROM
      sprintf(miniBuffer, "%02d", potsMatrix[m][n]);  //Der potsMatrix[i][j] muss nach dem Vorgang in den EEPROM gespeichert werden
      buffer[n * 2] = miniBuffer[0];
      buffer[n * 2 + 1] = miniBuffer[1];
    }
    Serial.println(buffer);
    unsigned long long bufferint = atoll(buffer);
    //CANBUSValues[m] = bufferint;                       //Store the buffer in the matrix
    preferences.begin(CANBUSdata, false);              //init preference to overwrite EEPROM values with the new matrix
    preferences.putULong64(CANBUSKeys[m], bufferint);  // Write the parameter with the following value.
    preferences.end();
  }
}
//Method for reading the matrix configuration from the EEPROM memory and storing it in the potsMatrix[][]
void readSlavesMatrix() {
  for (int m = 0; m < CANBUSlines; m++) {                                 //Loop for starting the 4 different channels
    preferences.begin(CANBUSdata, false);                                 //init preference to overwrite EEPROM values with the new matrix
    unsigned long long value = preferences.getULong64(CANBUSKeys[m], 0);  // Write the parameter with the following value.
    preferences.end();
    Serial.println(value);  //Info
    //CANBUSValues[m] = value;  //Store the buffer in the CANBUS EEPROM variable
    char buffer[21];
    sprintf(buffer, "%llu", value);  //Get the readed value from the EEPROM into string to divide it and store it in the matrix.
    for (int n = 0; n < maxSlavesInLine; n++) {
      char minibuffer[2];
      sprintf(minibuffer, "%c%c", buffer[n * 2], buffer[n * 2 + 1]);  //Get the readed value from the EEPROM into string to divide it and store it in the matrix.
      potsMatrix[m][n] = atoi(minibuffer);                            //Convert the string to int and store it in the matrix.
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
  Serial.println(numberCANBUS);                //Info
  switch (numberCANBUS) {                      //Accordung to numberCANBUS switch the correct CANBUS line.
    case 1:                                    //CANBUS1 configuration pins for the different channels
      Serial.println("Turning on channel 1");  //Info
      settings.mRxPin = GPIO_NUM_25;           // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_26;           // Optional, default Tx pin is GPIO_NUM_5
      break;
    case 2:                                    //CANBUS2
      Serial.println("Turning on channel 2");  //Info
      settings.mRxPin = GPIO_NUM_27;           // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_14;           // Optional, default Tx pin is GPIO_NUM_5
      break;
    case 3:                                    //CANBUS3
      Serial.println("Turning on channel 3");  //Info
      settings.mRxPin = GPIO_NUM_12;           // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_13;           // Optional, default Tx pin is GPIO_NUM_5
      break;
    case 4:                                    //CANBUS4
      Serial.println("Turning on channel 4");  //Info
      settings.mRxPin = GPIO_NUM_15;           // Optional, default Rx pin is GPIO_NUM_4
      settings.mTxPin = GPIO_NUM_4;            // Optional, default Tx pin is GPIO_NUM_5
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
      return j + 1;  //Return the column + 1 since it starts in 0
    }
  }
  return 99;
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
    return true;
  } else {  //If no data received, set the weather station flag to error.
    weatherStationHumidity = 0;
    weatherStationTemperature = 0;
    weatherStationLux = 0;
    return false;
  }
}