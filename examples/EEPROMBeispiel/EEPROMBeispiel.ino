/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           EEPROM write/read values test.
%   Description:    Script gets a timestamp from a ntp server if internet
%					connection is available. In case it is not, then it stores
%					the last available timestamp in the EEPROM in order to
%					remember this time for next powering on. The EEPROM stored
%					value is read while booting. In case the NTP connection is
%					not possible, then this value will be used as reference
%					for the system and this will be used date until the ntp
%					server is available. This also means that this timestamp
%					will be overwritten in the EEPROM in order to keep a
%					chronological order.
%					This Program is just a basic test script for testing the
&					sensor behaviour, this approach should be implemented
&					in a main program and/or another platform
%   Date:           06/12/2023      
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
%	V01:			EEPROM Read/write value test
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

//Included libraries
#include <time.h>			//Library for the time functions
#include <TimeLib.h>
#include <WiFi.h>			//Library for the WiFi Modul
#include <Preferences.h>	//Library for storing the variables in the EEPROM

// Replace with the current network credentials
const char* ssid = "nordisch-Box-1";
const char* password = "87654321";
// NTP server to request epoch time
const char* ntpServer = "pool.ntp.org";
const char* spaceEEPROM = "timestamp";
const char* paramEEPROM = "stamp";

Preferences preferences;

void setup()
{
  Serial.begin(115200);
  initWiFi(); // Init WiFi
  configTime(0, 0, ntpServer); //Start NTP server connection for the UNIX TIME
}

void loop()
{
  
  Serial.println("Checking connection"); 
  checkTimeSet();
  long stamp = readEEPROMTimestamp();
  Serial.println(stamp);
  checkTimeSet();
  delay (10000);
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
      Serial.println("Not connected, Trying to connect");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      delay(2000);
    }
    else{
      //WiFi was already connected
    }
    //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
    }

    void checkTimeSet() {
    //Try to connect if the WL_CONNECTED is false
    if(WiFi.status() != WL_CONNECTED){
      Serial.println("Not set");
      long timestamp = readEEPROMTimestamp();
      Serial.println(timestamp);
      Serial.println("Before setting the time");
      char buffer[26];
      sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", year(), month(), day(),hour(), minute(), second());
      Serial.println(buffer);
      Serial.println(now());
      setTime(timestamp);
      Serial.println("After setting the time");
      sprintf(buffer, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", year(), month(), day(),hour(), minute(), second());
      Serial.println(buffer);
      Serial.println(now());
    }
    else{
      struct tm timeinfo;
      char timestamp[30];
      getLocalTime(&timeinfo);
      sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02d+00:00", timeinfo.tm_year+1900, timeinfo.tm_mon+1, timeinfo.tm_mday,timeinfo.tm_hour+1, timeinfo.tm_min, timeinfo.tm_sec);
      Serial.println(timestamp);
      setTime(timeinfo.tm_hour+1, timeinfo.tm_min, timeinfo.tm_sec,timeinfo.tm_mday, timeinfo.tm_mon+1, timeinfo.tm_year+1900);
      overwriteEEPROMTimestamp(now());
    }
    //Serial.println(WiFi.localIP()); //Show the IP address of the ESP32 Modul if needed
    }

    long readEEPROMTimestamp() {
      //init preference
      preferences.begin(spaceEEPROM, false);
      //preferences.clear(); // remove all preferences in namespace myfile
      //preferences.remove("timeStampID");// remove varname in the namespace
      long readstamp = preferences.getULong(paramEEPROM, 0); //
      preferences.end();
      return readstamp;
      }

    void overwriteEEPROMTimestamp(long stamp) {
      //init preference
      preferences.begin(spaceEEPROM, false);
      //preferences.clear(); // remove all preferences in namespace myfile
      //preferences.remove("varname");// remove varname in the namespace
      preferences.putULong(paramEEPROM, stamp);
      preferences.end();
      }
