#include <time.h>
#include <TimeLib.h>
#include <WiFi.h>         //Library for the WiFi Modul
#include <Preferences.h>

// Replace with network credentials
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
