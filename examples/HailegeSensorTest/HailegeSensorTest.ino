/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           Hailege sensor light test
%   Description:    Script for testing the Hailege BH1750 BH1750FVI 
%					Digital Light Intensity Sensor.
%					This Program is just a basic test script for testing the
%					sensor behaviour, this approach should be implemented
%					in a main program and/or another platform
%					Sensor information:
%					One Time H-Resolution Mode:
%					Resolution = 1 lux
%					Measurement time (max.) = 180ms
%					Power down after each measurement
%					For further information about the sensor and its working
%					curve, please refer to Digital Light Sensor IC BH1750FVI
%   Date:           06/12/2023      
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
%	V01:			Sensor was tested for the first time after arrival.
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//Needed libraries
#include <Wire.h> //Library for the I2C communication protocol

#define ADDRESS_BH1750FVI 0x23    //Address aquired with the SensorFindIC2Adress.ino script
#define ONE_TIME_H_RESOLUTION_MODE 0x20 //I2C Setting

//Variables
 byte highByte = 0;
 byte lowByte = 0;
 unsigned int sensorOut = 0;
 unsigned int illuminance = 0;

 void setup()
 {
     Wire.begin();
     Serial.begin(115200);
 }

 void loop()
 {
	//Start the I2C communication protocol
	Wire.beginTransmission(ADDRESS_BH1750FVI); //"notify" the matching device
	Wire.write(ONE_TIME_H_RESOLUTION_MODE);     //set operation mode
	Wire.endTransmission();
	delay(180);
	Wire.requestFrom(ADDRESS_BH1750FVI, 2); //ask the ESP32 to read back 2 bytes from the sensor
	highByte = Wire.read();  // get the high byte
	lowByte = Wire.read(); // get the low byte
	//Sensor value conversion, for more information refer to: Digital 16bit Serial Output Type Ambient Light Sensor IC BH1750FVI
	sensorOut = (highByte<<8)|lowByte;
	illuminance = sensorOut/1.2;
	Serial.print(illuminance);
	Serial.println(" lux");
	delay(1000);
 }