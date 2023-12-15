/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           Chirp soil moisture sensor test
%   Description:    Program to test the chirp sensor, the sensor gives a value
%                   of 280 when placed in a cup without water and around 530
%                   when the cup is full of water, the values look stable,
%                   the problem is that the sensor it self does not look stable,
%                   it is not supossed to chirp when beeing in sensor mode, but
%                   it does it anyway. I will test other sensors and then
%                   come back to this one.
%					          This Program is just a basic test script for testing the
&					          sensor behaviour, this approach should be implemented
&					          in a main program and/or another platform
%   Date:           07/12/2023      
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
% 	V01:			      Test the chirp sensor after arrival for the first time
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//Needed libraries
#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void writeI2CRegister8bit(int addr, int value) {
  Wire.beginTransmission(addr);
  Wire.write(value);
  Wire.endTransmission();
}

unsigned int readI2CRegister16bit(int addr, int reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  delay(300);
  Wire.requestFrom(addr, 2);
  unsigned int t = Wire.read() << 8;
  t = t | Wire.read();
  return t;
}

void loop() {
  Serial.print(readI2CRegister16bit(0x20, 0)); //read capacitance register
  writeI2CRegister8bit(0x20, 3); //request light measurement 
  delay(500);                   //this can take a while
  Serial.print(", ");
  Serial.println(readI2CRegister16bit(0x20, 4)); //read light register
  delay(100);
}