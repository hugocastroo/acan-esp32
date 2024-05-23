/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           Find sensor adress using I2C protocol
%   Description:    Program used to find the address of different sensors
%					which communication protocol is I2C. In this project it
%					was used for the Hailege light sensor
%					This Program is just a basic test script for testing the
&					sensor behaviour, this approach should be implemented
&					in a main program and/or another platform
%   Date:           06/12/2023
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
%             Date:     30/04/2024
%             V02:      Changing the I2C pins for testing the ZSE Leitterplatte
%             Date:     06/12/2023
%	            V01:			Adress of the sensor was acquired using this script.
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
//Needed libraries
#include <Wire.h> //Library needed for the I2C communication protocol
#define I2C_SDA 33
#define I2C_SCL 32
#define PIN_IOEXPANDER 16 //Change PIN Position if using other PIN
 
void setup() {
  
  //Wire.begin(); //For default PINS - SDA (default is GPIO 21), SCL (default is GPIO 22) 
  Wire.begin(I2C_SDA, I2C_SCL); //For ZSE Test procedure to implement it in the main programm
  Serial.begin(115200);
  Serial.println("\nI2C Scanner");
  pinMode (PIN_IOEXPANDER, OUTPUT);
  digitalWrite (PIN_IOEXPANDER, HIGH);
}
 
void loop() {
  byte error, address;  
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
  delay(5000);          
}
