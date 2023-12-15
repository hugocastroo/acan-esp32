/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           SMT50 Sensor curve tunning
%   Description:    Script for tunning the SMT50 soil moisture sensor from
%					Trübner producer. More information about the sensor can
%					be found in: https://www.truebner.de/de/smt50.php
%					This Program is just a basic test script for testing the
&					sensor behaviour, this approach should be implemented
&					in a main program and/or another platform
%   Date:           06/12/2023      
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
%	V01:			Sensor was tested for the first time after arrival.
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
// Sensor outputs are connected to GPIO 34 and 35 (Analog ADC1_CH6/CH7) 
const int HumidPin = 34;
const int TempPin = 35;

// variable for storing the temperature and humidity values
float TempValue = 0;
float HumidValue = 0;

void setup() {
	//Start serial monitor for debugging
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  // Reading input values Analog ADC1_CH6/Analog ADC1_CH7
  //3.3/4095 - Conversion to volts of the analog input
  //+0.0 - Offset for the ESP32 Kennlinie - Here should be implemented something else, since it is not linear for the whole range
  //Offset set to 0, it worked this way, I need to check it with a better multimeter again, to see the values from the sensor, it should not give mroe than 3 V
  //50/3 - SMT50 Sensor voltage to °C conversion
  HumidValue = ((analogRead(HumidPin)*(3.3/4095))+0.09)*(50/3);
  //3.3/4095 - Conversion to volts of the analog input
  //+0.09 - Offset for the ESP32 Kennlinie - Here should be implemented something else, since it is not linear for the whole range
  //-0.5*100 - SMT50 Sensor voltage to °C conversion
  TempValue = (((analogRead(TempPin)*(3.3/4095))+0.09)-0.5)*100;
  //Printing the results
  Serial.print("Temperatur: ");
  Serial.print(TempValue);
  Serial.print("°C, Humidity: ");
  Serial.print(HumidValue);
  Serial.println("%");
  delay(100*1);
  //Setting the value to 0, just to discard that the input get no measurement next cycle.
  TempValue = 0;
  HumidValue = 0;
  delay(1000*1*5);
}