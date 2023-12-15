/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           Fork moisture sensor test
%   Description:    Program to test the fork sensor, the sensor gives a value
%                   of 4095 when placed in a cup without water and around XXXX
%                   when the cup is full of water, the values look stable, but
%                   it is for sure not linear. The sensor seems not to like
%                   reading the value many times a minute, but for the purpose
%                   of the project it might not be important.
%					          This Program is just a basic test script for testing the
&					          sensor behaviour, this approach should be implemented
&					          in a main program and/or another platform
%   Date:           06/12/2023      
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
%	  V01:			      Testing the sensor after arrival for the first time
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

int sensorPin = 25;
int sensorValue;
int dryAnalogValue = 4095;
int wetAnalogValue = 0;
int DryValuePercent = 0;
int WetValuepercent = 100;
void setup()
{
Serial.begin(115200);
pinMode(13, OUTPUT); // Led Output
}
void loop()
{
sensorValue = analogRead(sensorPin);
Serial.print("Analog Value : ");
Serial.println(sensorValue);
sensorValue = analogRead(sensorPin);
Serial.print("Analog moisture Value : ");
Serial.println(sensorValue);
int moistureValuePercent = map(sensorValue, dryAnalogValue,
wetAnalogValue, DryValuePercent, WetValuepercent);
Serial.print("moisture percent: ");
Serial.print(moistureValuePercent);
Serial.println("%");
if (moistureValuePercent == DryValuePercent)
{
digitalWrite(13, HIGH);
Serial.println("plant is dry !! ");
}
else
{
digitalWrite(13, LOW);
}
delay(1000*60*5);
}