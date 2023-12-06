/* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	This program is part of the research project "Automationssystem für 
%	Fassadenbegrünung zur Optimierung der Energieeffizienz von Gebäuden",
%	from the Hochschule Flensbug.
%   Name:           AZ Delivery Hygrometer program test
%   Description:    Script for testing the Hygrometer sensor from AZ Delivery.
%					This Program is just a basic test script for testing the
%					sensor behaviour, this approach should be implemented
%					in a main program and/or another platform
%   Date:           06/12/2023      
%   Programmer:     Hugo Valentin Castro Saenz
%   History:
%	V01:			Sensor was tested for the first time after arrival. 
%					Since there is any provided sensor curve, the analog input
%					was not converted to a percentage or any other value
%					that could simplify the acquired value. Sensor sensed a
%					value of 4095, when no moisture was present at all.
%					After incrementing the moisture around the sensor,
%					glass with water. The sensor value started decrementing
%					up to aprox. 1500-2000, but not less. The curve of the sensor
%					seems not the best one. Further test will be done, in 
%					case that the measurements do not improve, there is no sense
%					in making a own sensor curve for the implementation.
%	
%  
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

//Globals for the program
const int HygrometerPin = 32; //Using the Analog ADC1_CH4
float HygrometerValue = 0;

void setup() {
// initalize serial communicaton at 115200 bits per second:
Serial.begin(115200);
}
// the loop routine runs over and over again forever:
void loop() {
// read the input on HygrometerPin:
HygrometerValue = analogRead(HygrometerPin);
// print out the readed value
Serial.print("Analog input value: ");
Serial.println(HygrometerValue);

delay(1000); // delay in between reads for stability
HygrometerValue = 0;
delay(100);
}