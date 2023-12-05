// Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
const int TempPin = 35;
const int HumidPin = 34;

// variable for storing the potentiometer value
float TempValue = 0;
float HumidValue = 0;
float test = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
}

void loop() {
  // Reading potentiometer value
  //3.3/4095 - Conversion to volts of the analog input
  //+0.09 - Offset for the ESP32 Kennlinie - Here should be implemented something else, since it is not linear for the whole range
  //-0.5*100 - SMT50 Sensor voltage to °C conversion
  TempValue = (((analogRead(TempPin)*(3.3/4095))+0.09)-0.5)*100;
  //3.3/4095 - Conversion to volts of the analog input
  //+0.0 - Offset for the ESP32 Kennlinie - Here should be implemented something else, since it is not linear for the whole range
  //Offset set to 0, it worked this way, I need to check it with a better multimeter again, to see the values from the sensor, it should not give mroe than 3 V
  //50/3 - SMT50 Sensor voltage to °C conversion
  HumidValue = ((analogRead(HumidPin)*(3.3/4095))+0.0)*(50/3);
  test = min(float(50),HumidValue);
  Serial.print("Temperatur: ");
  Serial.print(TempValue);
  Serial.print("°C, Humidity: ");
  Serial.print(test);
  Serial.println("%");
  delay(1000*1);
  TempValue = 0;
  HumidValue = 0;
  delay(1000*60*5);
}
