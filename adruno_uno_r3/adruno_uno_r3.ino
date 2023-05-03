#include "Wire.h"
#include "SHT31.h"
#include <SoftwareSerial.h>
SoftwareSerial espSerial(5, 6);
//5->tx
//6->rx
String str;
uint32_t start;
uint32_t stop;

SHT31 sht;
// Sensor pins
#define sensorPower 7
#define sensorDat A0
#define sensorMua A1
const int rain_pin = 3;
void setup() {
 	Serial.begin(115200);
	pinMode(sensorPower, OUTPUT);
	digitalWrite(sensorPower, LOW);

  ///SHT
  Wire.begin();

  sht.begin(0x44);    //Sensor I2C Address

  Wire.setClock(100000);
  uint16_t stat = sht.readStatus();
  Serial.print(stat, HEX);
  Serial.println();


  /////
  espSerial.begin(115200);
  delay(500);

}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  //Serial.print(analogRead(sensorMua));
  //Serial.print("  ");
  int mua = digitalRead(rain_pin);
  //Serial.println(mua);
  //Serial.print("độ ẩm đất: ");
  int doamdat= readSensor();
	//Serial.println(doamdat);
  ////
  sht.read();
  //Serial.print("Temperature:");
  float Temperature = sht.getTemperature();
  //Serial.print(Temperature, 1);
  //Serial.print("\t");
  //Serial.print("Humidity:");
  float Humidity = sht.getHumidity();
  //Serial.println(Humidity, 1);
  delay(1000);
  
  ////////
  String out = "#" + String(mua) + "|" +String(doamdat)+ "|" + String(Temperature) +"|" +String(Humidity)+ "@";
  Serial.print(out);                                                                                                                                                                                                                                                                                                                                                         
  delay(1000);
  ////////
}
int readSensor() {
	digitalWrite(sensorPower, HIGH);	// Turn the sensor ON
	delay(10);							// Allow power to settle
	int val = analogRead(sensorDat);	// Read the analog value form sensor
	digitalWrite(sensorPower, LOW);		// Turn the sensor OFF
  int percent =100- map(val, 0, 1023, 0, 100);
	return percent;							// Return analog moisture value
}