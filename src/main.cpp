#include <Arduino.h>
#include "DHT.h"

// 温湿度读取引脚
#define DHTPIN 2
//温湿度传感器型号    
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);


float humidity;
float temperature_celsius;
float temperature_heat;


// 封装的函数，用于读取传感器数据
void readDHTData() {
    humidity = dht.readHumidity();
    temperature_celsius = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature_celsius)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
    
    temperature_heat = dht.computeHeatIndex(temperature_celsius, humidity, false);
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("DHT11"));
  dht.begin();
}


void loop() {
  delay(2000);

  readDHTData();
  Serial.print(F("Humidity: "));
  Serial.print(humidity);
  Serial.print(F("%  Temperature: "));
  Serial.print(temperature_celsius);
  Serial.print(F("°C "));
  Serial.print(F("Heat index: "));
  Serial.print(temperature_heat);
  Serial.println(F("°C "));
}