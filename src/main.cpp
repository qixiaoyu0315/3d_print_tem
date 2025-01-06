#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include "DHT.h"

// wifi 定义
const char *ssid = "MTK_CHEETAH_AP_2.4G";
const char *password = "5680578abc..";

// mqtt服务相关
const char *mqttServer = "192.168.6.248";
const int mqttPort = 1883;
const char *mqttUser = "esp32s3";
const char *mqttPassword = "123456";
const char *topic = "testtopic/#";

unsigned long previousMillis = 0;
const long interval = 10000;

WiFiClient espClient;
PubSubClient client(espClient);
// 继电器引脚
#define controlPin 2

// 温湿度读取引脚
#define DHTPIN 1
// 温湿度传感器型号    
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
// 湿度
float humidity;
// 温度
float temperature_celsius;
// 体感温度
float temperature_heat;
// 低温开启
float temp_low = 30.0;
// 高温关闭
float temp_high = 40.0;
//keep 温度
boolean temp_keep = false;

// 连接wifi
void setup_wifi()
{
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

// 连接mqtt服务
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("3dprint", mqttUser, mqttPassword))
    {
      Serial.println("connected");
      client.subscribe(topic);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// 封装的函数，用于读取传感器数据
void readDHTData() {
  Serial.println(F("DHT33"));
    humidity = dht.readHumidity();
    temperature_celsius = dht.readTemperature();

    if (isnan(humidity) || isnan(temperature_celsius)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }
    temperature_heat = dht.computeHeatIndex(temperature_celsius, humidity, false);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // 控制继电器开关
  if (strcmp(topic, "testtopic/temp") == 0)
  {
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    // 尝试将接收到的消息内容转换为float类型
    float floatValue;
    char* endptr;
    floatValue = strtof(message, &endptr);
    if (*endptr == '\0') {
      Serial.print("接收到的值转换为float后: ");
      Serial.println(floatValue);
    } else {
      Serial.println("接收到的消息无法转换为float类型");
    }

    if (floatValue >= temp_high )
    {
      digitalWrite(controlPin, LOW);
      Serial.println("关闭加热");
      client.publish("testtopic/msg", "OFF");
      temp_keep = true;
    }
    else if (floatValue <= temp_low || !temp_keep)
    {
      digitalWrite(controlPin, HIGH);
      Serial.println("开启加热");
      client.publish("testtopic/msg", "ON");
      temp_keep == false;
    }
    else{
      Serial.println("OK");
      client.publish("testtopic/msg", "KEEP");
    }
  }else if (strcmp(topic, "testtopic/high") == 0)
  {
    char high_str[10];
    strncpy(high_str, (const char *)payload, length);
    high_str[length] = '\0';

    float high_t = atof(high_str);
    if (high_t >= 0.0 && high_t <= 60.0)
    {
      temp_high = high_t;
    }
  }else if (strcmp(topic, "testtopic/low") == 0)
  {
    char low_str[10];
    strncpy(low_str, (const char *)payload, length);
    low_str[length] = '\0';

    float low_t = atof(low_str);
    if (low_t >= 0.0 && low_t <= 60.0)
    {
      temp_low = low_t;
    }
  }
}
void setup() {
  pinMode(controlPin, OUTPUT);
  Serial.begin(115200);
  dht.begin();
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

void loop() {

  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  // 使用millis()函数来判断是否到达读取传感器数据的时间间隔
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      readDHTData();
      // 将温度数据发布到MQTT主题
      char tempStr[8];
      dtostrf(temperature_celsius, 4, 2, tempStr);
      client.publish("testtopic/temp", tempStr);

      char humStr[8];
      dtostrf(humidity, 4, 2, humStr);
      client.publish("testtopic/hum", humStr);

      char heatStr[8];
      dtostrf(temperature_heat, 4, 2, heatStr);
      client.publish("testtopic/heat", heatStr);

      char highStr[8];
      dtostrf(temp_high, 4, 2, highStr);
      client.publish("testtopic/high", highStr);

      char lowStr[8];
      dtostrf(temp_low, 4, 2, heatStr);
      client.publish("testtopic/low", heatStr);
  }
}