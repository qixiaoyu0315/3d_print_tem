#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include "DHT.h"

// 定义用于擦除信息的引脚
#define ERASE_PIN 5
#define EEPROM_SIZE 100
// 继电器引脚
#define controlPin 2
// 温湿度读取引脚
#define DHTPIN 1
// 温湿度传感器型号
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
struct TemperatureData
{
  // 湿度
  float humidity;
  // 温度
  float temperature_celsius;
  // 体感温度
  float temperature_heat;
  // 低温开启
  float temp_low = 30.0;
  // 高温关闭
  float temp_high = 45.0;
  // keep 温度
  boolean temp_keep = false;
};
TemperatureData tempData;

// mqtt服务相关信息
struct MqttMsgData
{
  int mqtt_port;
  String mqtt_broker;
  String mqtt_username;
  String mqtt_password;
  String mqtt_topic;
  String topic_sub;
  String topic_temp;
  String topic_msg;
  String topic_high;
  String topic_low;
  String topic_hum;
  String topic_heat;
};
MqttMsgData mqData;

String targetSSID;
String targetPassword;

WiFiClientSecure espClient;
PubSubClient client(espClient);

const char *ssid_AP = "MyESPAP";
const char *password_AP = "12345678";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

unsigned long previousMillis = 0;
const long interval = 10000;

// 从 EEPROM 读取 WiFi 和 MQTT 信息
void readInfoFromEEPROM()
{
  EEPROM.begin(EEPROM_SIZE);
  int ssidLength = EEPROM.read(0);
  int passLength = EEPROM.read(1);
  int mqttBrokerLength = EEPROM.read(2);
  int mqttUserLength = EEPROM.read(3);
  int mqttPassLength = EEPROM.read(4);
  int mqttTopicLength = EEPROM.read(5);
  int mqttPort = (EEPROM.read(6) << 8) | EEPROM.read(7);

  if (ssidLength > 0 && passLength > 0 && mqttBrokerLength > 0 && mqttUserLength > 0 && mqttPassLength > 0 && mqttTopicLength > 0)
  {
    char ssid[ssidLength + 1];
    char pass[passLength + 1];
    char mqtt_broker[mqttBrokerLength + 1];
    char mqtt_username[mqttUserLength + 1];
    char mqtt_password[mqttPassLength + 1];
    char mqtt_topic[mqttTopicLength + 1];

    for (int i = 0; i < ssidLength; i++)
    {
      ssid[i] = EEPROM.read(i + 8);
    }
    ssid[ssidLength] = '\0';

    for (int i = 0; i < passLength; i++)
    {
      pass[i] = EEPROM.read(i + 8 + ssidLength);
    }
    pass[passLength] = '\0';

    for (int i = 0; i < mqttBrokerLength; i++)
    {
      mqtt_broker[i] = EEPROM.read(i + 8 + ssidLength + passLength);
    }
    mqtt_broker[mqttBrokerLength] = '\0';

    for (int i = 0; i < mqttUserLength; i++)
    {
      mqtt_username[i] = EEPROM.read(i + 8 + ssidLength + passLength + mqttBrokerLength);
    }
    mqtt_username[mqttUserLength] = '\0';

    for (int i = 0; i < mqttPassLength; i++)
    {
      mqtt_password[i] = EEPROM.read(i + 8 + ssidLength + passLength + mqttBrokerLength + mqttUserLength);
    }
    mqtt_password[mqttPassLength] = '\0';

    for (int i = 0; i < mqttTopicLength; i++)
    {
      mqtt_topic[i] = EEPROM.read(i + 8 + ssidLength + passLength + mqttBrokerLength + mqttUserLength + mqttPassLength);
    }
    mqtt_topic[mqttTopicLength] = '\0';

    targetSSID = String(ssid);
    targetPassword = String(pass);
    mqData.mqtt_broker = String(mqtt_broker);
    mqData.mqtt_username = String(mqtt_username);
    mqData.mqtt_password = String(mqtt_password);
    mqData.mqtt_topic = String(mqtt_topic);
    mqData.mqtt_port = mqttPort;

    mqData.topic_sub = mqData.mqtt_topic + "/#";
    mqData.topic_temp = mqData.mqtt_topic + "/temp";
    mqData.topic_msg = mqData.mqtt_topic + "/msg";
    mqData.topic_high = mqData.mqtt_topic + "/high";
    mqData.topic_low = mqData.mqtt_topic + "/low";
    mqData.topic_hum = mqData.mqtt_topic + "/hum";
    mqData.topic_heat = mqData.mqtt_topic + "/heat";
  }
  EEPROM.end();
}

// 将 WiFi 和 MQTT 信息保存到 EEPROM
void saveInfoToEEPROM(String ssid, String password, String mqtt_broker, String mqtt_username, String mqtt_password, String mqtt_topic, int mqtt_port)
{
  EEPROM.begin(EEPROM_SIZE);
  int ssidLength = ssid.length();
  int passLength = password.length();
  int mqttBrokerLength = mqtt_broker.length();
  int mqttUserLength = mqtt_username.length();
  int mqttPassLength = mqtt_password.length();
  int mqttTopicLength = mqtt_topic.length();

  EEPROM.write(0, ssidLength);
  EEPROM.write(1, passLength);
  EEPROM.write(2, mqttBrokerLength);
  EEPROM.write(3, mqttUserLength);
  EEPROM.write(4, mqttPassLength);
  EEPROM.write(5, mqttTopicLength);
  EEPROM.write(6, mqtt_port >> 8);   // 存储高字节
  EEPROM.write(7, mqtt_port & 0xFF); // 存储低字节

  for (int i = 0; i < ssidLength; i++)
  {
    EEPROM.write(i + 8, ssid.charAt(i));
  }
  for (int i = 0; i < passLength; i++)
  {
    EEPROM.write(i + 8 + ssidLength, password.charAt(i));
  }
  for (int i = 0; i < mqttBrokerLength; i++)
  {
    EEPROM.write(i + 8 + ssidLength + passLength, mqtt_broker.charAt(i));
  }
  for (int i = 0; i < mqttUserLength; i++)
  {
    EEPROM.write(i + 8 + ssidLength + passLength + mqttBrokerLength, mqtt_username.charAt(i));
  }
  for (int i = 0; i < mqttPassLength; i++)
  {
    EEPROM.write(i + 8 + ssidLength + passLength + mqttBrokerLength + mqttUserLength, mqtt_password.charAt(i));
  }
  for (int i = 0; i < mqttTopicLength; i++)
  {
    EEPROM.write(i + 8 + ssidLength + passLength + mqttBrokerLength + mqttUserLength + mqttPassLength, mqtt_topic.charAt(i));
  }

  EEPROM.commit();
  EEPROM.end();
}

// 擦除 EEPROM 中的 WiFi 和 MQTT 信息
void eraseInfoFromEEPROM()
{
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++)
  {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  EEPROM.end();
  targetSSID = "";
  targetPassword = "";
  mqData.mqtt_broker = "";
  mqData.mqtt_username = "";
  mqData.mqtt_password = "";
  mqData.mqtt_topic = "";
  mqData.mqtt_port = 0;
}

// 处理根页面，显示 WiFi 配置表单和 MQTT 配置表单
void handleRoot()
{
  String html = "<html><body><h1>WiFi and MQTT Config</h1>"
                "<form method='post' action='/config'>"
                "WiFi SSID: <input type='text' name='ssid'><br>"
                "WiFi Password: <input type='password' name='password'><br>"
                "MQTT Broker: <input type='text' name='mqtt_broker'><br>"
                "MQTT Username: <input type='text' name='mqtt_username'><br>"
                "MQTT Password: <input type='password' name='mqtt_password'><br>"
                "MQTT Port: <input type='number' name='mqtt_port'><br>" // 添加 MQTT 端口输入框
                "MQTT Topic: <input type='text' name='mqtt_topic'><br>"
                "<input type='submit' value='Submit'></form></body></html>";
  server.send(200, "text/html", html);
}

// 连接到用户输入的 WiFi
void connectToWiFi()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(targetSSID.c_str(), targetPassword.c_str());
  // esp32c3 super mini 当网络连接不稳定时需要添加此代码
  // WiFi.setTxPower(WIFI_POWER_8_5dBm);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20)
  {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Connected to the target WiFi network");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("Failed to connect to the target WiFi network");
  }
}

// 检查是否需要擦除信息
void checkErasePin()
{
  Serial.println(digitalRead(ERASE_PIN));
  if (digitalRead(ERASE_PIN) == HIGH)
  {
    Serial.println("Erasing WiFi information...");
    eraseInfoFromEEPROM();
    // 等待一段时间，防止误触发
    delay(1000);
  }
}

// 连接mqtt服务
void reconnectToMqtt()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // 第一个参数为id 默认保持一直减少配网填写信息
    if (client.connect(mqData.mqtt_username.c_str(), mqData.mqtt_username.c_str(), mqData.mqtt_password.c_str()))
    {
      Serial.println("connected");
      client.subscribe(mqData.topic_sub.c_str());
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

// 处理用户提交的 WiFi 配置信息和 MQTT 配置信息
void handleConfig()
{
  if (server.method() == HTTP_POST)
  {
    targetSSID = server.arg("ssid");
    targetPassword = server.arg("password");
    mqData.mqtt_broker = server.arg("mqtt_broker");
    mqData.mqtt_username = server.arg("mqtt_username");
    mqData.mqtt_password = server.arg("mqtt_password");
    mqData.mqtt_topic = server.arg("mqtt_topic");
    mqData.mqtt_port = server.arg("mqtt_port").toInt();

    server.send(200, "text/plain", "Configuration received. Connecting to WiFi and MQTT...");
    saveInfoToEEPROM(targetSSID, targetPassword, mqData.mqtt_broker, mqData.mqtt_username, mqData.mqtt_password, mqData.mqtt_topic, mqData.mqtt_port);
    readInfoFromEEPROM();
    // 尝试连接到目标 WiFi 和 MQTT
    connectToWiFi();
    reconnectToMqtt();
  }
  else
  {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

// 读取DHT传感器数据的函数
void readDHTData()
{
  tempData.humidity = dht.readHumidity();
  tempData.temperature_celsius = dht.readTemperature();
  if (isnan(tempData.humidity) || isnan(tempData.temperature_celsius))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  tempData.temperature_heat = dht.computeHeatIndex(tempData.temperature_celsius, tempData.humidity, false);
}

// 发布DHT传感器数据的函数
void publishSensorData()
{
  readDHTData();

  char tempStr[8];
  dtostrf(tempData.temperature_celsius, 4, 2, tempStr);
  client.publish(mqData.topic_temp.c_str(), tempStr);

  char humStr[8];
  dtostrf(tempData.humidity, 4, 2, humStr);
  client.publish(mqData.topic_hum.c_str(), humStr);

  char heatStr[8];
  dtostrf(tempData.temperature_heat, 4, 2, heatStr);
  client.publish(mqData.topic_heat.c_str(), heatStr);

  char highStr[8];
  dtostrf(tempData.temp_high, 4, 2, highStr);
  client.publish(mqData.topic_high.c_str(), highStr);

  char lowStr[8];
  dtostrf(tempData.temp_low, 4, 2, lowStr);
  client.publish(mqData.topic_low.c_str(), lowStr);
}

// 处理MQTT消息的回调函数
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
  if (strcmp(topic, mqData.topic_temp.c_str()) == 0)
  {
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    // 尝试将接收到的消息内容转换为float类型
    float floatValue;
    char *endptr;
    floatValue = strtof(message, &endptr);
    if (*endptr == '\0')
    {
      Serial.print("接收到的值转换为float后: ");
      Serial.println(floatValue);
    }
    else
    {
      Serial.println("接收到的消息无法转换为float类型");
    }

    if (floatValue >= tempData.temp_high)
    {
      digitalWrite(controlPin, LOW);
      Serial.println("关闭加热");
      client.publish(mqData.topic_msg.c_str(), "OFF");
      tempData.temp_keep = true;
    }
    else if (floatValue <= tempData.temp_low || !tempData.temp_keep)
    {
      digitalWrite(controlPin, HIGH);
      Serial.println("开启加热");
      client.publish(mqData.topic_msg.c_str(), "ON");
      tempData.temp_keep == false;
    }
    else
    {
      Serial.println("OK");
      client.publish(mqData.topic_msg.c_str(), "KEEP");
    }
  }
  else if (strcmp(topic, mqData.topic_high.c_str()) == 0)
  {
    char high_str[10];
    strncpy(high_str, (const char *)payload, length);
    high_str[length] = '\0';

    float high_t = atof(high_str);
    if (high_t >= 0.0 && high_t <= 60.0)
    {
      tempData.temp_high = high_t;
    }
  }
  else if (strcmp(topic, mqData.topic_low.c_str()) == 0)
  {
    char low_str[10];
    strncpy(low_str, (const char *)payload, length);
    low_str[length] = '\0';

    float low_t = atof(low_str);
    if (low_t >= 0.0 && low_t <= 60.0)
    {
      tempData.temp_low = low_t;
    }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(controlPin, OUTPUT);
  digitalWrite(controlPin, LOW);
  pinMode(ERASE_PIN, INPUT);
  // // 设置 WiFiClientSecure 为 TLS 模式
  // const char *caCert = "-----BEGIN CERTIFICATE-----\n"
  //                      "输入自己的CA证书"
  //                      "-----END CERTIFICATE-----";
  // espClient.setCACert(caCert);

  // 与上面二选一
  espClient.setCACert(NULL);
  espClient.setInsecure();

  EEPROM.begin(EEPROM_SIZE);
  checkErasePin();
  readInfoFromEEPROM();

  if (targetSSID.length() > 0 && targetPassword.length() > 0)
  {
    client.setServer(mqData.mqtt_broker.c_str(), mqData.mqtt_port);
    Serial.println("EEPROM read WiFi...");
    connectToWiFi();
    reconnectToMqtt();
    client.setCallback(callback);
    dht.begin();
  }
  else
  {
    // 如果没有保存的信息，则开启 SoftAP 模式进行配网
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
    Serial.println("Setting soft-AP... ");
    boolean result = WiFi.softAP(ssid_AP, password_AP);

    if (result)
    {
      Serial.println("SoftAP started successfully!");
      Serial.println(String("SoftAP IP address = ") + WiFi.softAPIP().toString());

      server.on("/", handleRoot);
      server.on("/config", handleConfig);
      server.begin();
      Serial.println("Web server started.");
    }
    else
    {
      Serial.println("Failed to start SoftAP!");
    }
  }
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected())
    {
      reconnectToMqtt();
    }
    client.loop();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      previousMillis = currentMillis;
      publishSensorData();
    }
  }
  else
  {
    if (WiFi.getMode() == WIFI_AP)
    {
      server.handleClient();
    }
  }
}