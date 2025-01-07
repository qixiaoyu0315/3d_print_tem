#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>
#include <WebServer.h>

#include "DHT.h"

// 定义用于擦除信息的引脚
#define EEPROM_SIZE 100
#define ERASE_PIN 5

// mqtt服务相关
const char *mqttServer = "192.168.6.248";
const int mqttPort = 1883;
const char *mqttUser = "esp32s3";
const char *mqttPassword = "123456";
const char *topic = "testtopic/#";

// // wifi 定义
// const char *ssid = "MTK_CHEETAH_AP_2.4G";
// const char *password = "5680578abc..";

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
// keep 温度
boolean temp_keep = false;

const char *ssid_AP = "MyESP32S3AP";
const char *password_AP = "12345678";
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);

// 存储用户输入的目标 WiFi 的 SSID 和密码
String targetSSID;
String targetPassword;

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>WiFi Configuration</title>
    <!-- 引入 Bootstrap CSS -->
    <link
      rel="stylesheet"
      href="https://maxcdn.bootstrapcdn.com/bootstrap/4.5.2/css/bootstrap.min.css"
    />
  </head>
  <body>
    <div class="container mt-5">
      <div class="card">
        <div class="card-header">
          <h1>WiFi Configuration</h1>
        </div>
        <div class="card-body">
          <form method="post" action="/config">
            <div class="form-group">
              <label for="ssid">WiFi SSID:</label>
              <input
                type="text"
                class="form-control"
                id="ssid"
                name="ssid"
                required
              />
            </div>
            <div class="form-group">
              <label for="password">WiFi Password:</label>
              <input
                type="password"
                class="form-control"
                id="password"
                name="password"
                required
              />
            </div>
            <button type="submit" class="btn btn-primary">Submit</button>
          </form>
        </div>
      </div>
    </div>
    <!-- 引入 Bootstrap JS 和 jQuery (可选) -->
    <script src="https://ajax.googleapis.com/ajax/libs/jquery/3.5.1/jquery.min.js"></script>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.16.0/umd/popper.min.js"></script>
    <script src="https://maxcdn.bootstrapcdn.com/bootstrap/4.5.2/js/bootstrap.min.js"></script>
  </body>
</html>
)rawliteral";

// 从 EEPROM 读取 WiFi 信息
void readWiFiInfoFromEEPROM()
{
  EEPROM.begin(EEPROM_SIZE);
  int ssidLength = EEPROM.read(0);
  int passLength = EEPROM.read(1);
  if (ssidLength > 0 && passLength > 0)
  {
    char ssid[ssidLength + 1];
    char pass[passLength + 1];
    for (int i = 0; i < ssidLength; i++)
    {
      ssid[i] = EEPROM.read(i + 2);
    }
    ssid[ssidLength] = '\0';
    for (int i = 0; i < passLength; i++)
    {
      pass[i] = EEPROM.read(i + 2 + ssidLength);
    }
    pass[passLength] = '\0';
    targetSSID = String(ssid);
    targetPassword = String(pass);
  }
  EEPROM.end();
}

// 将 WiFi 信息保存到 EEPROM
void saveWiFiInfoToEEPROM(String ssid, String password)
{
  EEPROM.begin(EEPROM_SIZE);
  int ssidLength = ssid.length();
  int passLength = password.length();
  EEPROM.write(0, ssidLength);
  EEPROM.write(1, passLength);
  for (int i = 0; i < ssidLength; i++)
  {
    EEPROM.write(i + 2, ssid.charAt(i));
  }
  for (int i = 0; i < passLength; i++)
  {
    EEPROM.write(i + 2 + ssidLength, password.charAt(i));
  }
  EEPROM.commit();
  EEPROM.end();
}

// 擦除 EEPROM 中的 WiFi 信息
void eraseWiFiInfoFromEEPROM()
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
}

// 处理根页面，显示 WiFi 配置表单
void handleRoot()
{
  server.send(200, "text/html", index_html);
}

// 连接到用户输入的 WiFi
void connectToWiFi()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(targetSSID.c_str(), targetPassword.c_str());
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

// 处理用户提交的 WiFi 配置信息
void handleConfig()
{
  if (server.method() == HTTP_POST)
  {
    targetSSID = server.arg("ssid");
    targetPassword = server.arg("password");
    Serial.print("Received SSID: ");
    Serial.println(targetSSID);
    Serial.print("Received Password: ");
    Serial.println(targetPassword);
    server.send(200, "text/plain", "Configuration received. Connecting to WiFi...");
    // 保存 WiFi 信息到 EEPROM
    saveWiFiInfoToEEPROM(targetSSID, targetPassword);
    // 尝试连接到目标 WiFi
    connectToWiFi();
  }
  else
  {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

// 检查是否需要擦除信息
void checkErasePin()
{
  if (digitalRead(ERASE_PIN) == HIGH)
  {
    Serial.println("Erasing WiFi information...");
    eraseWiFiInfoFromEEPROM();
    // 等待一段时间，防止误触发
    delay(1000);
  }
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
/**
 * 读取DHT传感器数据的函数
 * 该函数负责读取DHT传感器的湿度、温度和体感温度，并将这些数据存储在全局变量中。
 * 如果读取失败，将通过串口输出错误信息。
 */
void readDHTData()
{
  // 打印DHT传感器的型号
  Serial.println(F("DHT33"));

  // 读取湿度数据并存储在humidity变量中
  humidity = dht.readHumidity();
  // 读取温度数据并存储在temperature_celsius变量中
  temperature_celsius = dht.readTemperature();
  // 检查湿度或温度数据是否无效
  if (isnan(humidity) || isnan(temperature_celsius))
  {
    // 如果数据无效，打印错误信息并返回
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  // 计算体感温度并存储在temperature_heat变量中
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

    if (floatValue >= temp_high)
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
    else
    {
      Serial.println("OK");
      client.publish("testtopic/msg", "KEEP");
    }
  }
  else if (strcmp(topic, "testtopic/high") == 0)
  {
    char high_str[10];
    strncpy(high_str, (const char *)payload, length);
    high_str[length] = '\0';

    float high_t = atof(high_str);
    if (high_t >= 0.0 && high_t <= 60.0)
    {
      temp_high = high_t;
    }
  }
  else if (strcmp(topic, "testtopic/low") == 0)
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
void setup()
{
  pinMode(controlPin, OUTPUT);
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  pinMode(ERASE_PIN, INPUT_PULLUP); // 设置擦除引脚为输入上拉模式

  Serial.println("ESP32S3 WiFi 配网示例");

  // 尝试读取 EEPROM 中的 WiFi 信息
  readWiFiInfoFromEEPROM();

  // 检查是否需要擦除信息
  checkErasePin();

  if (targetSSID.length() > 0 && targetPassword.length() > 0)
  {
    Serial.println("尝试从 EEPROM 读取 WiFi 信息并连接...");
    connectToWiFi();
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

  dht.begin();
  // setup_wifi();
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
}

void loop()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    if (!client.connected())
    {
      reconnect();
    }
    client.loop();
    // 使用millis()函数来判断是否到达读取传感器数据的时间间隔
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
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
  else
  {
    if (WiFi.getMode() == WIFI_AP)
    {
      server.handleClient();
    }
  }
  // 循环检查擦除引脚状态
  checkErasePin();
}