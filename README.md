# 3D 打印温度监控系统

## 项目概述

本项目旨在实现一个基于 ESP32 的 3D 打印温度监控系统，通过 DHT 传感器读取环境温度和湿度，并通过 MQTT 协议将数据发送到指定的服务器。

## 功能特性

- 通过 DHT 传感器读取环境温度和湿度。
- 计算体感温度。
- 将温度和湿度数据通过 MQTT 协议发送到指定的服务器。
- 支持配置 WiFi 和 MQTT 连接参数。

## 硬件要求

- ESP32 开发板
- DHT 传感器

## 软件要求

- PlatformIO
- Arduino 框架

## 项目结构

- `include/`：包含项目的头文件。
- `lib/`：包含项目的库文件。
- `src/`：包含项目的源代码文件。
- `test/`：包含项目的测试文件。

## 如何使用

1. 使用 PlatformIO 打开项目。
2. 配置 WiFi 和 MQTT 连接参数。
3. 上传代码到 ESP32 开发板。
4. 监控串口输出，查看温度和湿度数据。

## 配置参数

- `targetSSID`：WiFi 网络的 SSID。
- `targetPassword`：WiFi 网络的密码。
- `mqData.mqtt_broker`：MQTT 服务器的地址。
- `mqData.mqtt_username`：MQTT 服务器的用户名。
- `mqData.mqtt_password`：MQTT 服务器的密码。
- `mqData.mqtt_topic`：MQTT 主题。
- `mqData.mqtt_port`：MQTT 端口。

## 函数说明

- `readDHTData()`：读取 DHT 传感器的温度和湿度数据。
- `readInfoFromEEPROM()`：从 EEPROM 中读取 WiFi 和 MQTT 连接参数。
- `saveWiFiInfoToEEPROM()`：将 WiFi 和 MQTT 连接参数保存到 EEPROM 中。
