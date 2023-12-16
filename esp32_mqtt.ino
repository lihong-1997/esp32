/***********************************************************************
  Adafruit MQTT Library ESP32 Adafruit IO SSL/TLS example

  Use the latest version of the ESP32 Arduino Core:
    https://github.com/espressif/arduino-esp32

  Works great with Adafruit Huzzah32 Feather and Breakout Board:
    https://www.adafruit.com/product/3405
    https://www.adafruit.com/products/4172

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Tony DiCola for Adafruit Industries.
  Modified by Brent Rubell for Adafruit Industries
  MIT license, all text above must be included in any redistribution
 **********************************************************************/
#include <WiFi.h>
#include <Arduino.h>
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <ArduinoJson.h>
//#include <Arduino_FreeRTOS.h>

/************************* WiFi Access Point *********************************/

#define WLAN_SSID "616"
#define WLAN_PASS "10101019"

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "uc8838d8.ala.cn-hangzhou.emqxsl.cn"

// Using port 8883 for MQTTS
#define AIO_SERVERPORT  8883

// Adafruit IO Account Configuration
// (to obtain these values, visit https://io.adafruit.com and click on Active Key)
#define AIO_USERNAME "computer"
#define AIO_KEY      "computer"

/************ Global State (you don't need to change this!) ******************/

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// io.adafruit.com root CA
const char* adafruitio_root_ca = \
      "-----BEGIN CERTIFICATE-----\n"
      "MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
      "QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
      "9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
      "CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
      "nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
      "43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
      "T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
      "gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
      "BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
      "TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
      "DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
      "hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
      "06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
      "PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
      "YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
      "CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
      "-----END CERTIFICATE-----\n"
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEqjCCA5KgAwIBAgIQAnmsRYvBskWr+YBTzSybsTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
      "QTAeFw0xNzExMjcxMjQ2MTBaFw0yNzExMjcxMjQ2MTBaMG4xCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xLTArBgNVBAMTJEVuY3J5cHRpb24gRXZlcnl3aGVyZSBEViBUTFMgQ0EgLSBH\n"
      "MTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALPeP6wkab41dyQh6mKc\n"
      "oHqt3jRIxW5MDvf9QyiOR7VfFwK656es0UFiIb74N9pRntzF1UgYzDGu3ppZVMdo\n"
      "lbxhm6dWS9OK/lFehKNT0OYI9aqk6F+U7cA6jxSC+iDBPXwdF4rs3KRyp3aQn6pj\n"
      "pp1yr7IB6Y4zv72Ee/PlZ/6rK6InC6WpK0nPVOYR7n9iDuPe1E4IxUMBH/T33+3h\n"
      "yuH3dvfgiWUOUkjdpMbyxX+XNle5uEIiyBsi4IvbcTCh8ruifCIi5mDXkZrnMT8n\n"
      "wfYCV6v6kDdXkbgGRLKsR4pucbJtbKqIkUGxuZI2t7pfewKRc5nWecvDBZf3+p1M\n"
      "pA8CAwEAAaOCAU8wggFLMB0GA1UdDgQWBBRVdE+yck/1YLpQ0dfmUVyaAYca1zAf\n"
      "BgNVHSMEGDAWgBQD3lA1VtFMu2bwo+IbG8OXsj3RVTAOBgNVHQ8BAf8EBAMCAYYw\n"
      "HQYDVR0lBBYwFAYIKwYBBQUHAwEGCCsGAQUFBwMCMBIGA1UdEwEB/wQIMAYBAf8C\n"
      "AQAwNAYIKwYBBQUHAQEEKDAmMCQGCCsGAQUFBzABhhhodHRwOi8vb2NzcC5kaWdp\n"
      "Y2VydC5jb20wQgYDVR0fBDswOTA3oDWgM4YxaHR0cDovL2NybDMuZGlnaWNlcnQu\n"
      "Y29tL0RpZ2lDZXJ0R2xvYmFsUm9vdENBLmNybDBMBgNVHSAERTBDMDcGCWCGSAGG\n"
      "/WwBAjAqMCgGCCsGAQUFBwIBFhxodHRwczovL3d3dy5kaWdpY2VydC5jb20vQ1BT\n"
      "MAgGBmeBDAECATANBgkqhkiG9w0BAQsFAAOCAQEAK3Gp6/aGq7aBZsxf/oQ+TD/B\n"
      "SwW3AU4ETK+GQf2kFzYZkby5SFrHdPomunx2HBzViUchGoofGgg7gHW0W3MlQAXW\n"
      "M0r5LUvStcr82QDWYNPaUy4taCQmyaJ+VB+6wxHstSigOlSNF2a6vg4rgexixeiV\n"
      "4YSB03Yqp2t3TeZHM9ESfkus74nQyW7pRGezj+TC44xCagCQQOzzNmzEAP2SnCrJ\n"
      "sNE2DpRVMnL8J6xBRdjmOsC3N6cQuKuRXbzByVBjCqAA8t1L0I+9wXJerLPyErjy\n"
      "rMKWaBFLmfK/AHNF4ZihwPGOc7w6UHczBZXH5RFzJNnww+WnKuTPI0HfnVH8lg==\n"
      "-----END CERTIFICATE-----\n";

/****************************** Feeds ***************************************/

// Setup a feed called 'test' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
// Adafruit_MQTT_Publish test = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/test");
Adafruit_MQTT_Subscribe sub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/hardware/info");

/****************************** Led ***************************************/
// 定义LED连接的引脚
const int ledPin = 23;
const int dacPin25 = 25;
const int dacPin26 = 26;

// 全局变量
int memoryLoadValue = 0;
int cpuLoadValue = 0;
int cpuTempValue = 0;
int gpuLoadValue = 0;
int gpuTempValue = 0;

int preMemoryLoadValue = 0;
int preCpuLoadValue = 0;
int preCpuTempValue = 0;
int preGpuLoadValue = 0;
int preGpuTempValue = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 5000;  // 5s间隔
/*************************** Sketch Code ************************************/
// 定义任务句柄
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;

// 定义互斥锁
SemaphoreHandle_t xMutex;
// mqtt
void Task1(void *pvParameters) {
  while (1) {
    MQTT_connect();
    Adafruit_MQTT_Subscribe *subscription;
    while ((subscription = mqtt.readSubscription(1000))) {
      if (subscription == &sub) {
        const char* data = (char *)sub.lastread;
        // 创建 JSON 缓冲区，大小取决于你的数据大小
        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, data);
          const char* memoryLoad = doc["memory"]["memory_load_percent"].as<const char*>();
          const char* cpuLoad = doc["cpu"]["cpu_load_percent"].as<const char*>();
          const char* cpuTemp = doc["cpu"]["cpu_average_temperature"].as<const char*>();
          const char* gpuLoad = doc["gpu"]["gpu_load_percent"].as<const char*>();
          const char* gpuTemp = doc["gpu"]["gpu_core_temperature"].as<const char*>();
          if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
            memoryLoadValue = atoi(memoryLoad);
            cpuLoadValue = atoi(cpuLoad);
            cpuTempValue = atoi(cpuTemp);
            gpuLoadValue = atoi(gpuLoad);
            gpuTempValue = atoi(gpuTemp);

          }
          xSemaphoreGive(xMutex);
          
          int ratio = (int)((float)memoryLoadValue/100.0 * 255);
          analogWrite(dacPin25, ratio);
          Serial.print("Hardware Info:");
          Serial.print(memoryLoadValue);
          Serial.print(",");
          Serial.print(cpuLoad);
          Serial.print(",");
          Serial.print(cpuTemp);
          Serial.print(",");
          Serial.print(gpuLoad);
          Serial.print(",");
          Serial.println(gpuTemp);
      }
    }
  }
}

// 任务2
void Task2(void *pvParameters) {
  while (1) {
  // 获取当前时间
  unsigned long currentMillis = millis();
  // 检查值是否变化
  if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
    if (memoryLoadValue != preMemoryLoadValue ||
        cpuLoadValue != preCpuLoadValue ||
        cpuTempValue != preCpuTempValue ||
        gpuLoadValue != preGpuLoadValue ||
        gpuTempValue != preGpuTempValue) {
        // 如果值变化，更新记录的值和时间
        analogWrite(ledPin, 10);
        preMemoryLoadValue = memoryLoadValue;
        preCpuLoadValue = cpuLoadValue;
        preCpuTempValue = cpuTempValue;
        preGpuLoadValue = gpuLoadValue;
        preGpuTempValue = gpuTempValue;
        // 释放锁
        xSemaphoreGive(xMutex);
        previousMillis = currentMillis;
    } else {
      // 直接释放锁
      xSemaphoreGive(xMutex);
      // 如果值没有变化，并且时间间隔超过5秒，输出信息
      if (currentMillis - previousMillis >= interval) {
        Serial.println("no latest hardware info");
        // 电压表、LED归零
        analogWrite(ledPin, 0);
        analogWrite(dacPin25, 0);
      }
    }
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);

  // Connect to WiFi access point.
  Serial.println(); Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WLAN_SSID);

  delay(1000);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("WiFi connected");

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);
  mqtt.subscribe(&sub);

  // 初始化LED引脚为输出模式
  pinMode(ledPin, OUTPUT);

  pinMode(dacPin25, OUTPUT);
  analogWrite(dacPin25, 0);
  pinMode(dacPin26, OUTPUT);
  analogWrite(dacPin26, 0);

  // 创建互斥锁
  xMutex = xSemaphoreCreateMutex();

  // 创建任务1
  //xTaskCreate(Task1, "Task1", 4096, NULL, 1, &Task1Handle);

  // 创建任务2
  xTaskCreate(Task2, "Task2", 1024, NULL, 0, &Task2Handle);

  // 开启多线程调度器
  //vTaskStartScheduler();
  //xTaskCreatePinnedToCore(Task1, "TaskOne", 4096, NULL, 1, NULL, 0);//TaskOne在 0核心
    //xTaskCreatePinnedToCore(Task2, "TaskTwo", 4096, NULL, 2, NULL, 1);//TaskOne在 1核心
}


void loop() {
  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &sub) {
      analogWrite(ledPin, 10);
      const char* data = (char *)sub.lastread;
      // 创建 JSON 缓冲区，大小取决于你的数据大小
      DynamicJsonDocument doc(512);
      DeserializationError error = deserializeJson(doc, data);
        const char* memoryLoad = doc["memory"]["memory_load_percent"].as<const char*>();
        const char* cpuLoad = doc["cpu"]["cpu_load_percent"].as<const char*>();
        const char* cpuTemp = doc["cpu"]["cpu_average_temperature"].as<const char*>();
        const char* gpuLoad = doc["gpu"]["gpu_load_percent"].as<const char*>();
        const char* gpuTemp = doc["gpu"]["gpu_core_temperature"].as<const char*>();
        if (xSemaphoreTake(xMutex, portMAX_DELAY)) {
          memoryLoadValue = atoi(memoryLoad);
          cpuLoadValue = atoi(cpuLoad);
          cpuTempValue = atoi(cpuTemp);
          gpuLoadValue = atoi(gpuLoad);
          gpuTempValue = atoi(gpuTemp);

        }
        xSemaphoreGive(xMutex);
        
        int ratio = (int)((float)memoryLoadValue/100.0 * 255);
        analogWrite(dacPin25, ratio);
        Serial.print("Hardware Info:");
        Serial.print(memoryLoadValue);
        Serial.print(",");
        Serial.print(cpuLoad);
        Serial.print(",");
        Serial.print(cpuTemp);
        Serial.print(",");
        Serial.print(gpuLoad);
        Serial.print(",");
        Serial.println(gpuTemp);
    }
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
}