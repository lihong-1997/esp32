/***********************************************************************
 基于 MQTT 的电脑硬件信息监控
 **********************************************************************/
#include <WiFi.h>
#include <Arduino.h>

#include "mqtt.h"
#include "WiFiClientSecure.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

#include <ArduinoJson.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "WiFiUser.h"

/************************* OLED *********************************/

#define SCREEN_WIDTH 128 // 使用 128×32 OLED 显示屏
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);// 采用 I2C 通信协议

/************************* WiFi Access Point *********************************/

#define WLAN_SSID "616"
#define WLAN_PASS "10101019"
int connectTimeOut_s = 10;    //WiFi连接超时时间，单位秒

/******************************** MQTT  *****************************/

// WiFiFlientSecure for SSL/TLS support
WiFiClientSecure client;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup a feed called '/hardware/info' for subscribe.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
Adafruit_MQTT_Subscribe sub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/hardware/info");

/****************************** PIN ***************************************/
// 定义LED连接的引脚
const int ledPin = 23;
const int sclPin = 22;
const int sdaPin = 21;
const int dacPin25 = 25;
const int dacPin26 = 26;
const int switch1Pin = 12;
const int switch2Pin = 14;
const int resetPin = 13;

// 全局变量
int memoryLoadValue = 0;
int cpuLoadValue = 0;
int cpuTempValue = 0;
int gpuLoadValue = 0;
int gpuTempValue = 0;

int cpuLoadRatio;
int gpuLoadRatio;
int cpuTempRatio;
int gpuTempRatio;

int preMemoryLoadValue = 0;
int preCpuLoadValue = 0;
int preCpuTempValue = 0;
int preGpuLoadValue = 0;
int preGpuTempValue = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 5000;  // 5s间隔

int wifiStatus;
/*************************** Sketch Code ************************************/
// 定义任务句柄
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;

// 定义互斥锁
SemaphoreHandle_t xMutex;
SemaphoreHandle_t mutex_wifi;
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

// 任务2:监视硬件信息是否改变
void Task2(void *pvParameters) {
  while (1) {
    //xSemaphoreTake(mutex_wifi, portMAX_DELAY);
    if (xSemaphoreTake(mutex_wifi, (TickType_t) 5) == pdTRUE) {
      wifiStatus = WiFi.status();
      xSemaphoreGive(mutex_wifi);                 // 释放锁 mutex_wifi

      if (wifiStatus != WL_CONNECTED) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        continue;
      }
      // 连接上wifi
      unsigned long currentMillis = millis();     // 获取当前时间
      if (xSemaphoreTake(xMutex, portMAX_DELAY)) {       // 检查硬件值是否变化
          if (memoryLoadValue != preMemoryLoadValue ||
              cpuLoadValue    != preCpuLoadValue    ||
              cpuTempValue    != preCpuTempValue    ||
              gpuLoadValue    != preGpuLoadValue    ||
              gpuTempValue    != preGpuTempValue) {
              analogWrite(ledPin, 10);// 如果值变化，更新记录的值和时间
              preMemoryLoadValue = memoryLoadValue;
              preCpuLoadValue = cpuLoadValue;
              preCpuTempValue = cpuTempValue;
              preGpuLoadValue = gpuLoadValue;
              preGpuTempValue = gpuTempValue;
              
              xSemaphoreGive(xMutex);// 释放锁 xMutex
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
              analogWrite(dacPin26, 0);
            }
          }
        }
    } else {
      Serial.println("未能获取 WiFi信息 锁");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void Task3();

void setup() {
  Serial.begin(115200);
  delay(10);

  Serial.println("1.OLED初始化...");
  while(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
   { 
    Serial.println("SSD1306 allocation failed");
    delay(1000);
  }
  Serial.println("1.OLED初始化成功");
  display.clearDisplay();// 清除显示
  display.setTextSize(2);// 设置文本大小
  display.setTextColor(WHITE);// 设置文本颜色
  display.setCursor(0, 0);//设置显示坐标
  display.println("ready");// 
  display.display(); // 屏幕上实际显示文本

  // Connect to WiFi access point.
  Serial.println("2.WiFi连接中...");
  connectToWiFi(connectTimeOut_s);
  delay(500);

  // Set Adafruit IO's root CA
  client.setCACert(adafruitio_root_ca);
  mqtt.subscribe(&sub);

  // 初始化LED引脚为输出模式
  pinMode(ledPin, OUTPUT);
  // 初始化开关1模式
  pinMode(switch1Pin, INPUT_PULLUP);
  // 重置初始化操作
  pinMode(resetPin, INPUT);
  // 初始化电压表
  pinMode(dacPin25, OUTPUT);
  analogWrite(dacPin25, 0);
  pinMode(dacPin26, OUTPUT);
  analogWrite(dacPin26, 0);

  // 创建互斥锁
  xMutex = xSemaphoreCreateMutex();
  mutex_wifi = xSemaphoreCreateMutex();
  // 创建任务1
  //xTaskCreate(Task1, "Task1", 4096, NULL, 1, &Task1Handle);

  // 创建任务2
  //xTaskCreate(Task2, "Task2", 1024, NULL, 0, &Task2Handle);

  // 开启多线程调度器
  //vTaskStartScheduler();
  //xTaskCreatePinnedToCore(Task1, "TaskOne", 4096, NULL, 1, NULL, 0);//TaskOne在 0核心
  xTaskCreatePinnedToCore(Task2, "Task2", 1024, NULL, tskIDLE_PRIORITY, &Task2Handle, 0); // 在小核上面运行
}


void loop() {
  // 由 resetPin 引脚值，判断用户是否发起重新配置信息操作,需长按5s.
  if (digitalRead(resetPin)) {
    delay(5000);
    if (digitalRead(resetPin)) {
      Serial.println("正在清除当前配置信息...");
      // wifi 加锁
      xSemaphoreTake(mutex_wifi, portMAX_DELAY);
      restoreWiFi(); // 清除WiFi连接信息
      // todo:清除用户信息
      xSemaphoreGive(mutex_wifi);
      delay(500);
      ESP.restart(); //重启复位esp32,设置AP模式
    }
  }
  checkDNS_HTTP();                  //检测客户端DNS&HTTP请求，也就是检查配网页面那部分
  checkConnect(true);               //检测网络连接状态，参数true表示如果断开重新连接
  delay(30);
  // Serial.println(digitalRead(switch1Pin));
  if (WiFi.status() == WL_CONNECTED) {
      MQTT_connect();
      Adafruit_MQTT_Subscribe *subscription;
      while ((subscription = mqtt.readSubscription(1000))) {
        if (subscription == &sub) {
          const char* data = (char *)sub.lastread;
          // 创建 JSON 缓冲区，大小取决于数据大小
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

            if (digitalRead(switch1Pin)) { // 占用率
              cpuLoadRatio = (int)((float)cpuLoadValue/100.0 * 255);
              gpuLoadRatio = (int)((float)gpuLoadValue/100.0 * 255);
              analogWrite(dacPin25, cpuLoadRatio);
              analogWrite(dacPin26, gpuLoadRatio);
            } else {
              cpuTempRatio = (int)((float)cpuTempValue/100.0 * 255);
              gpuTempRatio = (int)((float)gpuTempValue/100.0 * 255);
              analogWrite(dacPin25, cpuTempRatio);
              analogWrite(dacPin26, gpuTempRatio);
            }

            oledShowRamLoad(memoryLoad);
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

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.println("3.MQTT服务器连接中...");

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
  Serial.println("3.MQTT服务器连接成功");
}

void (const char* val) {
    display.clearDoledShowRamLoadisplay();// 清除显示
    display.setTextSize(2);// 设置文本大小
    display.setTextColor(WHITE);// 设置文本颜色
    display.setCursor(0, 0);//设置显示坐标
    display.println("RAM Load:");
    display.print(val);
    display.print("%");
    display.print(" <Win");
    display.display(); // 屏幕上实际显示文本
}