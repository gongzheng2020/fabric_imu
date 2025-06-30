/*
  Modbus-Arduino Example - Test Holding Register (Modbus IP ESP8266)
  Configure Holding Register (offset 100) with initial value 0xABCD
  You can get or set this holding register
  Original library
  Copyright by Andrиж Sarmento Barbosa
  http://github.com/andresarmento/modbus-arduino

  Current version
  (c)2017 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"
#include <QMC5883LCompass.h>
#include <AHRS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLE2901.h>
#include <Tone.h>
#include <U8g2lib.h>

float pressure, temp;
bool deviceConnected = false;
bool oldDeviceConnected = false;
const int mpuinterruptPin = 2;     // 定义MPU6050中断引脚
const int ledPin = 7;              // 定义LED引脚
unsigned long interval = 2000;     // 闪灯间隔时间（毫秒）
const int compassInterruptPin = 3; // 定义地磁计中断引脚
const int batteryPin = 0;          // 定义电池测量引脚
const float batteryThreshold = 3.58; // 关机电压mV
float batteryValue;
const int buzzerPin = 1;
bool buzzerStartup = true;
bool buzzerError = false;
bool buzzerSuccess = false;
bool buzzerLowbattery = false;
bool oledLowbattery = false;

QueueHandle_t sensorDataQueue;
TaskHandle_t ahrsTaskHandle;
TaskHandle_t sendTaskHandle;
TaskHandle_t blinkTaskHandle;
TaskHandle_t batteryTaskHandle;
TaskHandle_t buzzerTaskHandle;
TaskHandle_t oledTaskHandle;

MPU6050 mpu(0x69);
QMC5883LCompass compass;
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/4, /* data=*/5, /* reset=*/U8X8_PIN_NONE);
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;
Tone buzzer(buzzerPin);

class CustomAHRS : public AHRS
{
public:
    uint32_t getSystemTime() override
    {
        return xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    void fetchRawSensorData() override
    {
        if (digitalRead(mpuinterruptPin) == HIGH)
        {
            mpu.getMotion6(&raw_accel[0], &raw_accel[1], &raw_accel[2], &raw_gyro[0], &raw_gyro[1], &raw_gyro[2]);
        }
        if (digitalRead(compassInterruptPin) == HIGH)
        {
            compass.read();
            raw_magnetom[0] = compass.getX();
            raw_magnetom[1] = compass.getY();
            raw_magnetom[2] = compass.getZ();
        }
    }
    void sendData(uint8_t *data, size_t length) override
    {
        if (deviceConnected)
        {
            pCharacteristic->setValue(data, length);
            pCharacteristic->notify();
        }
    }
};

CustomAHRS ahrs; // 使用自定义的 AHRS 类

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "0000ffe5-0000-1000-8000-00805f9a34fb"
#define CHARACTERISTIC_UUID "0000ffe4-0000-1000-8000-00805f9a34fb"

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

void setup()
{
    Serial.begin(921600);
    // Create the BLE Device
    BLEDevice::init("WT1123");
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // Create a BLE Characteristic
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
    // Creates BLE Descriptor 0x2902: Client Characteristic Configuration Descriptor (CCCD)
    pCharacteristic->addDescriptor(new BLE2902());
    // Adds also the Characteristic User Description - 0x2901 descriptor
    descriptor_2901 = new BLE2901();
    descriptor_2901->setDescription("AHRS Server");
    descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ); // enforce read only - default is Read|Write
    pCharacteristic->addDescriptor(descriptor_2901);
    // Start the service
    pService->start();
    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();

    Wire.begin(18, 10);
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    u8g2.begin();
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // 设置陀螺仪量程为 ±2000°/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // 设置加速度计量程为 ±16g
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);             // 设置数字低通滤波器带宽为 98Hz
    mpu.setIntDataReadyEnabled(true);                // 启用数据就绪中断
    mpu.setInterruptMode(false);                     // 设置中断为高电平模式
    mpu.setInterruptLatch(true);                     // 设置中断为电平模式
    // compass.setADDR(0x30);
    compass.init();
    compass.setMode(0x01, 0x0C, 0x00, 0X80);
    delay(1000);
    ahrs.init();
    delay(100);
    pinMode(ledPin, OUTPUT);             // 设置LED引脚为输出模式
    digitalWrite(ledPin, HIGH);          // 初始化LED为关闭状态
    pinMode(mpuinterruptPin, INPUT);     // 设置陀螺仪中断引脚为输入模式
    pinMode(compassInterruptPin, INPUT); // 设置地磁计中断引脚为输入模式

    sensorDataQueue = xQueueCreate(30, sizeof(SensorData)); // 创建消息队列
    xTaskCreate(ahrs_task, "ahrs_task", 1024, NULL, 6, &ahrsTaskHandle);
    xTaskCreate(send_task, "send_task", 1024, NULL, 5, &sendTaskHandle);
    xTaskCreate(battery_task, "battery_task", 1024, NULL, 4, &batteryTaskHandle);
    xTaskCreate(buzzer_task, "buzzer_task", 1024, NULL, 3, &buzzerTaskHandle);
    xTaskCreate(oled_task, "oled_task", 2048, NULL, 2, &oledTaskHandle);
    xTaskCreate(blink_task, "blink_task", 512, NULL, 1, &blinkTaskHandle);
}

void ahrs_task(void *param)
{
    SensorData data;
    const TickType_t loopDelay = pdMS_TO_TICKS(5); // 5ms 循环间隔
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        if (uxQueueSpacesAvailable(sensorDataQueue) == 0)
        {
            xQueueReceive(sensorDataQueue, &data, 0);  //队列满丢弃头部数据
        }
        ahrs.run_once(5);
        ahrs.getData(SensorType::Euler, Axis::Invalid, data.euler);
        ahrs.getData(SensorType::Accel, Axis::Invalid, data.accel);
        ahrs.getData(SensorType::Gyro, Axis::Invalid, data.gyro);
        xQueueSend(sensorDataQueue, &data, 0);
        // Serial.print(data.accel[0]);
        // Serial.print(",");
        // Serial.print(data.accel[1]);
        // Serial.print(",");
        // Serial.print(data.accel[2]);
        // Serial.print("\r\n");
        vTaskDelayUntil(&lastWakeTime, loopDelay); // 精确延迟到下一个周期
    }
}

void send_task(void *param)
{
    SensorData data;
    while (1)
    {
        if (deviceConnected)
        {
            if (xQueueReceive(sensorDataQueue, &data, 0))
            {
                ahrs.data_pack(data);
            }
        }
        if (!deviceConnected && oldDeviceConnected)
        {
            buzzerError = true;
            interval = 7000;
            digitalWrite(ledPin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(5000)); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising();     // restart advertising
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected)
        {
            // do stuff here on connecting
            buzzerSuccess = true;
            interval = 400;
            digitalWrite(ledPin, HIGH);
            oldDeviceConnected = deviceConnected;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void oled_task(void *param)
{
    SensorData data;
    while (1)
    {
        if(oledLowbattery)
        {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_8x13_tr);
            u8g2.setCursor(10, 34);
            u8g2.printf("Low Battery!!!");
            u8g2.sendBuffer();
            vTaskDelay(pdMS_TO_TICKS(2000));
            u8g2.clearBuffer();
            u8g2.sendBuffer();
            oledLowbattery = false;
            while(1)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        else if(xQueuePeek(sensorDataQueue, &data, 0)) // 从消息队列读取数据但不移除
        {
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_5x8_tr);

            // 左侧显示欧拉角
            u8g2.setCursor(0, 10);
            u8g2.printf("Roll :%6.2f", data.euler[0]);

            u8g2.setCursor(0, 22);
            u8g2.printf("Pitch:%6.2f", data.euler[1]);

            u8g2.setCursor(0, 34);
            u8g2.printf("Batty:%6.2f", batteryValue);
            
            // 右侧显示加速度
            u8g2.setCursor(72, 10);
            u8g2.printf("AX:%6.2f", data.accel[0]);

            u8g2.setCursor(72, 22);
            u8g2.printf("AY:%6.2f", data.accel[1]);

            u8g2.setCursor(72, 34);
            u8g2.printf("AZ:%6.2f", data.accel[2]);

            // 右侧显示角速度
            u8g2.setCursor(72, 46);
            u8g2.printf("GX:%6.2f", data.gyro[0]);

            u8g2.setCursor(72, 58);
            u8g2.printf("GY:%6.2f", data.gyro[1]);

            u8g2.setCursor(72, 70);
            u8g2.printf("GZ:%6.2f", data.gyro[2]);

            // 下方绘制指示图标
            int centerX = 30; // 圆心X坐标
            int centerY = 48; // 圆心Y坐标
            int radius = 6;  // 圆的半径

            // 绘制圆
            u8g2.drawCircle(centerX, centerY, radius);

            // 根据Roll和Pitch绘制移动的线
            int rollLineX = (int)(data.euler[0] * -15); // Roll影响水平线
            int pitchLineY = (int)(data.euler[1] * -6); // Pitch影响垂直线

            // 绘制水平线
            u8g2.drawLine(centerX-25, centerY+pitchLineY, centerX+25, centerY+pitchLineY);

            // 绘制垂直线
            u8g2.drawLine(centerX+rollLineX, centerY-10, centerX+rollLineX, centerY+10);

            // 绘制边框
            u8g2.drawLine(centerX-25, centerY-10, centerX+25, centerY-10);
            u8g2.drawLine(centerX-25, centerY+10, centerX+25, centerY+10);
            u8g2.drawLine(centerX+25, centerY-10, centerX+25, centerY+10);
            u8g2.drawLine(centerX-25, centerY-10, centerX-25, centerY+10);

            u8g2.sendBuffer(); // 将缓冲区内容发送到OLED
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void blink_task(void *param)
{
    while (1)
    {
        digitalWrite(ledPin, LOW);           // 打开LED
        vTaskDelay(pdMS_TO_TICKS(150));      // LED亮起150ms
        digitalWrite(ledPin, HIGH);          // 关闭LED
        vTaskDelay(pdMS_TO_TICKS(interval)); // 闪灯间隔
    }
}

void battery_task(void *param)
{
    while (1)
    {
        batteryValue = analogReadMilliVolts(batteryPin) * 0.003;
        if (batteryValue < batteryThreshold)
        {
            buzzerLowbattery = true;
            oledLowbattery = true;
            while(buzzerLowbattery || oledLowbattery)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            esp_deep_sleep_start();
        }
        // Serial.print("Battery Voltage: ");
        // Serial.print(batteryValue);
        // Serial.print(" mV\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void buzzer_task(void *param)
{
    while (1)
    {
        if (buzzerStartup)
        {
            buzzer.play(startupMelody);
            vTaskDelay(pdMS_TO_TICKS(1000));
            buzzerStartup = false;
        }
        else if (buzzerError)
        {
            buzzer.play(errorMelody);
            vTaskDelay(pdMS_TO_TICKS(1000));
            buzzerError = false;
        }
        else if (buzzerLowbattery)
        {
            buzzer.play(errorMelody);
            vTaskDelay(pdMS_TO_TICKS(1000));
            buzzerLowbattery = false;
            while(1)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        else if (buzzerSuccess)
        {
            buzzer.play(successMelody);
            vTaskDelay(pdMS_TO_TICKS(1000));
            buzzerSuccess = false;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

void loop()
{
}
