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

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
// MPU6050 mpu;
MPU6050 mpu(0x69); // <-- use for AD0 high
QMC5883LCompass compass;
// ArtronShop_SPL06_001 pres(0x77, &Wire); // ADDR: 0 => 0x77, ADDR: 1 => 0x76
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;

float pressure, temp;
bool deviceConnected = false;
bool oldDeviceConnected = false;
const int mpuinterruptPin = 2;     // 定义MPU6050中断引脚
const int ledPin = 7;              // 定义LED引脚
unsigned long previousMillis = 0;  // 上一次闪灯的时间
unsigned long interval = 2000;     // 闪灯间隔时间（毫秒）
unsigned long ledOnMillis = 0;     // 记录LED亮起的时间
bool ledState = LOW;               // 当前LED状态
const int compassInterruptPin = 3; // 定义地磁计中断引脚
QueueHandle_t sensorDataQueue;

class CustomAHRS : public AHRS
{
public:
    uint32_t getSystemTime() override
    {
        return xTaskGetTickCount() * portTICK_PERIOD_MS; // 使用 FreeRTOS 的时间函数
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
    descriptor_2901->setDescription("My own description for this characteristic.");
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
    mpu.initialize();
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000); // 设置陀螺仪量程为 ±2000°/s
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16); // 设置加速度计量程为 ±16g
    mpu.setDLPFMode(MPU6050_DLPF_BW_98);             // 设置数字低通滤波器带宽为 98Hz
    mpu.setIntDataReadyEnabled(true);                // 启用数据就绪中断
    mpu.setInterruptMode(false);                     // 设置中断为高电平模式
    mpu.setInterruptLatch(true);                     // 设置中断为电平模式
    compass.init();
    compass.setMode(0x01, 0x0C, 0x00, 0X80);
    delay(1000);
    ahrs.init();
    delay(100);
    pinMode(ledPin, OUTPUT);             // 设置LED引脚为输出模式
    digitalWrite(ledPin, HIGH);          // 初始化LED为关闭状态
    pinMode(mpuinterruptPin, INPUT);     // 设置陀螺仪中断引脚为输入模式
    pinMode(compassInterruptPin, INPUT); // 设置地磁计中断引脚为输入模式

    sensorDataQueue = xQueueCreate(10, sizeof(SensorData)); // 创建消息队列
    xTaskCreate(ahrs_task, "ahrs_task", 2048, NULL, 3, NULL);
    xTaskCreate(send_task, "send_task", 1024, NULL, 2, NULL);
    xTaskCreate(blink_task, "blink_task", 512, NULL, 1, NULL);
}

void ahrs_task(void *param)
{
    SensorData data;
    const TickType_t loopDelay = pdMS_TO_TICKS(5); // 5ms 循环间隔
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1)
    {
        ahrs.run_once(5); // 每次运行一次 AHRS 算法
        ahrs.getData(SensorType::Euler, Axis::Invalid, data.euler);
        ahrs.getData(SensorType::Accel, Axis::Invalid, data.accel);
        ahrs.getData(SensorType::Gyro, Axis::Invalid, data.gyro);
        if (deviceConnected)
        {
            xQueueSend(sensorDataQueue, &data, 0); // 将数据发送到消息队列
        }
        Serial.print(data.euler[0]);
        Serial.print(",");
        Serial.print(data.euler[1]);
        Serial.print(",");
        Serial.print(data.euler[2]);
        Serial.print("\r\n");
        vTaskDelayUntil(&lastWakeTime, loopDelay); // 精确延迟到下一个周期
    }
}

void send_task(void *param)
{
    SensorData data;
    while (1)
    {
        if (xQueueReceive(sensorDataQueue, &data, 0)) // 从消息队列接收数据
        {
            ahrs.data_pack(data); // 调用 data_pack 函数发送数据
        }
        if (!deviceConnected && oldDeviceConnected)
        {
            interval = 7000;
            digitalWrite(ledPin, HIGH);
            vTaskDelay(pdMS_TO_TICKS(5000)); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising();     // restart advertising
            Serial.println("start advertising");
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected)
        {
            // do stuff here on connecting
            interval = 400;
            digitalWrite(ledPin, HIGH);
            oldDeviceConnected = deviceConnected;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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

void loop()
{
    // if(deviceConnected)
    // {
    //     ahrs.run_loop(5, true);
    // }
    // ahrs.calibration(0);

    // disconnecting

    // Serial.print(ahrs.getData(SensorType::Magnetom, Axis::X));
    // Serial.print(",");
    // Serial.print(ahrs.getData(SensorType::Magnetom, Axis::Y));
    // Serial.print(",");
    // Serial.print(ahrs.getData(SensorType::Magnetom, Axis::Z));
    // Serial.print("\n");
}
