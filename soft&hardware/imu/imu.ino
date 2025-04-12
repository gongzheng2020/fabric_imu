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
#include "MPU6050.h"  // not necessary if using MotionApps include file
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
MPU6050 mpu(0x69);  // <-- use for AD0 high
QMC5883LCompass compass;
// ArtronShop_SPL06_001 pres(0x77, &Wire); // ADDR: 0 => 0x77, ADDR: 1 => 0x76
BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLE2901 *descriptor_2901 = NULL;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float pressure, temp;
bool deviceConnected = false;
bool oldDeviceConnected = false;

AHRS ahrs;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "0000ffe5-0000-1000-8000-00805f9a34fb"
#define CHARACTERISTIC_UUID "0000ffe4-0000-1000-8000-00805f9a34fb"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

void AHRS::fetchRawSensorData() {
  // read raw accel/gyro measurements from device
  mpu.getMotion6(&raw_accel[0], &raw_accel[1], &raw_accel[2], &raw_gyro[0], &raw_gyro[1], &raw_gyro[2]);
  // Return mag XYZ readings
  compass.read();
  raw_magnetom[0] = compass.getX();
  raw_magnetom[1] = compass.getY();
  raw_magnetom[2] = compass.getZ();
}

void AHRS::sendData(uint8_t *data, size_t length) {
  if (deviceConnected) {
    pCharacteristic->setValue(data, length);
    pCharacteristic->notify();
  }
}

// void sensor_debug(void);
// void output_calibration(int calibration_sensor);

void setup() {
  Serial.begin(115200);
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
  descriptor_2901->setAccessPermissions(ESP_GATT_PERM_READ);  // enforce read only - default is Read|Write
  pCharacteristic->addDescriptor(descriptor_2901);
  // Start the service
  pService->start();
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  Wire.begin(18, 10);
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();
  compass.init();
  delay(1000);
  ahrs.init();
  delay(100);
}

void loop() {
//   if ((millis() - timestamp) >= 5) {
//     timestamp_old = timestamp;
//     timestamp = millis();
//     if (timestamp > timestamp_old)
//       G_Dt = (float)(timestamp - timestamp_old) / 1000.0f;  // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
//     else
//       G_Dt = 0;
//     ahrs.run();

//     // notify changed value
//     if (deviceConnected) {
//       uint8_t* date=data_pack(float *acc, float *gyro, float *ang);
//       pCharacteristic->setValue((uint8_t *)&date, 20);
//       pCharacteristic->notify();
//     }
//     // output_calibration(2);
//   }
  ahrs.run_loop(5);
  ahrs.data_pack();

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(10);                    // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // restart advertising
    Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }

  Serial.print(ahrs.getData(SensorType::Euler, Axis::X));
  Serial.print("\t");
  Serial.print(ahrs.getData(SensorType::Euler, Axis::Y));
  Serial.print("\t");
  Serial.print(ahrs.getData(SensorType::Euler, Axis::Z));
  Serial.print("\r\n");
}

// void sensor_debug(void) {
//   // display tab-separated accel/gyro x/y/z values
//   Serial.print("a/g/m/p:\t");
//   Serial.print(accel[0]);
//   Serial.print("\t");
//   Serial.print(accel[1]);
//   Serial.print("\t");
//   Serial.print(accel[2]);
//   Serial.print("\t");
//   Serial.print(gyro[0]);
//   Serial.print("\t");
//   Serial.print(gyro[1]);
//   Serial.print("\t");
//   Serial.print(gyro[2]);
//   Serial.print("\t");
//   Serial.print(magnetom[0]);
//   Serial.print("\t");
//   Serial.print(magnetom[1]);
//   Serial.print("\t");
//   Serial.print(mmagnetom[2]);
//   Serial.print("\r\n");
// }

// void output_calibration(int calibration_sensor) {
//   sensor_read();
//   if (calibration_sensor == 0)  // Accelerometer
//   {
//     // Output MIN/MAX values
//     Serial.print("accel x,y,z (min/max) = ");
//     for (int i = 0; i < 3; i++) {
//       if (accel[i] < accel_min[i])
//         accel_min[i] = accel[i];
//       if (accel[i] > accel_max[i])
//         accel_max[i] = accel[i];
//       Serial.print(accel_min[i]);
//       Serial.print("/");
//       Serial.print(accel_max[i]);
//       if (i < 2)
//         Serial.print("  ");
//       else
//         Serial.println();
//     }
//   } else if (calibration_sensor == 1)  // Magnetometer
//   {
//     // Output MIN/MAX values
//     Serial.print("magn x,y,z (min/max) = ");
//     for (int i = 0; i < 3; i++) {
//       if (magnetom[i] < magnetom_min[i])
//         magnetom_min[i] = magnetom[i];
//       if (magnetom[i] > magnetom_max[i])
//         magnetom_max[i] = magnetom[i];
//       Serial.print(magnetom_min[i]);
//       Serial.print("/");
//       Serial.print(magnetom_max[i]);
//       if (i < 2)
//         Serial.print("  ");
//       else
//         Serial.println();
//     }
//   } else if (calibration_sensor == 2)  // Gyroscope
//   {
//     // Average gyro values
//     for (int i = 0; i < 3; i++)
//       gyro_average[i] += gyro[i];
//     gyro_num_samples++;

//     // Output current and averaged gyroscope values
//     Serial.print("gyro x,y,z (current/average) = ");
//     for (int i = 0; i < 3; i++) {
//       Serial.print(gyro[i]);
//       Serial.print("/");
//       Serial.print(gyro_average[i] / (float)gyro_num_samples);
//       if (i < 2)
//         Serial.print("  ");
//       else
//         Serial.println();
//     }
//   }
// }
