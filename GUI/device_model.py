# coding:UTF-8
import threading
import time
import struct
import bleak
import asyncio


# 设备实例 Device instance
class DeviceModel:
    # region 属性 attribute
    # 设备名称 deviceName
    deviceName = "我的设备"

    # 设备数据字典 Device Data Dictionary
    deviceData = {}

    # 设备是否开启
    isOpen = False

    # 临时数组 Temporary array
    TempBytes = []

    # 将字节数组转换为浮点数 Convert byte array to float
    def bytesToFloat(self, byte_array):
        if len(byte_array) != 4:
            raise ValueError("Byte array must be exactly 4 bytes long")
        return struct.unpack('<f', bytes(byte_array))[0]

    def __init__(self, deviceName, BLEDevice, callback_method):
        print("Initialize device model")
        # 设备名称（自定义） Device Name
        self.deviceName = deviceName
        self.BLEDevice = BLEDevice
        self.client = None
        self.writer_characteristic = None
        self.isOpen = False
        self.callback_method = callback_method
        self.deviceData = {}

    # region 获取设备数据 Obtain device data
    # 设置设备数据 Set device data
    def set(self, key, value):
        # 将设备数据存到键值 Saving device data to key values
        self.deviceData[key] = value

    # 获得设备数据 Obtain device data
    def get(self, key):
        # 从键值中获取数据，没有则返回None Obtaining data from key values
        if key in self.deviceData:
            return self.deviceData[key]
        else:
            return None

    # 删除设备数据 Delete device data
    def remove(self, key):
        # 删除设备键值
        del self.deviceData[key]

    # endregion

    # 打开设备 open Device
    async def openDevice(self):
        print("Opening device......")
        # 获取设备的服务和特征 Obtain the services and characteristic of the device
        async with bleak.BleakClient(self.BLEDevice, timeout=15) as client:
            self.client = client
            # 设备UUID常量 Device UUID constant
            target_service_uuid = "0000ffe5-0000-1000-8000-00805f9a34fb"
            target_characteristic_uuid_read = "0000ffe4-0000-1000-8000-00805f9a34fb"
            target_characteristic_uuid_write = "0000ffe9-0000-1000-8000-00805f9a34fb"
            notify_characteristic = None

            print("Matching services......")
            for service in client.services:
                if service.uuid == target_service_uuid:
                    print(f"Service: {service}")
                    print("Matching characteristic......")
                    for characteristic in service.characteristics:
                        if characteristic.uuid == target_characteristic_uuid_read:
                            notify_characteristic = characteristic
                        if characteristic.uuid == target_characteristic_uuid_write:
                            self.writer_characteristic = characteristic
                    if notify_characteristic:
                        break

            if self.writer_characteristic:
                # 读取磁场四元数 Reading magnetic field quaternions
                print("Reading magnetic field quaternions")
                time.sleep(3)
                asyncio.create_task(self.sendDataTh())

            if notify_characteristic:
                print(f"Characteristic: {notify_characteristic}")
                # 设置通知以接收数据 Set up notifications to receive data
                await client.start_notify(notify_characteristic.uuid, self.onDataReceived)
                self.isOpen = True
                # 保持连接打开 Keep connected and open
                try:
                    while self.isOpen:
                        await asyncio.sleep(1)
                except asyncio.CancelledError:
                    pass
                finally:
                    # 在退出时停止通知 Stop notification on exit
                    await client.stop_notify(notify_characteristic.uuid)
            else:
                print("No matching services or characteristic found")

    # 关闭设备  close Device
    def closeDevice(self):
        self.isOpen = False
        print("The device is turned off")

    async def sendDataTh(self):
        while self.isOpen:
            # await self.readReg(0x3A)
            await time.sleep(0.1)
            # await self.readReg(0x51)
            await time.sleep(0.1)

    # region 数据解析 data analysis
    # 串口数据处理  Serial port data processing
    def onDataReceived(self, sender, data):
        tempdata = bytes.fromhex(data.hex())
        self.TempBytes = list(tempdata)  
        self.parseData(self.TempBytes)

    def parseData(self, data):
        # 状态机变量
        state = 0
        cnt = 0
        data_receive = []
        checkout = 0

        for dat in data:
            if state == 0 and dat == 0xAA:
                state += 1
            elif state == 1 and dat == 0x55:
                state += 1
            elif state == 2:
                data_receive.append(dat)
                cnt += 1
                if cnt >= 37:  # 数据包长度
                    # 校验和计算
                    checkout = sum(data_receive[:-1]) & 0xFF
                    if checkout == data_receive[-1]:
                        # print("Checksum passed, decoding data...")
                        self.processData(data_receive[:-1])
                    else:
                        print("Checksum failed!")
                    # 重置状态机
                    state = 0
                    cnt = 0
                    data_receive = []
            else:
                state = 0

    # 数据解析 data analysis
    def processData(self, Bytes):
        Bytes = Bytes[::-1]  # Reverse the order of the Bytes array
        Gz = self.bytesToFloat(Bytes[0:4])
        Gy = self.bytesToFloat(Bytes[4:8])
        Gx = self.bytesToFloat(Bytes[8:12])
        Az = self.bytesToFloat(Bytes[12:16])
        Ay = self.bytesToFloat(Bytes[16:20])
        Ax = self.bytesToFloat(Bytes[20:24])
        AngZ = self.bytesToFloat(Bytes[24:28]) * (180 / 3.141592653589793)
        AngY = self.bytesToFloat(Bytes[28:32]) * (180 / 3.141592653589793)
        AngX = self.bytesToFloat(Bytes[32:36]) * (180 / 3.141592653589793)
        self.set("AccX", round(Ax, 2))
        self.set("AccY", round(Ay, 2))
        self.set("AccZ", round(Az, 2))
        self.set("AsX", round(Gx, 2))
        self.set("AsY", round(Gy, 2))
        self.set("AsZ", round(Gz, 2))
        self.set("AngX", round(AngX, 2))
        self.set("AngY", round(AngY, 2))
        self.set("AngZ", round(AngZ, 2))
        # print(f"Euler Angles: AngX={round(AngX, 2)}, AngY={round(AngY, 2)}, AngZ={round(AngZ, 2)}")
        self.callback_method(self)

