import sys
import os
from PyQt5.QtWidgets import QApplication,QMainWindow
from stm32plot_ui import Ui_mainWindow
from PyQt5.QtCore import Qt,QTimer
from PyQt5 import QtGui
import numpy as np
import pyqtgraph as pg 
from PyQt5.QtGui import QPen 
from PyQt5.QtWidgets import QApplication,QStyleFactory
import asyncio
import bleak
import device_model
from threading import Thread
import math
import time
import winsound

def resource_path(relative_path):
    """获取打包后资源文件的绝对路径"""
    if hasattr(sys, '_MEIPASS'):
        # 如果是打包后的环境
        base_path = sys._MEIPASS
    else:
        # 开发环境，直接使用当前路径
        base_path = os.path.abspath(".")
    return os.path.join(base_path, relative_path)

class mainwindow(QMainWindow):
    def __init__(self):
        super().__init__()
        QApplication.setStyle(QStyleFactory.create('Fusion'))
        # 实例化一个 Ui_MainWindow对象
        self.ui=Ui_mainWindow()
        ico_path = resource_path('CQU.ico')
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(ico_path), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.setWindowIcon(icon)
        self.setFixedSize(620,355)
        # 设置鼠标点击获取焦点
        self.setFocusPolicy(Qt.NoFocus)
       	# setupUi函数
       	# 这个函数很多地方说是初始化ui对象，我觉得直接翻译为“设置UI”
       	# 这样表明ui对象的实例化和设置（或者说加载）是完全不相干的两步
        self.ui.setupUi(self)
        # 这里使用的是 self.show(),和之后的区分一下
        self.show()
        
        #初始化各种变量
        self.BLEaddress = None
        self.BLEdevice = None
        self.BLEthread = None
        self.TESTthread = None

        #初始状态的角度误差
        self.AngleX_C_error = 0
        self.AngleY_C_error = 0
        self.AngleZ_C_error = 0

        #初始状态的磁场误差
        self.AccelerateX_C_error = 0
        self.AccelerateY_C_error = 0
        self.AccelerateZ_C_error = 0

        #坡度
        self.Angle_slope = 0

        # 初始化数据数组
        self.timeperiod = 50  
        self.data = np.array([])  
        self.data2 = np.array([])  # 第二条数据线的数据  
        self.data3 = np.array([])  # 第三条数据线的数据  
        self.p = self.ui.graphicsView.addPlot()
        # 设置横纵坐标名称  
        self.p.setLabel('bottom', '时间(秒)')  # 横坐标，注意units是用来设置坐标轴刻度的单位  
        self.p.setLabel('left', '坡度(度)')  # 纵坐标1
        self.p.setLabel('right', '加速度(G)')  # 纵坐标2  
        # 设置背景颜色
        self.ui.graphicsView.setBackground(pg.mkBrush(255, 255, 255))
        # 创建纯黑色的画笔用于设置刻度颜色  
        black_pen = QPen(pg.mkColor('k'))  # 'k' 代表黑色  
        black_pen.setWidth(2)  # 设置刻度线的宽度  
        # 获取x轴和y轴对象，并设置其刻度画笔为黑色  
        self.p.getAxis('bottom').setPen(black_pen)  
        self.p.getAxis('left').setPen(black_pen)
        self.p.getAxis('right').setPen(black_pen)
        self.p.getAxis('left').setTextPen(black_pen) 
        self.p.getAxis('bottom').setTextPen(black_pen)
        self.p.getAxis('right').setTextPen(black_pen)  
        strs = range(0,210,20) # 设置每个刻度值的显示数值
        x = tuple(strs)     # 设置刻度值
        strs = [str(i*self.timeperiod*0.0005) for i in strs]
        ticks = [[i, j] for i, j in zip(x,strs)] # 刻度值与显示数值绑定
        self.p.getAxis('bottom').setTicks([ticks])
        self.p.showGrid(True,True,alpha=0.1)  # 显示网格
        self.p.setYRange(-90, 90)
        strs = range(-100,110,20) # 设置每个刻度值的显示数值
        x = tuple(strs)     # 设置刻度值
        strs = [str(i*0.025) for i in strs]
        ticks = [[i, j] for i, j in zip(x,strs)] # 刻度值与显示数值绑定
        self.p.getAxis('right').setTicks([ticks])
        self.p.setXRange(0, 200)
        self.p.addLegend(labelTextColor="k",offset=(1, 1))  # 添加图例
        # 创建三条数据线，设置画笔颜色和宽度
        self.line_plot = self.p.plot(pen=pg.mkPen('r', width=3), name="X轴加速度")   # 红色线条
        self.line_plot2 = self.p.plot(pen=pg.mkPen('b', width=3), name="坡度")  # 蓝色线条  
        self.line_plot3 = self.p.plot(pen=pg.mkPen('g', width=3), name="Y轴加速度")  # 绿色线条  
        # 设置定时器  
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)  

        # 连接信号和槽
        self.ui.pushButton_3.clicked.connect(self.scan)
        self.ui.comboBox_5.currentTextChanged.connect(self.chosen_device)
        self.ui.pushButton_4.clicked.connect(self.connect)
        self.ui.pushButton_2.clicked.connect(self.check_level)
        self.ui.pushButton.clicked.connect(self.Measure_Angle)

    def setAllEnabled(self, enabled):
        self.ui.pushButton_2.setEnabled(enabled)
        self.ui.pushButton_3.setEnabled(enabled)
        self.ui.pushButton_4.setEnabled(enabled)
        self.ui.comboBox_5.setEnabled(enabled)
        self.ui.pushButton.setEnabled(enabled)

    def closeEvent(self, event):
        if self.BLEdevice.isOpen:
            self.BLEdevice.closeDevice()
        event.accept()  # 关闭窗口

    def update_data(self, DeviceModel):
        self.Angle_slope = math.acos(math.cos( (DeviceModel.get("AngX") - self.AngleX_C_error) /180.0*math.pi)*math.cos((DeviceModel.get("AngY") - self.AngleY_C_error)/180.0*math.pi))
        # self.Angle_slope = math.acos(math.cos( (DeviceModel.get("AngX")) /180.0*math.pi)*math.cos((DeviceModel.get("AngY"))/180.0*math.pi))
        #print("Axy:{}".format(self.Angle_slope*180/math.pi))
        self.data = np.append(self.data, DeviceModel.get("AccX") * 40)
        self.data2 = np.append(self.data2, self.Angle_slope*180/math.pi)
        self.data3 = np.append(self.data3, DeviceModel.get("AccY") * 40)
        # print("x = {} , y = {} , z = {};" .format(DeviceModel.get("AngX") , DeviceModel.get("AngY") , DeviceModel.get("AngZ")))
        #print("x = {} , y = {} , z = {};" .format(DeviceModel.get("AccX") , DeviceModel.get("AccY") , DeviceModel.get("AccZ")))
        pass

    def update_plot(self):  
        # 仅保留最近的N个点，以避免数据无限增长  
        self.data = self.data[-200:]
        self.data2 = self.data2[-200:]  
        self.data3 = self.data3[-200:]  
        # 更新数据  
        self.line_plot.setData(self.data)
        self.line_plot2.setData(self.data2)
        self.line_plot3.setData(self.data3)
    
    def check_level(self):
        if self.BLEdevice is not None and self.BLEdevice.isOpen:
            self.ui.pushButton_2.setStyleSheet("background-color: red")
            self.ui.pushButton_2.setText("自校准中...")
            self.setAllEnabled(False)
            self.repaint()
            overtime = 0
            while 1:
                for i in range(0 , 20):
                    self.AngleX_C_error = self.AngleX_C_error + self.BLEdevice.deviceData.get('AngX')
                    self.AngleY_C_error = self.AngleY_C_error + self.BLEdevice.deviceData.get('AngY')
                    self.AngleZ_C_error = self.AngleZ_C_error + self.BLEdevice.deviceData.get('AngZ')
                    self.AccelerateX_C_error = self.AccelerateX_C_error + self.BLEdevice.deviceData.get('AccX')
                    self.AccelerateY_C_error = self.AccelerateY_C_error + self.BLEdevice.deviceData.get('AccY')
                    self.AccelerateZ_C_error = self.AccelerateZ_C_error + self.BLEdevice.deviceData.get('AccZ')
                    time.sleep(0.1)
                self.AngleX_C_error = self.AngleX_C_error * 0.05
                self.AngleY_C_error = self.AngleY_C_error * 0.05
                self.AngleZ_C_error = self.AngleZ_C_error * 0.05
                self.AccelerateX_C_error = self.AccelerateX_C_error * 0.05
                self.AccelerateY_C_error = self.AccelerateY_C_error * 0.05
                self.AccelerateZ_C_error = self.AccelerateZ_C_error * 0.05
                print("x_error = {:.3f} , y_error = {:.3f}".format(self.AngleX_C_error , self.AngleY_C_error))
                if abs(self.AngleX_C_error) < 2 and abs(self.AngleY_C_error) < 2:
                    print("自校正完成，同时水平处于水平状态")
                    print("Angle_Ex = {:.3} , Angle_Ey = {:.3} , Angle_Ez = {:.3}".format(self.AngleX_C_error , self.AngleY_C_error , self.AngleZ_C_error))
                    print("Accelerate_Ex = {:.3} , Accelerate_Ey = {:.3} , Accelerate_Ez = {:.3}".format(self.AccelerateX_C_error , self.AccelerateY_C_error , self.AccelerateZ_C_error))
                    winsound.Beep(2000, 300)
                    time.sleep(0.1)
                    winsound.Beep(2000, 300)
                    time.sleep(0.5)  
                    self.ui.pushButton_2.setStyleSheet("background-color: white")
                    self.ui.pushButton_2.setText("校准成功\r\n再次校准")
                    self.setAllEnabled(True)              
                    break
                else:
                    self.AngleX_C_error = 0
                    self.AngleY_C_error = 0
                    self.AngleZ_C_error = 0
                    self.MagneticX_C_error = 0
                    self.MagneticY_C_error = 0
                    self.MagneticZ_C_error = 0
                    if(overtime > 10):
                        print("over time! 目前的位置不水平已超时退出")
                        self.ui.pushButton_2.setStyleSheet("background-color: white")
                        self.ui.pushButton_2.setText("校准失败\r\n再次校准")
                        self.setAllEnabled(True)   
                        break
                    overtime = overtime + 1
    
    def Measure_Angle(self):
        if self.BLEdevice is not None and self.BLEdevice.isOpen:
            self.ui.pushButton.setStyleSheet("background-color: red")
            self.ui.pushButton.setText("测试中...")
            self.setAllEnabled(False)
            self.TESTthread = Thread(target=self.measure_angle_thread)
            self.TESTthread.start()

    def measure_angle_thread(self):
        while 1:
            time.sleep(0.1)
            if self.BLEdevice.deviceData.get('AccX')**2 + self.BLEdevice.deviceData.get('AccY')**2 + self.BLEdevice.deviceData.get('AccZ')**2 < 0.81:
                angle1 = self.Angle_slope
                angle2 = math.atan((9.8*math.sin(angle1) - abs(self.BLEdevice.deviceData.get('AccX'))-abs(self.BLEdevice.deviceData.get('AccY'))) / (9.8 * math.cos(angle1)))
                self.ui.lcdNumber.display(angle1 * 180 / math.pi)
                self.ui.lcdNumber_2.display(angle2 * 180 / math.pi)
                break
        print("滑动是的坡度angle1 = {:.3f}".format(angle1 * 180 / math.pi))
        print("匀速滑动的坡度angle2 = {:.3f}".format(angle2 * 180 / math.pi))
        print("AccX = {:.3} , AccY = {:.3} , AccZ = {:.3}".format(self.BLEdevice.deviceData.get('AccX') , self.BLEdevice.deviceData.get('AccY') , self.BLEdevice.deviceData.get('AccZ')))
        self.ui.pushButton.setStyleSheet("background-color: white")
        self.ui.pushButton.setText("开始测试")
        self.setAllEnabled(True)

    # 扫描蓝牙设备并过滤名称中包含“WT”的设备
    def scan(self):
        devices = []
        self.ui.comboBox_5.clear()
        self.setAllEnabled(False)
        self.ui.pushButton_3.setStyleSheet("background-color: red")
        self.ui.pushButton_3.setText("搜索中...")
        self.repaint()
        print("Searching for Bluetooth devices......")
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            devices = loop.run_until_complete(bleak.BleakScanner.discover(timeout=6.0))
            print("Search ended")
            for d in devices:
                if d.name is not None and "WT" in d.name:
                    self.ui.comboBox_5.addItem(d.address)
            if devices is None:
                self.BLEaddress = self.ui.comboBox_5.currentText()
                print("Chosen device: ", self.BLEaddress)
            self.setAllEnabled(True)
            self.ui.pushButton_3.setText("搜索设备")
            self.ui.pushButton_3.setStyleSheet("background-color: white")
        except Exception as ex:
            print("Bluetooth search failed to start")
            print(ex)
            
    def chosen_device(self):
        self.BLEaddress = self.ui.comboBox_5.currentText()
        print("Chosen device: ", self.BLEaddress)

    def connect(self):
        if self.BLEdevice is not None and self.BLEdevice.isOpen:
            self.timer.stop()
            if self.BLEdevice.isOpen:
                self.BLEdevice.closeDevice()
            self.ui.pushButton_4.setStyleSheet("background-color: white")
            self.ui.pushButton_4.setText("连接设备")
            return
        if self.BLEaddress is None:
            print("No device chosen")
            return
        self.setAllEnabled(False)
        self.ui.pushButton_4.setText("连接中...")
        self.repaint()
        print("Connecting to device: ", self.BLEaddress)
        try:
            self.BLEdevice = device_model.DeviceModel("MyBle5.0", self.BLEaddress, self.update_data)
            self.BLEthread = Thread(target=asyncio.run, args=(self.BLEdevice.openDevice(),))
            self.BLEthread.start()
            while not self.BLEdevice.isOpen:
                pass
            self.setAllEnabled(True)
            self.ui.pushButton_4.setStyleSheet("background-color: red")
            self.ui.pushButton_4.setText("断开设备")
            self.timer.start(self.timeperiod)
        except Exception as ex:
            print("Failed to connect to device")
            print(ex)
    

if __name__=="__main__":
    app=QApplication(sys.argv)
    window=mainwindow()
    sys.exit(app.exec_())
