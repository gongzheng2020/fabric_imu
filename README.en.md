# Fabric_imu

<p align="center">
  <a href="README.md">中文</a> | <strong>English</strong>
</p>

<div align="center">
    <img src="assets/3d_model.jpg" width="500" alt="3D Model"/>
</div>

A portable measurement system based on a 9-axis IMU, designed to quantify fabric softness. Independently completed the full development pipeline from circuit design, enclosure modeling, embedded programming, to host computer data processing software.

## Demo

<div align="center">
    <img src="assets/demo.webp" width="500" alt="Measurement demo"/>
</div>

## Hardware Design

### Control Circuit

The main controller uses the **ESP32C2** microcontroller, consisting of a control board and an expansion board for modular design.

- **Control Board** — Integrates the MCU and a 9-axis gyroscope, responsible for calculating device姿态 and providing additional GPIO for expansion.

<div align="center">
  <table>
    <tr>
      <td><img src="assets/imu_board_top.png" width="300" alt="Control Board Top"/></td>
      <td><img src="assets/imu_board_3d.png" width="300" alt="Control Board 3D View"/></td>
    </tr>
    <tr>
      <td align="center">Top View</td>
      <td align="center">3D View</td>
    </tr>
  </table>
</div>

- **Expansion Board** — Integrates USB-to-serial converter, OLED screen, battery management system, and other components, providing power and additional expansion for the control board.

<div align="center">
  <table>
    <tr>
      <td><img src="assets/mother_board_top.png" width="300" alt="Expansion Board Top"/></td>
      <td><img src="assets/mother_board_3d.png" width="300" alt="Expansion Board 3D View"/></td>
    </tr>
    <tr>
      <td align="center">Top View</td>
      <td align="center">3D View</td>
    </tr>
  </table>
</div>

## Embedded Firmware

Built on FreeRTOS to manage multiple tasks, including: acquiring sensor data via SPI and computing 3D orientation using the DCM fusion algorithm; managing Bluetooth connections and transmitting data to the host computer; driving the OLED display via I2C; and monitoring battery level through ADC.

### GUI Host Software

Developed with Qt, the host software deploys algorithms, displays data waveforms, and processes/saves collected data.

<div align="center">
    <img src="assets/gui.png" width="500" alt="GUI Interface"/>
</div>
