# I2C INA 219 Example

## Overview

This example demonstrates how to use the ESP32-S3 with the INA 219 current sensor peripheral.

## How to use example

### Hardware Required

To run this example, you should have one ESP32-S3, as well as one INA 219 current sensor.

#### Pin Assignment(esp32s3):

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                     | SDA   | SCL   |
| ------------------- | ----- | ----- |
| ESP32-S3 I2C Master | GPIO1 | GPIO2 |
| INA219 Sensor       | SDA   | SCL   |

- master:

  - GPIO1 is assigned as the data signal of I2C master port
  - GPIO2 is assigned as the clock signal of I2C master port

- Connection:

  - connect GPIO1 with SDA
  - connect GPIO2 with SCL

**Note:** It is recommended to add external pull-up resistors for SDA/SCL pins to make the communication more stable, though the driver will enable internal pull-up resistors.

### Configure the project

Open the project configuration menu (`idf.py menuconfig`). Then go into `Example Configuration` menu.

- In the `I2C Master` submenu, you can set the pin number of SDA/SCL according to your board. Also you can modify the I2C port number and freauency of the master.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type `Ctrl-]`.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.
