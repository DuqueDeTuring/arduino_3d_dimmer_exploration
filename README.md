Proof of concept for a bigger idea.

## Hardware:
- [Nano 33 IoT](https://docs.arduino.cc/hardware/nano-33-iot).

## Description

The Arduino's sketch is part of a setup involving an automation in [Home Assistant](https://www.home-assistant.io/) for controlling the brightness of a light via an [Arduino's gyroscope](https://docs.arduino.cc/tutorials/nano-33-iot/imu_gyroscope) (LSM6DS3 module: [datasheet PDF](https://content.arduino.cc/assets/st_imu_lsm6ds3_datasheet.pdf)) . 

The Arduino's code is completely independent from Home Assistant, it just posts attitude changes in the Y axis to one MQTT topic, so it can be used for anything.

In addition to the Arduino:
1. [MQTT](https://mqtt.org/) broker. For exploring ideas, I am just running a simple installation of [Mosquitto](https://mosquitto.org/) on a Pi.
2. Home Assistant with the [MQTT integration](https://www.home-assistant.io/integrations/mqtt/) installed and configured.
3. A Home Assistant MQTT switch tied to the topic where the Arduino sends the messages. (/ha_switch.yaml)
4. A couple of automations in HA: one for increasing the brightness and one for decreasing the brightness of the light (/arduino_ha_automation_test.yaml).


## End result

The brightness of the light increases when you rotate the Arduino upwards and decreases when it is rotated downwards (over its X axis), but to activate this the Arduino must be turned at least 80 degrees left over the Y axis and then to lock the reading it must be turned "right" at least 80 degrees (this makes sense if the Arduino boots horizontal like the board sitting on a table and the antenna pointing away from you).


