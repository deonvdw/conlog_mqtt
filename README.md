# conlog_mqtt
MQTT interface to Conlog prepaid electricity meters

This project creates a MQTT interface to the Conlog BEC62 (08) prepaid meter equipped with a WMI (wireless interface).
At the moment other Conlog meter models are not supported - they  may be using different frequencies and/or protocols.

Hardware: ESP32 and Si4432 radio module. The Si4432 is problematic for a number of reasons (future versions are likely
to target the Ti CC1101 instead):

1. The Conlog uses Ti CC1101 compatible packets. We have to hit the Si4432 sideways to send and receive these packets

2. The Si4432 takes a long time to switch between RX and TX mode and back causing us to lose messages. Maybe we are
doing something wrong, maybe the chip is bad. AN415 does not include all the steps we have to do to get a successful
switch.

Right now the code works nicely to monitor the stream of messages between the wireless keypad (wUIU) and the meter.
From this we can log the available kWh units left, any recharges and the power consumption.
