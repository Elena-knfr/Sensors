# Raspberry Pi Sensor Integration
This repository contains Python code designed for Raspberry Pi to integrate various sensors and actuators, sending the gathered data to a remote server using MQTT (Message Queuing Telemetry Transport) protocol. The integration includes sensors like DHT22, BMP280, TCS34725, PIR Motion, and an MQ9 gas sensor.

## Features:
Diverse Sensor Support: Incorporates DHT22 for temperature and humidity, BMP280 for temperature, pressure, and humidity, TCS34725 for color temperature and luminosity, PIR Motion sensor, and an MQ9 gas sensor.
Multi-Threading: Utilizes multi-threading for parallel sensor readings, optimizing efficiency and responsiveness.
MQTT Communication: Communicates with a remote server using MQTT for data transmission.
Dynamic Configuration: Dynamically configures sensor types, models, and modes based on a JSON handshake template.
Error Handling: Provides robust error handling for sensor readings to ensure reliability.
Usage:
Clone the repository to your Raspberry Pi.
Configure the handshake_template.json file with your device information.
Execute the main() function to start the sensor integration process.
Monitor the data updates on the MQTT server.
Dependencies:
RPi.GPIO
Adafruit_DHT
Adafruit_BME280
Adafruit_CCS811
Adafruit_TCS34725
geocoder
paho-mqtt
arrow
