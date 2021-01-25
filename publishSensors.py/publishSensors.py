import RPi.GPIO as GPIO
import time
import json
import Adafruit_DHT as dht
from Adafruit_BME280 import *
from Adafruit_CCS811 import Adafruit_CCS811
import Adafruit_TCS34725
import math
import platform
from pprint import pprint
import geocoder
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import arrow
import copy
import threading
from time import sleep
import smbus
import serial
import random
import string

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#setting GPIO pins as an input/output
GPIO.setup(23, GPIO.IN)
GPIO.setup(5, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

# Getting ip address
ipAddress = geocoder.ip('me')


sensorValues = [None]*12 # Setting a static list size to the number of devices
sensorNames = [None]*12

indexNum = 0

#finding that all sensors work or some of them work or none of them
errorList = []
numTrue = 0
numFalse = 0

#difference between DHT11 and DHT22:
#1.the DHT11 is cheaper.
#2.the DHT22 is more precise and lasts longer.

#reading DHT22 sensor
def dht22():
    
    try:
        h, t = dht.read_retry(dht.DHT22, 17)
        t = round(t,2) #temperature
        h = round(h,2) #humidity

        sensorValues[0] = t
        sensorValues[1] = h
        sensorNames[0] = "dhtTemp"
        sensorNames[1] = "dhtHum"
        
        errorList.append(False) #this sensor works
    except:
        errorList.append(True) #this sensor doesn't work
        pass
    
#reading BMP280 sensor
def bmp280():
    try:
        sensor = BME280(t_mode=BME280_OSAMPLE_8, p_mode=BME280_OSAMPLE_8, h_mode=BME280_OSAMPLE_8)
        degrees = sensor.read_temperature() #temperature in Celcius
        pascals = sensor.read_pressure() #pressure
        hectopascals = pascals / 100
        humidity = sensor.read_humidity() #humidity
        
        #rounding to two precision(e.x. if degrees is 20.234, it goes to 20.23)
        degrees = round(degrees, 2)
        hectopascals = round(hectopascals, 2)
        humidity = round(humidity, 2)
        
        sensorValues[2] = degrees
        sensorValues[3] = hectopascals
        sensorValues[4] = humidity
        
        sensorNames[2] = "bmpTemp"
        sensorNames[3] = "bmpPressure"
        sensorNames[4] = "bmpHum"
        errorList.append(False)
        
    except IOError:
        errorList.append(True)
        pass
    
#reading the TCS34725 color sensor
def tcs():
    try:
        tcs = Adafruit_TCS34725.TCS34725()
        
        #Enable or disable interrupts
        tcs.set_interrupt(False)
        
        #returns a tuple with the red, green, blue, clear color values
        r, g, b, c = tcs.get_raw_data()
        
        #calculate color-temperature and lux
        color_temp = Adafruit_TCS34725.calculate_color_temperature(r, g, b)
        lux = Adafruit_TCS34725.calculate_lux(r, g, b)
        
        
        sensorValues[5] = color_temp
        sensorValues[6] = lux
        
        sensorNames[5] = "colorTemp"
        sensorNames[6] = "lux"
        
        tcs.set_interrupt(True)
        tcs.disable()
        errorList.append(False)
        
    except:
        errorList.append(True)
        pass

#reading the PIR motion sensor
def pirMotion():
    try:
        #if there is motion,it will return True
        if GPIO.input(23):
            sensorValues[7] = "motion detected"
            sensorNames[7] = "motion"
            
            # waiting so as to CPU become completely ready
            time.sleep(1.5)
            errorList.append(False)
        else:
            
            sensorValues[7] = "no motion"
            sensorNames[7] = "motion"
            errorList.append(False)
        time.sleep(2)
            
    except IOError:
        errorList.append(True)
        pass
    
    
#reading the gas sensor from serial of raspberry pi(gas sensor connects to arduino uno board and this arduino connects to raspberry through usb cable)
def mq9():
    # open serial port
    """easily you can be informed about each port of raspberry with executing this code in the terminal:
    for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do(
        syspath="${sysdevpath%/dev}"
        devname="$(udevadm info -q name -p $syspath)"
        [[ "$devname" == "bus/"* ]] && continue
        eval "$(udevadm info -q property --export -p $syspath)"
        [[ -z "$ID_SERIAL" ]] && continue
        echo "/dev/$devname - $ID_SERIAL"
    )
    done"""
    ser = serial.Serial('/dev/ttyACM0',9600, 8, 'N', 1, timeout=5)
    
    #discarding all input buffer contents
    ser.flushInput()
    
    #the first time data comes takes 30 seconds less than other times so I add this in order to equalize the time of priting datas
    time.sleep(30)
    
    while True:
        try:
            if(ser.in_waiting>0):# in_waiting returns the number of bytes in buffer
                myData=ser.readlines()
                length=len(myData)
                for i in range(length):
                    if i==length-1:
                        pass
                    else:
                        sensorValues[8+i]=myData[i].decode('utf-8',errors='ignore')
                        sensorNames[8+i]="gas"
                errorList.append(False)
                
        except:
            errorList.append(True)
            
            
#Define and assign event callbacks
def main():
  
  def on_connect(client, userdata, flags, rc):
    #deviceName = "raspberrypi"
    client.subscribe("handshake-reply/" + deviceName)

  def on_message(client, userdata, msg):
    handshakeReply = json.loads(msg.payload) #json.loads read the data from json file
    print("\n")
    print(handshakeReply)
    integrationResult = handshakeReply["integration-result"]
    deviceID = handshakeReply["passport"]["device-id"]
 
    with open('sensorData.json') as sensorData:
      sensorData = json.loads(sensorData.read())

    with open('data_update.json') as dataUpdate:
      dataUpdate = json.loads(dataUpdate.read())
      dataUpdate["passport"]["message-topic"] = "data-update/" + deviceID
      dataUpdate["passport"]["private-key"] = privateKey

      utc = arrow.utcnow() #illustrates the date and time
      local = utc.to('local') #set the date and time to local ones
    
    deviceIntegrated = False;
     
    while True:
      if integrationResult == "unsuccessful":
        
        errorList = []
        numTrue = 0
        numFalse = 0

        #running several different programs at the same time
        DHT = threading.Thread(target=dht22)
        BMP = threading.Thread(target=bmp280)
        TCS = threading.Thread(target=tcs)
        PIR = threading.Thread(target=pirMotion)
        MQ9 = threading.Thread(target=mq9)

        #starting thread
        DHT.start()
        BMP.start()
        TCS.start()
        PIR.start()
        MQ9.start()

        #joining thread
        DHT.join()
        BMP.join()
        TCS.join()
        PIR.join()
        ##if MQ9.is_alive():
        #for gas sensor,you should wait 8 seconds because it needs some times to join(according to my experience 8 seconds is enough)
        MQ9.join(8.0)

        #appending values of sensors and their names in json files
        for i in range(numOfDevices):
            if (i < 8 and i >= 0):
                if (len(sensorData["value"]) == 0):
                    sensorData["value"].append(sensorValues[i])
                    sensorData["capture-time"] = str(local.format())
                    dataUpdate["data"]["sensor"].append(sensorData)
                    dataUpdate["data"]["sensor"][i]["type"] = sensorNames[i]
                else:
                    if deviceIntegrated == True:
                        dataUpdate["data"]["sensor"][i]["value"].append(sensorValues[i])

                    else:
                        dataUpdate["data"]["sensor"].append(copy.deepcopy(sensorData))
                    dataUpdate["data"]["sensor"][i]["value"].pop(0)
                        dataUpdate["data"]["sensor"][i]["value"].append(sensorValues[i])
                        dataUpdate["data"]["sensor"][i]["type"] = sensorNames[i]
            
            elif (i >= 8 and i < numOfDevices):
                if (len(sensorData["value"]) == 0):
                    time.sleep(30)
                    sensorData["value"].append(sensorValues[i])
                    sensorData["capture-time"] = str(local.format())
                    dataUpdate["data"]["sensor"].append(sensorData)
                    dataUpdate["data"]["sensor"][i]["type"] = sensorNames[i]
                else:
                    if (i == numOfDevices - 1):
                        
                        if deviceIntegrated == False:
                            time.sleep(60)
                            dataUpdate["data"]["sensor"].append(copy.deepcopy(sensorData))
                        dataUpdate["data"]["sensor"][i]["value"].pop(0)
                            dataUpdate["data"]["sensor"][i]["value"].append(sensorValues[i])
                            dataUpdate["data"]["sensor"][i]["type"] = sensorNames[i]
                            deviceIntegrated = True;
                        else:
                            time.sleep(60)
                            dataUpdate["data"]["sensor"][i]["value"].append(sensorValues[i])
                    else:
                        if deviceIntegrated==True:
                            time.sleep(60)
                            dataUpdate["data"]["sensor"][i]["value"].append(sensorValues[i])
                        else:
                            time.sleep(60)
                            dataUpdate["data"]["sensor"].append(copy.deepcopy(sensorData))
                            dataUpdate["data"]["sensor"][i]["value"].pop(0)
                            dataUpdate["data"]["sensor"][i]["value"].append(sensorValues[i])
                            dataUpdate["data"]["sensor"][i]["type"]=sensorNames[i]
                            
        publish.single("data-update/" + deviceID, json.dumps(dataUpdate), hostname = " ")
        print("\n")
        print(json.dumps(dataUpdate, sort_keys=True, indent=4)) #json.dumps writes the data to json file
       
        

  #Creating new client for subscribing(random id)
  client = mqtt.Client()
  client.on_connect = on_connect
  client.on_message = on_message
  
  client.subscribe(" ")
  client.connect(" ", 1883, 60)
  client.loop_start()
  with open('handshake_template.json') as handshake:
       handshake = json.loads(handshake.read())
       randomName = ''.join([random.choice(string.letters) for _ in range(5)])
       handshake["passport"]["device-name"] = randomName
       deviceName = randomName
       handshake["passport"]["device-OS"] = platform.system()
       handshake["passport"]["device-type"] = platform.node()
       handshake["passport"]["interfaces"]["wifi"] = "true"
       handshake["passport"]["interfaces"]["ethernet"] = "true"
       handshake["passport"]["power-source"]["battery"] = "false"
       handshake["passport"]["power-source"]["plugged"] = "true"
       handshake["passport"]["device-schema"] = "sensor/data"
       #handshake["passport"]["private-key"] = "abcedf"
       handshake["passport"]["private-key"] = """-----BEGIN PRIVATE KEY-----
-----END PRIVATE KEY-----"""
       privateKey = handshake["passport"]["private-key"]
       handshake["passport"]["geolocation"]["latitude"] = ipAddress.latlng[0]
       handshake["passport"]["geolocation"]["longitude"] = ipAddress.latlng[1]
       handshake["passport"]["port-type"]["digital"] = "true"
       handshake["passport"]["port-type"]["analog"] = "true"
       handshake["passport"]["port-type"]["command"] = "false"
       handshake["passport"]["data-generation"]["frequency"] = 20
       handshake["passport"]["data-generation"]["metric"] = "s/m"
       handshake["passport"]["data-generation"]["firmness"] = "true"
       handshake["configuration"]["current-power-source"] = "plugged"
       handshake["configuration"]["sensor"]["type"].extend(["temperature", "humidity", "temp", "pressure", "hum", "color temp", "luminosity", "motion", "gas", "gas", "gas"])
       numOfDevices = len(handshake["configuration"]["sensor"]["type"])
       handshake["configuration"]["sensor"]["model"].extend(["dht22", "dht22", "bmp280", "bmp280", "bmp280", "tcs34725", "tcs34725", "pir motion", "mq9", "mq9", "mq9"])
       handshake["configuration"]["sensor"]["mode"].extend(["digital", "digital", "digital", "digital", "digital", "digital", "digital", "digital", "analog", "analog", "analog"])
       handshake["configuration"]["actuator"]["type"].append("light")
       handshake["configuration"]["actuator"]["model"].append("M3")
       handshake["configuration"]["actuator"]["mode"] = "digital"
       handshake["configuration"]["active-interface"] = "ethernet"

       
  publish.single("handshake", json.dumps(handshake), hostname = "142.150.208.252")
  print(json.dumps(handshake, sort_keys=True, indent=4))
  while True:
    pass

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass

 
