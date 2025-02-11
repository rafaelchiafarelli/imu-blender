#!/usr/bin/python

import os
import sys
sys.path.append(os.getcwd() + "/MPU6050")
import mpu6050
import time
import math
import csv
import socket
import smbus
from pycomms import PyComms
from gpiozero import LED
from time import sleep
b1 = LED(17)
b2 = LED(27)
b3 = LED(22)

def select_bone(bone):    
    if bone == 0:
        b1.off
        b2.on
        b3.on
    elif bone == 1:
        b1.on
        b2.off
        b3.on
    elif bone == 2:
        b1.on
        b2.on
        b3.off
    else:
        b1.on
        b2.on
        b3.on
    sleep(0.01)

# Send UDP Data
def send_data(msg):
    try:
        sock.sendto(bytes(msg,'utf-8'), (REMOTE_IP, DST_PORT))
    except socket.error as err:
        sock.close()
        print("Connection err!")

# UDP socket instance
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Read configuration from file (csv file format)
with open("imu-blender.cfg", 'r') as f:
    file = csv.DictReader(f)
    for rows in file:
        REMOTE_IP = rows['remote_ip']
        DST_PORT = int(rows['dst_port'])
        DEBUG = rows['debug']
        bones = int(rows['bones']) #amount of bones in the structure
        print("the amount of bones: {}, the debug:{}".format(bones,DEBUG))

# Sensor initialization // all mpu's are considered the same, the address will be changed
packetSize = []
mpu = mpu6050.MPU6050() #one sensor object to control all sensors
for bone in range(bones):
    #select the new bone
    select_bone(bone)
    print("configuring bone:{}".format(bone))
    sleep(0.5)
    mpu.dmpInitialize()
    mpu.setDMPEnabled(True)
    # get expected DMP packet size for later comparison
    packetSize.append(mpu.dmpGetFIFOPacketSize())

print ("IMU Sensor started (CTRL-C to stop)!")

while True:
    for bone in range(bones):
        #select the new bone
        select_bone(bone)
        # Get INT_STATUS byte
        mpuIntStatus = mpu.getIntStatus()
        if mpuIntStatus >= 0x10:
            # reset so we can continue cleanly
            mpu.resetFIFO()
            print('FIFO overflow!')
        else:
            # check for DMP data ready interrupt (this should happen frequently) 
            if mpuIntStatus >= 1:
                # get current FIFO count
                fifoCount = mpu.getFIFOCount()
                
                # check for overflow (this should never happen unless our code is too inefficient)
                if fifoCount == 1024:
                    # reset so we can continue cleanly
                    mpu.resetFIFO()
                    print('FIFO overflow!')
                else:
                    if fifoCount > packetSize[bone] or fifoCount == packetSize[bone]:
                        result = mpu.getFIFOBytes(fifoCount)
                        # Get quaternio, q return y, x, z, w
                        for shift in range(0,fifoCount/packetSize[bone],42):
                            q = mpu.dmpGetQuaternion(result[shift:])
                            data = mpu.dmpGet(q)

                            x = "{0:.6f}".format(data['x'])
                            y = "{0:.6f}".format(data['y'])
                            z = "{0:.6f}".format(data['z'])

                            yaw = "{0:.6f}".format(data['yaw'])
                            pitch = "{0:.6f}".format(data['pitch'])
                            roll = "{0:.6f}".format(data['roll'])

                            if DEBUG == "1":
                                print ("b:" + str(bone) + ", i:" + str(x) + ", j:" + str(y) + ", k:" + str(z) + ", y:" + str(yaw) + ", p:" + str(pitch) + ", r:" + str(roll))

                            # Sends quaternion through UDP
                            #send_data("{b:" + str(bone) + ", i:" + str(x) + ", j:" + str(y) + ", k:" + str(z) + ", y:" + str(yaw) + ", p:" + str(pitch) + ", r:" + str(roll)+"}")
                            

