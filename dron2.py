import time
import socket
import exceptions
import math
import argparse
import os
import platform
import sys

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, APIException
from pymavlink import mavutil

##############################

vehicle = connect('/dev/ttyAMA0',baud=921600,wait_ready=True)
print(str(vehicle.system_status.state))

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    baud_rate = 57600
    vehicle = connect(connection_string,baud=baud_rate, wait_ready=True)
    return vehicle
########
def arm_and_takeoff():
    while vehicle.is_armable!=False:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    print("Vehicle is now armable.")
    print("")

    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    while vehicle.mode!='GUIDED_NOGPS':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle is now in GUIDED_NOGPS mode.")

    vehicle.armed = True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Look out! Drone is armed and props are spinning!")
    time.sleep(.5)

  #  vehicle.simple_takeoff(targetHeight)
  #  while True:
  #      current_altitude = vehicle.location.global_relative_frame.alt
  #      print("Current Altitude: "+str(current_altitude))
  #      if current_altitude >=.95*targetHeight:
  #          break
  #      time.sleep(1)
    return None

#############################MAIN#################

vehicle = connectMyCopter(1)
arm_and_takeoff()

#vehicle.mode = VehicleMode("LAND")
#while vehicle.mode!='LAND':
#    time.sleep(1)
#    print("Waiting for drone to enter LAND mode")
#print("Drone is LAND mode. Exiting script")
