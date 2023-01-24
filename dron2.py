# Import Necessary Packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, socket, exceptions, argparse
#from sshkeyboard import listen_keyboard


vehicle = connect('/dev/ttyAMA0',baud=921600,wait_ready=True)
print(str(vehicle.system_status.state))

########
def arm_and_takeoff(targetHeight):
    while vehicle.is_armable!=True:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    print("Vehicle is now armable.")
    print("")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode!='GUIDED':
        print("Waiting for drone to enter GUIDED flight mode")
        time.sleep(1)
    print("Vehicle is now in GUIDED mode.")

    vehicle.armed = True
    while vehicle.armed==False:
        print("Waiting for vehicle to become armed.")
        time.sleep(1)
    print("Look out! Drone is armed and props are spinning!")
    time.sleep(.5)

    vehicle.simple_takeoff(targetHeight)
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print("Current Altitude: "+str(current_altitude))
        if current_altitude >=.95*targetHeight:
            break
        time.sleep(1)
    return None


def set_velocity_body(Vx, Vy, Vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0,0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFET_NED,
        0b0000111111000111,
        0, 0, 0,
        Vx, Vy, Vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

vehicle=connectMyCopter()
print("About to takeoff..")

arm_and_takeoff(4)

counter=0
while counter<2:
    set_velocity_body(1,0,0)
    print("Direction: NORTH relative to heading of drone")
    time.sleep(1)
    counter=counter+1

counter=0
while counter<2:
    set_velocity_body(-1,0,0)
    print("Direction: SOUTH relative to heading of drone")
    time.sleep(1)
    counter=counter+1

counter=0
while counter<2:
    set_velocity_body(0,1,0)
    print("Direction: EAST relative to heading of drone")
    time.sleep(1)
    counter=counter+1

counter=0
while counter<2:
    set_velocity_body(0,-1,0)
    print("Direction: WEST relative to heading of drone")
    time.sleep(1)
    counter=counter+1

vehicle.mode = VehicleMode("RTL")

print("End of function")
print("Arducopter version: %s"%vehicle.version)

while True:
    time.sleep(2)

vehicle.close()

        
        
