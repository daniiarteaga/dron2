# Import Necessary Packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time, socket, exceptions, argparse
#from sshkeyboard import listen_keyboard


def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    vehicle = connect(connection_string, wait_ready=True)

    return vehicle


def arm_and_takeoff(aTargetAltitude):
    while not vehicle.is_armable:
        print("Waiting for vehicle to become armable")
        time.sleep(1)
    while vehicle.mode != "GUIDED":
        print("Waiting for vehicle to enter GUIDED mode")
        time.sleep(1)
    vehicle.armed = True
    while vehicle.armed == False:
        print("Waiting for vehicle to become armed")
        time.sleep(1)
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print"Current Altitude: %d"vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * .95:
            break
        time.sleep(1)
    print("Target altitude reached")
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

        
        
