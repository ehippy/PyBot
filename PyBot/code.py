# SPDX-FileCopyrightText: 2020 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import gamblor21_ahrs

DO_CALIBRATION = False 
if DO_CALIBRATION:
    import calibrate
    calibrate.calibrate()
    exit()

import math
import time
import board
import neopixel

import busio
i2c = busio.I2C(board.SCL, board.SDA)

from adafruit_motorkit import MotorKit
motor_kit = MotorKit(i2c=i2c)
left_motor = motor_kit.motor1
right_motor = motor_kit.motor2
right_motor.throttle = 0
left_motor.throttle = 0


import adafruit_lsm9ds1
from lib.gamblor21_ahrs import mahony
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
current_bearing = None

MAG_MIN = [-0.28602, -0.31486, -0.45374]
MAG_MAX = [0.6475, 0.53438, 0.43498]

## Used to calibrate the magenetic sensor
def map_range(x, in_min, in_max, out_min, out_max):
    """
    Maps a number from one range to another.
    :return: Returns value mapped to new range
    :rtype: float
    """
    mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if out_min <= out_max:
        return max(min(mapped, out_max), out_min)

    return min(max(mapped, out_max), out_min)


## create the ahrs_filter
ahrs_filter = mahony.Mahony(50, 5, 100)
orientation_count = 0  # used to count how often we are feeding the ahrs_filter
orientation_lastPrint = time.monotonic()  # last time we printed the yaw/pitch/roll values
orientation_timestamp = time.monotonic_ns()  # used to tune the frequency to approx 100 Hz


import adafruit_gps
gps_uart = busio.UART(board.TX, board.RX, baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(gps_uart, debug=False)
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,500")

RADIO_FREQ_MHZ = 915.0
import digitalio
import adafruit_rfm69
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D6)
reset = digitalio.DigitalInOut(board.D9)

rfm69 = adafruit_rfm69.RFM69(spi, cs, reset, 915.0)

current_latitude = 0.0
current_longitude = 0.0

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.1)

max_loops_without_rf_update = 10
joystick_drive_loops_since_rf = 0

MODE_JOYSTICK = "joystick"
MODE_GPS = "gps"

drive_mode = MODE_JOYSTICK

# expected input values 0.0 to 1.0
def drive_from_joystick(joy_x, joy_y):
    global left_motor, right_motor

    dead_zone = 0.2

    # print(f"Raw X/Y: {joy_x}, {joy_y}")

    max_value = 1.0

    # Invert X
    joy_x = ((max_value - joy_x) * 2)-1.0 
    joy_y = ((max_value - joy_y) * 2)-1.0

    # print(f"-1 - 1 normalized X/Y: {joy_x}, {joy_y}") 

    # calibration if needed
    joy_x -= 0.02 # -8
    joy_y += 0 # 13

    # Calculate R+L (Call it V): V =(100-ABS(X)) * (Y/100) + Y
    v = (max_value-abs(joy_x)) * (joy_y/max_value) + joy_y

    # Calculate R-L (Call it W): W= (100-ABS(Y)) * (X/100) + X
    w = (max_value-abs(joy_y)) * (joy_x/max_value) + joy_x

    # Calculate R: R = (V+W) /2
    r_throttle = (v + w) / 2
    
    # Calculate L: L = (V-W)/2
    l_throttle = (v - w) / 2

    if abs(r_throttle) < dead_zone:
        r_throttle = 0

    if abs(l_throttle) < dead_zone:
        l_throttle = 0

    # Do any scaling on R and L your hardware may require.
    # r_throttle = r_throttle / (max_value/2)
    # l_throttle = l_throttle / (max_value/2)

    r_throttle = max(-1, min(r_throttle, 1))
    l_throttle = max(-1, min(l_throttle, 1))

    # print(f"X:{joy_x}, Y:{joy_y} => R:{r_throttle}, L:{l_throttle}") # joystick control print

    right_motor.throttle = r_throttle
    left_motor.throttle = l_throttle

gps_radio_rate = 10
gps_radio_counter = 0
def gps_stuff():
    global current_latitude, current_longitude, gps_radio_counter
    try:
        gps.update()

        if not gps.has_fix:
            # print("Waiting for GPS fix...")
            if gps_radio_counter >= gps_radio_rate:
                rfm69.send("gps,bad")
                gps_radio_counter = 0
            gps_radio_counter += 1
            return
        # We have a fix! (gps.has_fix is true)
        # Print out details about the fix like location, date, etc.
        # print("=" * 40)  # Print a separator line.
        # print(
        #     "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
        #         gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
        #         gps.timestamp_utc.tm_mday,  # struct_time object that holds
        #         gps.timestamp_utc.tm_year,  # the fix time.  Note you might
        #         gps.timestamp_utc.tm_hour,  # not get all data like year, day,
        #         gps.timestamp_utc.tm_min,  # month!
        #         gps.timestamp_utc.tm_sec,
        #     )
        # )

        current_latitude = gps.latitude 
        current_longitude = gps.longitude

        # gps packet
        gps_packet = f"gps,{gps.latitude:.8f},{gps.longitude:.8f},{gps.fix_quality}"
        # print(gps_packet)
        
        if gps_radio_counter >= gps_radio_rate:
            rfm69.send(gps_packet)
            gps_radio_counter = 0
        
        gps_radio_counter += 1

        # print("Longitude: {0:.6f} degrees".format(gps.longitude))
        # print("Fix quality: {}".format(gps.fix_quality))
        # Some attributes beyond latitude, longitude and timestamp are optional
        # and might not be present.  Check if they're None before trying to use!
        # if gps.satellites is not None:
        #     print("# satellites: {}".format(gps.satellites))
        # if gps.altitude_m is not None:
        #     print("Altitude: {} meters".format(gps.altitude_m))
        # if gps.speed_knots is not None:
        #     print("Speed: {} knots".format(gps.speed_knots))
        # if gps.track_angle_deg is not None:
        #     print("Track angle: {} degrees".format(gps.track_angle_deg))
        # if gps.horizontal_dilution is not None:
        #     print("Horizontal dilution: {}".format(gps.horizontal_dilution))
        # if gps.height_geoid is not None:
        #     print("Height geo ID: {} meters".format(gps.height_geoid))
    except Exception as e:
        print("GPS exception")
        print(e)

def orienteering_stuff():
    global orientation_count, orientation_timestamp, orientation_lastPrint, current_bearing
    try:
        # on an Feather M4 approx time to wait between readings
        if (time.monotonic_ns() - orientation_timestamp) > 6500000:

            # read the magenetic sensor
            mx, my, mz = sensor.magnetic

            # adjust for magnetic calibration - hardiron only
            # calibration varies per device and physical location
            mx = map_range(mx, MAG_MIN[0], MAG_MAX[0], -1, 1)
            my = map_range(my, MAG_MIN[1], MAG_MAX[1], -1, 1)
            mz = map_range(mz, MAG_MIN[2], MAG_MAX[2], -1, 1)

            # read the gyroscope
            gx, gy, gz = sensor.gyro
            # adjust for my gyro calibration values
            # calibration varies per device and physical location
            gx -= 0.0143024
            gy -= 0.0275528
            gz += 0.0213714

            # read the accelerometer
            ax, ay, az = sensor.acceleration

            # update the ahrs_filter with the values
            # gz and my are negative based on my installation
            ahrs_filter.update(gx, gy, -gz, ax, ay, az, mx, -my, mz)

            orientation_count += 1
            orientation_timestamp = time.monotonic_ns()

        # every 0.1 seconds print the ahrs_filter values
        if time.monotonic() > orientation_lastPrint + 0.1:
            # ahrs_filter values are in radians/sec multiply by 57.20578 to get degrees/sec
            yaw = ahrs_filter.yaw * 57.20578
            if yaw < 0:  # adjust yaw to be between 0 and 360
                yaw += 360

            current_bearing = yaw
            rfm69.send(f"head,{current_bearing}")
            
            # print(
            #     "Orientation: ",
            #     yaw,
            #     ", ",
            #     ahrs_filter.pitch * 57.29578,
            #     ", ",
            #     ahrs_filter.roll * 57.29578,
            # )
            # print(
            #     "Quaternion: ",
            #     ahrs_filter.q0,
            #     ", ",
            #     ahrs_filter.q1,
            #     ", ",
            #     ahrs_filter.q2,
            #     ", ",
            #     ahrs_filter.q3,
            # )

            # print("Count: ", count)    # optionally print out frequency
            orientation_count = 0  # reset count
            orientation_lastPrint = time.monotonic()

    except Exception as e:
        # print(dir(sensor))
        print('orienteering broke', e)

def get_compass_bearing(pointA, pointB):
    if (type(pointA) != tuple) or (type(pointB) != tuple):
        raise TypeError("Only tuples are supported as arguments")

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

def distance_on_unit_sphere(lat1, long1, lat2, long2):
    # Convert latitude and longitude to
    # spherical coordinates in radians.
    degrees_to_radians = math.pi / 180.0

    # phi = 90 - latitude
    phi1 = (90.0 - lat1) * degrees_to_radians
    phi2 = (90.0 - lat2) * degrees_to_radians

    # theta = longitude
    theta1 = long1 * degrees_to_radians
    theta2 = long2 * degrees_to_radians

    # Compute spherical distance from spherical coordinates.

    # For two locations in spherical coordinates
    # (1, theta, phi) and (1, theta, phi)
    # cosine( arc length ) =
    #    sin phi sin phi' cos(theta-theta') + cos phi cos phi'
    # distance = rho * arc length

    cos = (math.sin(phi1) * math.sin(phi2) * math.cos(theta1 - theta2) +
            math.cos(phi1) * math.cos(phi2))
    arc = math.acos(cos)

    # Remember to multiply arc by the radius of the earth
    # in your favorite set of units to get length.
    return arc * 20925524.928 # earth radius in feet

def rf_drive(parts):
    if drive_mode != MODE_JOYSTICK:
        return
    global joystick_drive_loops_since_rf
    a_x = float(parts[1])
    a_y = float(parts[2]) 
    # a = int(packet[8:9])
    # b = int(packet[9:10])
    # x = int(packet[10:11])
    # y = int(packet[11:12])
    # sel = int(packet[12:13])
    pixel.fill((0,a_x*65000,a_y*65000))
    drive_from_joystick(a_x,a_y)
    joystick_drive_loops_since_rf = 0 

def radio_listen():
    global max_loops_without_rf_update, drive_mode
    try:
        packet = rfm69.receive(timeout=0.1) # 1/10 second
        if packet is not None:
            packet_text = str(packet, "ascii")
            print("Received (ASCII): {0}".format(packet_text))
            parts = packet_text.split(",")
            if parts[0] == "ctl":
                rf_drive(parts)
            if parts[0] == "mode":
                drive_mode = parts[1]
                print("got mode: " + drive_mode)
                pixel.fill((128,255,0))



    except Exception as e:
        print(f"radio_listen exception: {e}")

gps_target_accuracy = 5.0 # advance once withing 5 ft of target
gps_target_index = 0
gps_track = ( # starts in our driveway, drives straight up block, then zigzags back up the block home
    (-104.8656196422264,38.82017344546039),
    (-104.8655198774372,38.82025416399443),
    (-104.8656000673942,38.82036386184941),
    (-104.8656494691619,38.82049694762563),
    (-104.8656495284854,38.8206083843953),
    (-104.8656466903741,38.82076334214701),
    (-104.865597350391,38.82071528104596),
    (-104.8656989290283,38.82060985170269),
    (-104.865587900495,38.82049905978958),
    (-104.8656175403995,38.8202966575198),
    (-104.8654858600097,38.82029841010031),
    (-104.8655965762355,38.82020226244945)
)

def gps_drive():
    pixel.fill((255,0,0))

    if not gps.has_fix:
        # return
        current_latitude = 38.820087
        current_longitude = -104.865519

    global gps_target_index
    target_coords = gps_track[gps_target_index]
    # print(f"Target: {target_coords[0]}, {target_coords[1]}")
    distance = distance_on_unit_sphere(current_latitude, current_longitude, target_coords[0], target_coords[1])

    if distance < gps_target_accuracy:
        gps_target_index += 1
        target_coords = gps_track[gps_target_index]
        distance = distance_on_unit_sphere(current_latitude, current_longitude, target_coords[0], target_coords[1])

    bearing_to_target = get_compass_bearing((current_latitude, current_longitude), target_coords)

    print(f"Target Distance: {distance}, Bearing: {bearing_to_target}, Current Bearing: {current_bearing}")

    if abs(current_bearing - bearing_to_target) > 10:
        turn_in_place_towards_bearing(bearing_to_target)
    else:
        drive_towards_bearing(bearing_to_target)
    

def turn_in_place_towards_bearing(bearing_to_target):
    difference = bearing_to_target - current_bearing

    print(f"bearing difference: {difference}")
    # positive right turn, negative left turn

    if difference > 0:
        right_motor.throttle = -0.8
        left_motor.throttle = 0.8
        print("hard right")

    elif difference < 0:
        right_motor.throttle = 0.8
        left_motor.throttle = -0.8
        print("hard left")




    pass

def drive_towards_bearing(bearing_to_target):

    difference = bearing_to_target - current_bearing
    
    if abs(difference) < 3:
        right_motor.throttle = 1
        left_motor.throttle = 1
        print("drive straight!")
        return

    if difference > 0:
        right_motor.throttle = 0.9
        left_motor.throttle = 1
        print("skew right")

    else:
        right_motor.throttle = 1
        left_motor.throttle = 0.9
        print("skew left")
        
    pass

def joystick_drive_killswitch():
    global joystick_drive_loops_since_rf
    if drive_mode != MODE_JOYSTICK:
        return
    joystick_drive_loops_since_rf += 1
    if joystick_drive_loops_since_rf > max_loops_without_rf_update:
        print("joystick rf timeout, zeroing")
        joystick_drive_loops_since_rf = 0
        left_motor.throttle = 0
        right_motor.throttle = 0

while True: 
    radio_listen()
    orienteering_stuff()
    gps_stuff()
    if drive_mode == "gps":
        gps_drive()

    joystick_drive_killswitch()