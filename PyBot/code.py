  
# SPDX-FileCopyrightText: 2020 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Used with ble_uart_echo_client.py. Receives characters from the UARTService and transmits them back. 
"""
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
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)
current_bearing = None

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

    print(f"X:{joy_x}, Y:{joy_y} => R:{r_throttle}, L:{l_throttle}")

    right_motor.throttle = r_throttle
    left_motor.throttle = l_throttle

def gps_stuff():
    global current_latitude, current_longitude
    try:
        gps.update()

        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            # print("Waiting for GPS fix...")
            rfm69.send("gps no fix")
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
        print(gps_packet)
        rfm69.send(gps_packet)

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
    try:
        # print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.acceleration))
        # print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.magnetic))
        mag_x, mag_y, mag_z = sensor.magnetic
        bearing = 0

        # bearing = math.atan(mag_y/mag_x)*(180/math.pi) 
        # if bearing < 0:
        #     bearing = bearing + 360 

        bearing = 180-math.atan2(mag_x, mag_y)/0.0174532925 

        # print(bearing)
        # print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(*sensor.gyro))
        # print('Temp (degrees/sec): ({0:0.3f})'.format(*sensor.temperature))
        # print(sensor.temperature)
    except Exception as e:
        # print(dir(sensor))
        print('orienteering broke', e)

def rf_drive():
    global max_loops_without_rf_update, joystick_drive_loops_since_rf
    try:
        
        packet = rfm69.receive(timeout=0.05) # 1/20 second
        if packet is not None:
            packet_text = str(packet, "ascii")
            # print("Received (ASCII): {0}".format(packet_text))
            parts = packet_text.split(",")
            if parts[0] == "ctl":
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
        else:
            joystick_drive_loops_since_rf += 1

        # loss-of-signal kill switch
        if joystick_drive_loops_since_rf > max_loops_without_rf_update:
            print("joystick rf timeout, zeroing")
            left_motor.throttle = 0
            right_motor.throttle = 0

    except Exception as e:
        print(f"rf_drive exception: {e}")

while True:
    orienteering_stuff()
    gps_stuff()
    rf_drive()

