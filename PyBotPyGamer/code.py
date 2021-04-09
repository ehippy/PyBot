import board
import displayio
import terminalio
from adafruit_display_text import label

display = board.DISPLAY

# Set text, font, and color
text = "HELLO WORLD"
font = terminalio.FONT
color = 0xFFFFFF

# Create the text label
lbl_signal = label.Label(font, text="Signal", color=color, max_glyphs=30)
lbl_signal.x = 10
lbl_signal.y = 10

lbl_heading = label.Label(font, text="Heading", color=color, max_glyphs=30)
lbl_heading.x = 10
lbl_heading.y = 30

lbl_gps = label.Label(font, text="GPS", color=color, max_glyphs=30)
lbl_gps.x = 10
lbl_gps.y = 50

lbl_mode = label.Label(font, text="Mode", color=color, max_glyphs=30)
lbl_mode.x = 10
lbl_mode.y = 70

# Show it

homescreen_screen = displayio.Group(max_size=10)
homescreen_screen.append(lbl_signal)
homescreen_screen.append(lbl_heading)
homescreen_screen.append(lbl_gps)
homescreen_screen.append(lbl_mode)

display.show(homescreen_screen)
# display.show(lbl_heading)

import time
import analogio
import math
import gamepadshift
from gamepadshift import GamePadShift
import digitalio
from board import SCL, SDA
import busio
from micropython import const

joystick_x = analogio.AnalogIn(board.JOYSTICK_X)
joystick_y = analogio.AnalogIn(board.JOYSTICK_Y)

pad = GamePadShift(digitalio.DigitalInOut(board.BUTTON_CLOCK),
                   digitalio.DigitalInOut(board.BUTTON_OUT),
				   digitalio.DigitalInOut(board.BUTTON_LATCH))


RADIO_FREQ_MHZ = 915.0
import digitalio
import adafruit_rfm69
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D6)
reset = digitalio.DigitalInOut(board.D9)

rfm69 = adafruit_rfm69.RFM69(spi, cs, reset, 915.0)

rfm69.send('Hello from bot!')
print("sent RF hello world!")

last_sent = ""


MODE_JOYSTICK = "joystick"
MODE_GPS = "gps"
drive_mode = MODE_JOYSTICK

import neopixel
pixel_strip = neopixel.NeoPixel(board.D8, 5, brightness=0.1) 
# pixel_strip.fill((34,56,87))

def handle_gps_packet(packet_parts):
    if packet_parts[1] == "bad":
        lbl_gps.text = "GPS no fix"
        pixel_strip[0] = (255,0,0)
        pixel_strip.show()
    else:
        lbl_gps.text = "GPS good fix"
        pixel_strip[0] = (0,255,0)
        pixel_strip.show()


def handle_heading_packet(packet_parts):
    lbl_heading.text = "Heading: " + packet_parts[1]

mode_change_start_time = time.monotonic()
while True:

    packet = rfm69.receive(timeout=0.05) # 1/10 second
    # If no packet was received during the timeout then None is returned.
    if packet is None:
        pass
    else:
        try:
            packet_text = str(packet, 'ascii', 'ignore')
            # print(packet_text)
            packet_parts = packet_text.split(",")
            print("Received (ASCII): {0}".format(packet_text))
            
            if packet_parts[0] == "gps":
                handle_gps_packet(packet_parts)

            if packet_parts[0] == "head":
                handle_heading_packet(packet_parts)

        except Exception as e:
            print("paket error caught " + str(e))
        

    try:
        x = joystick_x.value/65535
        y = joystick_y.value/65535
        # lbl_signal.text = f"Stick: {x:0.0}, {y:0.0}"
        lbl_signal.text = f"Signal: {rfm69.rssi}"
        
        pressed = pad.get_pressed()

        a_btn = (pressed & (1<<1)) != 0
        b_btn = (pressed & (1<<0)) != 0 
        start_btn = (pressed & (1<<2)) != 0 
        select_btn = (pressed & (1<<3)) != 0 

        # lbl_heading.text = f"A: {a_btn}, B: {b_btn}"

        spam = "ctl,"
        spam += f"{x:.2f}," + f"{y:.2f}"
        # spam = spam + f"{c['a']}{c['b']}{c['x']}{c['y']}{c['sel']}"
        if last_sent != spam:
            rfm69.send(spam)
            last_sent = spam
            # print("Sent " + spam) 
        
        if select_btn:
            now = time.monotonic()
            if now - mode_change_start_time > 1: # only change modes once per second
                mode_change_start_time = now
                print("changing mode")
                if drive_mode == MODE_JOYSTICK:
                    drive_mode = MODE_GPS
                else:
                    drive_mode = MODE_JOYSTICK

                lbl_mode.text = "Mode: " + drive_mode
                rfm69.send("mode," + drive_mode)
                print("sent mode")
            else:
                rfm69.send("mode," + drive_mode)
                print("no mode change, too recent, resent mode")
            
            
        
    except Exception as e:
        print("fucking problem", e)
        pass
