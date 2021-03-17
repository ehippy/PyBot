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
text_area = label.Label(font, text=text, color=color, max_glyphs=30)
text_area.x = 10
text_area.y = 10

btn_area = label.Label(font, text=text, color=color, max_glyphs=30)
btn_area.x = 10
btn_area.y = 30

# Show it

homescreen_screen = displayio.Group(max_size=3)
homescreen_screen.append(text_area)
homescreen_screen.append(btn_area)

display.show(homescreen_screen)
# display.show(btn_area)

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

import neopixel
pixel_strip = neopixel.NeoPixel(board.D8, 5, brightness=0.1) 
# pixel_strip.fill((34,56,87))

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
            if packet_text == "gps no fix":
                pixel_strip[0] = (255,0,0)
                pixel_strip.show()
            # print("Received (ASCII): {0}".format(packet_text))
            if packet_parts[0] != "ctl":
                btn_area.text = packet_text
        except:
            print("paket error caught")
        

    try:
        x = joystick_x.value/65535
        y = joystick_y.value/65535
        # text_area.text = f"Stick: {x:0.0}, {y:0.0}"
        text_area.text = f"Signal: {rfm69.rssi}"
        
        pressed = pad.get_pressed()

        a_btn = (pressed & (1<<1)) != 0
        b_btn = (pressed & (1<<0)) != 0 
        start_btn = (pressed & (1<<2)) != 0 
        select_btn = (pressed & (1<<3)) != 0 

        # btn_area.text = f"A: {a_btn}, B: {b_btn}"

        spam = "ctl,"
        spam += f"{x:.2f}," + f"{y:.2f}"
        # spam = spam + f"{c['a']}{c['b']}{c['x']}{c['y']}{c['sel']}"
        if last_sent != spam or True:
            rfm69.send(spam)
            last_sent = spam
            # print("Sent " + spam)
        
    except Exception as e:
        print("fucking problem", e)
        pass
