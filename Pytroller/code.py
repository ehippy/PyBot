import time
import board
import neopixel

pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.1)
pixel.fill((0,255,0)) 

# Bluetooth Setup
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService

from adafruit_bluefruit_connect.packet import Packet
from adafruit_bluefruit_connect.button_packet import ButtonPacket

ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)


# Joywing Setup
from board import SCL, SDA
import busio
from micropython import const
 
from adafruit_seesaw.seesaw import Seesaw
 
BUTTON_RIGHT = const(6)
BUTTON_DOWN = const(7)
BUTTON_LEFT = const(9)
BUTTON_UP = const(10)
BUTTON_SEL = const(14)
button_mask = const(
    (1 << BUTTON_RIGHT)
    | (1 << BUTTON_DOWN)
    | (1 << BUTTON_LEFT)
    | (1 << BUTTON_UP)
    | (1 << BUTTON_SEL)
)
 
i2c_bus = busio.I2C(SCL, SDA)
 
ss = Seesaw(i2c_bus)
 
ss.pin_mode_bulk(button_mask, ss.INPUT_PULLUP)
 
last_x = 0
last_y = 0
 

def read_buttons():

    global last_x, last_y
    state = {
        'a_x': last_x,
        'a_y': last_y,
        'a': 0,
        'b': 0,
        'x': 0,
        'y': 0,
        'sel': 0
    }

    x = ss.analog_read(2)
    y = ss.analog_read(3)
    state['a_x'] = x
    state['a_y'] = y

    if (abs(x - last_x) > 3) or (abs(y - last_y) > 3):
        print(x, y)
        last_x = x
        last_y = y
 
    buttons = ss.digital_read_bulk(button_mask)
    if not buttons & (1 << BUTTON_RIGHT):
        print("Button A pressed")
        state['a'] = 1
 
    if not buttons & (1 << BUTTON_DOWN):
        print("Button B pressed")
        state['b'] = 1
 
    if not buttons & (1 << BUTTON_LEFT):
        print("Button Y pressed")
        state['y'] = 1
 
    if not buttons & (1 << BUTTON_UP):
        print("Button X pressed")
        state['x'] = 1
 
    if not buttons & (1 << BUTTON_SEL):
        print("Button SEL pressed")
        state['sel'] = 1
    
    return state

# BLE CODE DOWN HERE

ble = BLERadio()
uart_connection = None

while True:
    read_buttons()
    
    # See if any existing connections are providing UARTService.
    if ble.connected:
        for connection in ble.connections:
            if UARTService in connection:
                uart_connection = connection
            break

    if not uart_connection:
        print("Scanning...")
        for adv in ble.start_scan(ProvideServicesAdvertisement, timeout=5):
            if UARTService in adv.services:
                pixel.fill((0,255,255))
                print("found a UARTService advertisement")
                uart_connection = ble.connect(adv)
                break
        # Stop scanning whether or not we are connected.
        ble.stop_scan()

    while uart_connection and uart_connection.connected:
        try:
            pixel.fill((0,0,255))
            c = read_buttons()
            print(c)
            spam = f"{c['a_x']:04}" + f"{c['a_y']:04}"
            spam = spam + f"{c['a']}{c['b']}{c['x']}{c['y']}{c['sel']}"
            uart_connection[UARTService].write(spam)
        except OSError:
            try:
                uart_connection.disconnect()
            except:  # pylint: disable=bare-except
                pass
            uart_connection = None
        # time.sleep(0.3)