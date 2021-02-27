  
# SPDX-FileCopyrightText: 2020 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Used with ble_uart_echo_client.py. Receives characters from the UARTService and transmits them back.
"""
import time
import board
import neopixel
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService


pixel = neopixel.NeoPixel(board.NEOPIXEL, 1, brightness=0.1)

ble = BLERadio()
uart = UARTService()
advertisement = ProvideServicesAdvertisement(uart)

while True:
    pixel.fill((0,255,0))
    print("Hunting Bluetooth daddies")
    ble.start_advertising(advertisement)
    while not ble.connected:
        pass
    while ble.connected:
        pixel.fill((0,0,255))
        # Returns b'' if nothing was read.
        one_byte = uart.read(6)
        if one_byte:
            print(one_byte)
            uart.write(one_byte)