  
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
        packet = uart.read(13).decode("utf-8") 
        if packet:
            print(packet)
            # print(type(packet))
            a_x = int(packet[0:4])
            a_y = int(packet[4:8])
            a = int(packet[8:9])
            b = int(packet[9:10])
            x = int(packet[10:11])
            y = int(packet[11:12])
            sel = int(packet[12:13])
            # print(a_y)
            # uart.write(one_byte)