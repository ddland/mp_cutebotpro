# mp_cutebotpro
MicroPython driver for the [CuteBot Pro](https://shop.elecfreaks.com/products/elecfreaks-smart-cutebot-pro-v2-programming-robot-car-for-micro-bit) and the [Micro:Bit](https://microbit.org/).

## Status
Work in progress. Library can be used for (almost) all hardware on the CuteBot Pro, but is not yet documentend and/or tested. 

# MicroPython
I downloaded the `hex`-file from [MicroBit - MicroPython](https://github.com/microbit-foundation/micropython-microbit-v2/releases), version 2.1.12. Copying the `.hex` file to the USB-drive (microbit) should be enough.

## Running
Upload the code to the micro:bit (using [Thonny](thonny.org) or another IDE). The main.py is an example where the PID line-follow is started once the A-button is pressed. 
