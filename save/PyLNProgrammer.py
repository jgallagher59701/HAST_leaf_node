"""
Leaf Node 'Programmer' that can be used to configure Leaf nodes so that
parameters do not have to be modified in the source files. Parameters
that can be set by this program are:

 Node number
 Current time
 Start time
 Sample interval
 LoRa Channel

James Gallagher <jgallagher@opendap.org>
"""

import serial
import struct
import argparse
import time
import csv

# For OSX w/platformio
# RocketScream
# usb_port = "/dev/cu.usbmodem142101"
# ESP8266
# usb_port = "/dev/tty.SLAB_USBtoUART"
# Adafruit feather M0
# usb_port = "/dev/tty.usbmodem14101"
# UNO
# usb_port = "/dev/tty.usbmodem22101"

crlf = b'\r\n'


def send_int_param(serial_port, parameter, value, debug=True):
    """Send the parameter name then the integer value for that parameter"""
    param_name = parameter.encode('ascii')
    serial_port.write(param_name)
    serial_port.flush()

    # read debug
    if debug:
        msg = serial_port.readline()
        print(f"Read msg: {msg}")

    # read an ack
    msg = serial_port.readline()
    if msg != b'ACK' + crlf:
        print(f"Error while sending {parameter} name ({msg}).")
        return False

    if debug: print(f"ACK: {msg}")

    # Send the value, the receiver will assume four bytes
    serial_port.write(struct.pack('i', value))
    serial_port.flush()

    if debug:
        msg = serial_port.readline()
        print(f"Read msg: {msg}")

    # read ack and the value just sent
    msg2 = serial_port.readline()
    if msg2 != b'ACK' + crlf:
        print(f"Error while sending {parameter} value.")
        return False

    if debug: print(f"ACK: {msg2}")

    v = serial_port.readline()
    print(f"Value set by leaf node {v}.")

    return True


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("-i", "--interval", help="Sample period, in seconds", default=299, type=int)
    parser.add_argument("-n", "--node-number", help="The id number for this node (1-255)", default=1, type=int)
    parser.add_argument("-c", "--channel", help="LoRa for data transmission", default=1, type=int)

    parser.add_argument("-s", "--start-time", help="Date and time to wake up node. If not set, node runs immediately")

    parser.add_argument("-d", "--debug", help="Expect debug messages from leaf node", default=False, type=bool)
    parser.add_argument("-p", "--port", help="USB Serial port", default='/dev/tty.usbmodem22101')
    parser.add_argument("-b", "--baud", help="LoRa for data transmission", default=9600, type=int)

    args = parser.parse_args()

    # Wait for the MCU board to be plugged in to the USB port and the serial port to become active.
    success = False
    while not success:
        try:
            ser = serial.Serial(args.port, args.baud)
            ser.reset_input_buffer()
            success = True
        except serial.SerialException as inst:
            print(f"Caught exception {inst}")

    # Read from the MCU - if it sends a "hello" message, acknowledge and
    # begin sending configuration information

    msg = ser.readline()
    if msg == b'hello' + crlf:
        print(f"Success, msg: {msg}")
        ack = b'ACK'
        ser.write(ack)
        ser.flush()
    else:
        print(f"FAIL msg: {msg}")

    # read debug info
    if args.debug:
        msg = ser.readline()
        print(f"Read msg: {msg}")

    # Now that we have the attention of the LN, send it configuration parameters

    print(f"Args: {args}")

    send_int_param(ser, 'interval', args.interval, debug=args.debug)
    # send_int_param(ser, 'node', args.node)
    # send_int_param(ser, 'channel', args.channel)


if __name__ == "__main__":
    main()
