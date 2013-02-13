#Python testing file for 2.5 Integration
#Duncan Haldane 2/12/2013

import msvcrt
import sys
import numpy as np
from lib import command
from struct import *
import time
from xbee import XBee
import serial
from callbackFunc import xbee_received
import shared

DEST_ADDR = '\x20\x12'

ser = serial.Serial(shared.BS_COMPORT, shared.BS_BAUDRATE, timeout=3, rtscts=0)
xb = XBee(ser, callback=xbee_received)


def xb_send(status, type, data):
    payload = chr(status) + chr(type) + ''.join(data)
    xb.tx(dest_addr=DEST_ADDR, data=payload)


def main():
	print 'Test suite for 2.5 functions\n'
	while True:
		print '>'
		keypress = msvcrt.getch()

		if keypress == 'e':
			xb_send(0, command.WHO_AM_I, "Robot Echo")

		elif (keypress == 'q') or (ord(keypress) == 26):
		    print "Exit."
		    xb.halt()
		    ser.close()
		    sys.exit(0)


#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        xb.halt()
        ser.close()
    except IOError:
        print "Error."
        xb.halt()
        ser.close()
