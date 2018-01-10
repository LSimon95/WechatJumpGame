import serial.tools.list_ports
import numpy as np

plist = list(serial.tools.list_ports.comports())

if len(plist) <= 0:
    print('no com!')
else:
    plist_0 = list(plist[0])
    serialName = plist_0[0]
    print(plist_0)
    with serial.Serial('/dev/ttyUSB0', 115200, timeout = 1) as ser:
        print(ser.name)
    
        b = bytearray(b'0')
        while True:
            i = input('please enter time:')
            if i == 'exit':
                break
            b[0] = int(i)
            ser.write(b)

    ser.close()
