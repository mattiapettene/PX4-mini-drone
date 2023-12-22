import serial
from time import sleep


ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
sleep(0.5)
ser.write(('stop \n').encode())
ser.reset_input_buffer()
ser.write(('respf 4 2400 100 \n').encode())

    
        
while True:
    lineser = ser.read_until(b"\n").decode('utf-8')
    lines_a = lineser.split('LAoA_deg":')
    lines_r = lineser.split('D_cm":')

    if len(lines_a) > 1 and len(lines_r) > 1:
        aoa = float(lines_a[1].split(',')[0])
        range = float(lines_r[1].split(',')[0])
        print('aoa is: {0}, range is: {1}'.format(round(aoa,2), round(range,2)))
    
