import serial

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

def lettura_uwb(rl):
    mesg = rl.readline().decode("utf-8")
    mesg = mesg.replace("\r\n", "")
    tx, rx, identity, distance = mesg.split(" ") # Divido la stringa e ottengo gli argo>
    
    return int(tx), int(rx), int(identity), float(distance)

def main(args=None):
    ser = serial.Serial(
    port='/dev/ttyACM0',\
    baudrate=115200)
    
    rl = ReadLine(ser)
    [tx, rx, identity, distance] = lettura_uwb(rl)
    print('Tx: {0}, Rx: {1}, id: {2}, Range: {3}'.format(round(tx,2),round(rx,2),round(identity,2),round(distance,2)))



if __name__ == '__main__':
    main()
