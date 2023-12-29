import re
import paho.mqtt.client as mqtt
import serial
import time 
import json

Connected = False  # global variable for the state of the connection
DEBUG = False

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


#SET OUTPUT MESSAGE
mesg = {}
hser = serial.Serial( '/dev/ttyAMA0', 921600, timeout = 0)
topic = "Tag"
rl = ReadLine(hser)


with open("config.json") as json_file:
    data = json.load(json_file)
    print (data)
    broker_addr = data["broker_adr"]
    print (broker_addr)
    broker_port = int(data["broker_port"])
    print (broker_port)


def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    
    global Connected                #Use global variable
    Connected = True



Connected = False
client = mqtt.Client()
client.max_inflight_messages_set(2000)
client.on_connect = on_connect
client.connect(broker_addr, broker_port, 60)
client.loop_start()

while Connected != True:    #Wait for connection
    time.sleep(0.1)
 
try:
    while True:
        mesg = rl.readline().decode("utf-8")
        client.publish(topic,mesg, qos=0)
        #print("published: " + mesg)


except KeyboardInterrupt:
    hser.close()
    print ('closed serial port')
    client.disconnect()
    client.loop_stop()
