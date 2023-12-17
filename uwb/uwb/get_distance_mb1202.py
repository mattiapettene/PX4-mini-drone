from smbus import SMBus
import time

# sensor I2C address
address = 0x70

while True:
    try:
        i2cbus = SMBus(1)
        i2cbus.write_byte(0x70, 0x51)
        time.sleep(0.1)  #100ms
        val = i2cbus.read_word_data(0x70, 0xe1)
        print((val >> 8) & 0xff | (val & 0xff), 'cm')
    except IOError as e:
        print(e)