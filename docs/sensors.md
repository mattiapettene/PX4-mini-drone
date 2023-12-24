# MaxBotix MB1202 I2CXL-MaxSonar-EZ0

Install the following packages:

``` bash
sudo apt-get install i2c-tools
sudo apt-get install python3-smbus
```

Set the baudrate:

``` bash
sudo nano /boot/firmware/config.txt  
```

At the end of the file insert: "dtparam=i2c1_baudrate=50000", then save and reboot.

After connecting the sensor to Raspberry Pi, get its address typing:

``` bash
sudo i2cdetect -y 1
```

By default the address should be 0x70.

Then launch the script to read the distance (change address if necessary):

``` bash
python3 uwb/uwb/get_distance_mb1202.py 
```
