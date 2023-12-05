# Companion board

## Raspberry PI 4

### Wiring

TO DO

### Ubuntu Setup on RPi

Install raspi-config:

```bash
sudo apt update
sudo apt upgrade
sudo apt-get install raspi-config 
```

Open raspi-config:

```bash
sudo raspi-config
```

Go to the Interface Option and then click Serial Port.

* Select No to disable serial login shell.
* Select Yes to enable the serial interface.
* Click Finish and restart the RPi.

Open the firmware boot configuration file in the nano editor on RaPi:

```bash
sudo nano /boot/firmware/config.txt
```

Append the following text to the end of the file (after the last line):

```bash
enable_uart=1
dtoverlay=disable-bt
```

Then save the file and restart the RPi.
In nano you can save the file using the following sequence of keyboard shortcuts: ctrl+x, ctrl+y, Enter.

Check that the serial port is available. In this case we use the following terminal commands to list the serial devices:

```bash
cd /
ls /dev/ttyAMA0
```

The result of the command should include the RX/TX connection /dev/ttyAMA0 (note that this serial port is also available as /dev/serial0).

The RPi is now setup to work with RPi and communicate using the /dev/ttyAMA0 serial port. Note that we'll install more software in the following sections to work with MAVLink and ROS 2.

## Jtson Nano (IN PROGRESS)
