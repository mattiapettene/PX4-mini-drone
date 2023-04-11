# Hardware in the loop

With Hardware-in-the-Loop (HITL) simulation the normal PX4 firmware is run on real hardware.

## Settung up HITL

1. Connect the autopilot directly to QGroundControl via USB
2. Enable HITL Mode
   1. Open Setup > Safety section.
   2. Enable HITL mode by selecting Enabled from the HITL Enabled list
3. Select Airframe
   1. Open Setup > Airframes
   2. Select a compatible airframe you want to test. Then click Apply and Restart on top-right of the Airframe Setup page.

Once configuration is complete, close QGroundControl and disconnect the flight controller hardware from the computer.

## MicroXRCE-DDS

When working with real hardware, the setup depends on the hardware, OS, and channel.

### Setup the client

The XRCE-DDS client module is included by default in all firmware and the simulator. This must be started with appropriate settings for the communication channel that you wish to use to communicate with the agent.

* XRCE_DDS_CFG: Set the port to connect on, such as TELEM2, Ethernet, or Wifi.
* Many ports are already have a default configuration. To use these ports you must first disable the existing configuration.
* Once set, you may need to reboot PX4 for the parameters to take effect. They will then persist through subsequent reboots.