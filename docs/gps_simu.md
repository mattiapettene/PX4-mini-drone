# GPS Simulation

## Parameters selection

To perform the simulation with GPS you need first to setup the PX4 parameters in QGroundControl. You can load the parameters for the GPS simulation by opening QGroundControl and loading the parameters present in the folder **QGround_parameters**. In particular, you can load:

* [SITL_outdoor_simu.params](/QGround_parameters/SITL_outdoor_simu.params) - to perform Software in the loop simulations
* [HITL_outdoor_simu.params](/QGround_parameters/HITL_outdoor_simu.params) - to perform Hardware in the loop simulations
* [outdoor_test.params](/QGround_parameters/outdoor_test.params) - to perform experimental simulations

## Simulations

Depending of the type of simulation that you want to perform, you need to setup the enviromnet following in different ways:

* [Software in the loop](/docs/sitl.md)
* [Hardware in the loop](/docs/hitl.md)
* Experimental tests: make sure that your drone is in Ready to fly state before lauch the simulation

### C++ simulation

Start the simulation, you can choose two different control strategy:

* point target:
  
  ```bash
  cd ~/PX4-mini-drone
  ./gps/gps_cpp/run_point.sh
  ```

* velocity target:
  
  ```bash
  cd ~/PX4-mini-drone
  ./gps/gps_cpp/run_vel.sh
  ```

### Python simulation

* point target:
  
  ```bash
  cd ~/PX4-mini-drone
  ./gps/gps_cpp/run_point.sh
  ```

* velocity target:
  
  ```bash
  cd ~/PX4-mini-drone
  ./gps/gps_cpp/run_vel.sh
  ```
