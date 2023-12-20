# GPS Simulation

Start the environment in gazebo and the Micro-XRCE-DDS-Agent:

```bash
cd ~/PX4-mini-drone
./sitl_environment_setup.sh
```

## C++ simulation

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

## Python simulation

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
