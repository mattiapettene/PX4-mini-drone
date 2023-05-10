# ROS2 Creating a package

## [Create a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

To create a new project it is necessary to add it to a packege created in this way:

``` bash
ros2 pkg create <package_name> --build-type <build_type>
```

- <package_name> is the name of the package  
- <build_type>  depend on the programming language  
  - "ament_python" for Python
  - "ament_cmake" for C++  

At this point will be created a folder named <package_name> in the workspace. In this folder there is another one that has the same name of the package and contains an init file: the project script has to be added in this folder.

- **Python project**

  It is necessary to modify the setup.py file, adding the following lines, in order to compile the project:

  ``` bash
  entry_points={
          'console_scripts': [
            '<executable_name> = <package_name>.<script_name>:main',
          ],
  ```

- **C++ project**

  It in necessary to modify the CMakeList in order to compile the project:

  - Add the dependencies after the standard one included by default

    ``` cmake
    # find dependencies
    find_package(ament_cmake REQUIRED)
    # uncomment the following section in order to fill in
    # further dependencies manually.
    # find_package(<dependency1> REQUIRED)
    # find_package(<dependency1> REQUIRED)
    ```

    an example of the usual nedded packages:

    ``` cmake
    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(px4_msgs REQUIRED)
    find_package(px4_ros_com REQUIRED)
    ```

  - Add the executable files

    ``` cmake
    add_executable(<executable_file> src/<executable_file>.cpp)
    ament_target_dependencies(<executable_file> <dependencie1> <dependencie2>) 
    install(TARGETS <executable_file> DESTINATION lib/${PROJECT_NAME})
    ```
  
  - Add header files

    ``` cmake
    target_include_directories(<header_file> PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>)
    target_compile_features(<header_file> PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    ```

## Build a project

After setting the files, build the package:

``` bash
colcon build
```

Finally run the project:

``` bash
source install/local_setup.bash
ros2 run <package_name> <executable_name>
```

## Build clean

It is possible to clean the workspace deleting the file created building the project:

Install the colcon build if it not installed yet

``` bash
sudo apt install python3-colcon-clean
```

Run to clean the project

``` bash
colcon clean workspace
```
