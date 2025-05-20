# mc_rtde
Interface between [Universal robots](https://www.universal-robots.com/) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc). Provides connectivity with [UR5e](https://www.universal-robots.com/products/ur5-robot/) and [UR10](https://www.universal-robots.com/products/ur10e/) robots.

## 1. Required dependencies

- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
- ur10
  - robot_module : https://github.com/isri-aist/mc_ur10
  - description : https://github.com/isri-aist/mc_ur10_description
- ur5e
  - robot_module : https://github.com/isri-aist/mc_ur5e
  - description : https://github.com/isri-aist/mc_ur5e_description
- [ur_rtde library](https://gitlab.com/sdurobotics/ur_rtde)

## 2. Install dependencies

### ur_rtde library

- From PPA : 

```
sudo add-apt-repository ppa:sdurobotics/ur-rtde
sudo apt-get update
sudo apt install librtde librtde-dev
```

- From Source : 
```
$ git clone https://gitlab.com/sdurobotics/ur_rtde.git
$ cd ur_rtde
$ mkdir build
$ cd build
$ cmake -DPYTHON_BINDINGS:BOOL=OFF ..
$ make
$ sudo make install
```

## 3. Install this project

### Build instructions

```
$ git clone https://github.com/isri-aist/mc_rtde
$ cd mc_rtde
$ mkdir build
$ cd build
$ cmake ..
$ make
$ sudo make install
```

###  The path set to the ROS library

`MCControlRtde` will not pick up ROS libraries. If you're using `mc_rtc`'s ROS plugin, create a file with content: `/etc/ld.so.conf.d/ros.conf`
```
/opt/ros/${ROS_DISTRO}/lib
```
Run the following command after this change:
```
$ sudo ldconfig
```

## 4. Usage
To use the interface and connect to a real robot run

```
$ MCControlRtde --help

 MCControlRtde options:
   --help                                display help message
   -h [ --host ] arg (=ur5e)             connection host, robot name {ur5e, ur10} or
                                         "simulation"
   -f [ --conf ] arg (=/usr/local/etc/mc_rtde/mc_rtc_ur.yaml)
                                         configuration file

$ MCControlRtde -h <robot_hostname> -f <mc_rtc_configuration_file.conf>
```

Where <mc_rtc_configuration_file.yaml> is based on (e.g).

 `<INSTALL_PREFIX>/etc/mc_rtde/<robot>.yaml` --> `/usr/local/etc/mc_rtde/mc_rtc_ur.yaml`

Your mc_rtc configuration file (`~/.config/mc_rtc/mc_rtc.yaml`) should contain the following lines:

```yaml
MainRobot: UR5e # or UR10
Enabled: YourController
Timestep: 0.001

# Set a LogPolicy suitable for real-time
LogPolicy : threaded

# Interface specific parameters (mc_rtde)
RTDE:
  ControlMode: Position # Can be: Position/Velocity/Torque

  ur5e: # Name of the robot in the controller
    ip: "localhost"
    driver: "ur_rtde" # Can be: "ur_rtde" (>=CB3) or "ur_modern_driver" (<=CB2). Default: ur_rtde

  ur10: # Name of the robot in the controller
    ip: "localhost"
```

Run the program:

```bash
$ MCControlRtde
```

You can also provide an additional configuration file (to swap between different network configurations easily for example):

```bash
$ MCControlRtde -f conf.yaml
```

## Toubleshooting

* Known issue

It may happen that some libraries are not found due to the high priviligies given for real-time scheduling. To overcome it, please consider running the following files (change `user_name` by your user name (`whoami`)):

```bash
sudo echo "/home/{user_name}/workspace/src/catkin_data_ws/install/lib" >> /etc/ld.so.conf.d/mc_rtc_ros.conf
sudo echo "/home/{user_name}/workspace/install/lib" >> /etc/ld.so.conf.d/mc_rtc.conf
sudo echo "/opt/ros/humble/lib" >> /etc/ld.so.conf.d/ros2.conf
```

Then run the following command :

```bash
sudo ldconfig
```

* Segfault

We noticed that a segfault happens when using `rmw_cyclonedds_cpp` as RMW. 

Please consider using the following one :

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
