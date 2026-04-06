# mc_rtde
Interface between [Universal robots](https://www.universal-robots.com/) and [mc_rtc](https://jrl-umi3218.github.io/mc_rtc). Provides connectivity with [UR5e | UR7e](https://www.universal-robots.com/products/ur5-robot/) and [UR10](https://www.universal-robots.com/products/ur10e/) robots.

## 1. Required dependencies

- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc/)
- ur10
  - robot_module : https://github.com/isri-aist/mc_ur10
  - description : https://github.com/isri-aist/mc_ur10_description
- ur5e | ur7e
  - robot_module : https://github.com/isri-aist/mc_ur5e
  - description : https://github.com/isri-aist/mc_ur5e_description
- [ur_rtde library](https://gitlab.com/sdurobotics/ur_rtde)
- [ur_modern_driver](https://github.com/jrl-umi3218/ur_modern_driver)

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

### ur_modern_driver library

```
$ git clone https://github.com/jrl-umi3218/ur_modern_driver
$ cd ur_modern_driver
$ mkdir build
$ cd build
$ cmake ..
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

### Set memory limits

Make sure user account has sufficient memory limits.

Edit `/etc/security/limits.conf` by adding the following line :

```
USERNAME - memlock 1000000000
```

Then log-out and log-in or reboot, to confirm the new limit :

```bash
ulimit -l
```

###  The path set to the ROS library

`MCControlRtde` will not pick up ROS libraries. If you're using `mc_rtc`'s ROS plugin, create this file `/etc/ld.so.conf.d/ros.conf` with the following content:
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
MCControlRTDE options:
  --help                Display help message
  -f [ --conf ] arg     Configuration file

see etc/mc_rtc_ur.yaml for mc_rtde configuration example

$ MCControlRtde -f <mc_rtc_configuration_file.conf>
```

Where <mc_rtc_configuration_file.yaml> is based on (e.g).

`<INSTALL_PREFIX>/etc/mc_rtde/<robot>.yaml` --> `/usr/local/etc/mc_rtde/mc_rtc_ur.yaml`

Your mc_rtc configuration file (`~/.config/mc_rtc/mc_rtc.yaml`) should contain the following lines:

```yaml
MainRobot: UR5e # or UR10 / UR7e
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
    gripper:
      name: "<gripper_name>" # Name of the gripper attached to the robot
      type: robotiq # Only robotiq is supported now
      ip: "localhost" # [Optional] Robot's ip is used by default
      port: 63352 # [Optional]

  ur7e: # Name of the robot in the controller
    ip: "localhost"

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

### Gripper Integration

The gripper is loaded as an independent robot in the mc_rtc controller and controlled via a posture task. It can be attached to the main robots by `addContact`.
```CPP
robots().robot("<gripper>").posW(robots().robot("<main_robot").surfacePose("<mounting_surface>"));
addContact({"<main_robot>", "<gripper>", "<mounting_surface>", "<gripper_base>"});
```

#### Limitation

The tested Robotiq gripper does not support real-time control. To avoid flooding the gripper, the interface only sends a new command when the mc_rtc commanded position has reached steady state. As a result:
- The real gripper will lag behind the simulated gripper
- Posture task gains do not have the same effect as on other robots — the gripper moves at its own pace once a command is sent

## 5. Toubleshooting

### Known issue

It may happen that some libraries are not found due to the high priviligies given for real-time scheduling. To overcome it, please consider running the following commands :

```bash
echo "$HOME/workspace/src/catkin_data_ws/install/lib" | sudo tee -a /etc/ld.so.conf.d/mc_rtc_ros.conf
echo "$HOME/workspace/install/lib" | sudo tee -a /etc/ld.so.conf.d/mc_rtc.conf
echo "/opt/ros/humble/lib" | sudo tee -a /etc/ld.so.conf.d/ros2.conf
```

Then run the following command :

```bash
sudo ldconfig
```

### Segfault

We noticed that a segfault happens when using `rmw_cyclonedds_cpp` as RMW.

Please consider using the following one :

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### PolyScope 5
*This is tested with `PolyScope 5.25`, but `5.23` and `5.24` versions of the software may run into the same issue.*

Some of the services used to connection might be disable by default. Consider following these steps on the teach pendant to enable them.
1. Make sure you are in [Local Control](https://www.universal-robots.com/manuals/EN/HTML/SW5_25/Content/prod-usr-man/software/PolyScope/content/hamburger_menu_g5/System_remote_en.htm?Highlight=local%20control) mode.
2. Go to [Hamburger menu > Settings > Security > Services](https://www.universal-robots.com/manuals/EN/HTML/SW5_25/Content/prod-usr-man/software/PolyScope/content/hamburger_menu_g5/Security_services_en.htm) and enable these services:
   - Dashboard Server
   - Primary Client Interface
   - Secondary Client Interface
   - Real-Time Client Interfcae
   - Real-Time Data Exchange (RTDE)
