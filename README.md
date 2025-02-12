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
```
$ git clone https://gitlab.com/sdurobotics/ur_rtde.git
$ cd ur_rtde
$ mkdir build
$ cd build
$ cmake -DPYTHON_BINDINGS:BOOL=OFF ..
$ make
$ sudo make install
```

If an error occurs in this library, detailed information will be output as a message by the following operations.

```
Change "DEBUG_OUTPUT" defined in "rtde.cpp" to true and rebuild.

  #define RTDE_PROTOCOL_VERSION 2
  #define DEBUG_OUTPUT false
```

For example, if the robot version does not match the library, the following message is printed (Added line breaks for readability):

```
RTDE:287: Payload size is: 1113
RTDE:304: SENDING buf containing: \O@_@timestamp,target_q,target_qd,target_qdd,target_current,target_moment,actual_q,
actual_qd,actual_current,joint_control_output,actual_TCP_pose,actual_TCP_speed,actual_TCP_force,target_TCP_pose,
target_TCP_speed,actual_digital_input_bits,joint_temperatures,actual_execution_time,robot_mode,joint_mode,safety_mode,
actual_tool_accelerometer,speed_scaling,target_speed_fraction,actual_momentum,actual_main_voltage,actual_robot_voltage,
actual_robot_current,actual_joint_voltage,actual_digital_output_bits,runtime_state,standard_analog_input0,
standard_analog_input1,standard_analog_output0,standard_analog_output1,robot_status_bits,safety_status_bits,

ft_raw_wrench,payload,payload_cog,payload_inertia,

output_int_register_2,output_int_register_12,output_int_register_13,output_int_register_14,
output_int_register_15,output_int_register_16,output_int_register_17,output_int_register_18,output_int_register_19,
output_double_register_12,output_double_register_13,output_double_register_14,output_double_register_15,
output_double_register_16,output_double_register_17,output_double_register_18,output_double_register_19, with len: 1116
RTDE:159: Done sending RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS
RTDE:332: Receiving...
RTDE:341: ControlHeader:
RTDE:342: size is: 450
RTDE:343: command is: 79
RTDE:399: Datatype:DOUBLE,VECTOR6D,VECTOR6D,VECTOR6D,VECTOR6D,VECTOR6D,VECTOR6D,
VECTOR6D,VECTOR6D,VECTOR6D,VECTOR6D,VECTOR6D,VECTOR6D,VECTOR6D,
VECTOR6D,UINT64,VECTOR6D,DOUBLE,INT32,VECTOR6INT32,INT32,
VECTOR3D,DOUBLE,DOUBLE,DOUBLE,DOUBLE,DOUBLE,
DOUBLE,VECTOR6D,UINT64,UINT32,DOUBLE,
DOUBLE,DOUBLE,DOUBLE,UINT32,UINT32,

NOT_FOUND,NOT_FOUND,NOT_FOUND,NOT_FOUND,

INT32,INT32,INT32,INT32,
INT32,INT32,INT32,INT32,INT32,
DOUBLE,DOUBLE,DOUBLE,DOUBLE,
DOUBLE,DOUBLE,DOUBLE,DOUBLE
RTDE:287: Payload size is: 0
RTDE:304: SENDING buf containing: S with len: 3
RTDE:318: Done sending RTDE_CONTROL_PACKAGE_START
RTDE:332: Receiving...
RTDE:341: ControlHeader:
RTDE:342: size is: 4
RTDE:343: command is: 83
RTDE:407: success: 0
```

In the above example of sending a command to the robot, the robot will return a data type such as "Datatype:DOUBLE,VECTOR6D". "NOT_FOUND" is returned if any field name is not supported.
In this case, comment out the send target field name in the setupRecipes function of "rtde_receive_interface.cpp" and rebuild.

An example of modifying "rtde_receive_interface.cpp" is as follows:

```
129                   "safety_status_bits",
130                   // "ft_raw_wrench",    // Supported from 5.9.0 version
131                   // "payload",          // Supported from 3.11.0 version
132                   // "payload_cog",      // Supported from 3.11.0 version
133                   // "payload_inertia",  // Supported from 3.15 version
134                   outIntReg(2),
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

If you wish to run the simulation only use as a simulation (replace the `<robot_hostname>` with simulation)

```
$ MCControlRtde -h simulation
```

Your mc_rtc configuration file (`~/.config/mc_rtc/mc_rtc.yaml`) should contain the following lines:

```
MainRobot: UR5e # or UR10
Enabled: YourController
Timestep: 0.001

# Set a LogPolicy suitable for real-time
LogPolicy : threaded

# Interface specific parameters (mc_rtde)
RTDE:
  JointSpeed: 1.05  # joint speed [rad/s]
  JointAcceleration: 1.4  # joint acceleration [rad/s^2]

  ur5e: # Name of the robot in the controller
    IP: "localhost"

  ur10:
    IP: "localhost"
```

Run the program:

```
$ MCControlRtde
```

You can also provide an additional configuration file (to swap between different network configurations easily for example):

```
$ MCControlRtde -f conf.yaml
```
