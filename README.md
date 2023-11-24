# ROS node for Feetech Servo Control

This repository contains two packages. They are designed to work with the [Feetech-STS-Teensy-Driver](https://github.com/alang321/Feetech-STS-Teensy-Driver). The original goal was to control the STS3032 servos in a wind tunnel model. The servos are controlled by a Teensy 4.0 with receives commands from this ROS node over serial.

## Running the Node

To run the node simply run:
```
roslaunch squirrel_servo_control control_node.launch
```

The arguments and their default values can be seen below.
- "servo_list" - default: "[1, 2, 3]" - The servo serial addresses from which feedback willl be polled.
- "serial_port" - default: "/dev/serial0" - The serial port to use on the host device for communication with the Teensy.
- "feedback_frequency" - default: "3.0" - The frequency at hich feedback is polled for each servo.

For example:
```
roslaunch squirrel_servo_control control_node.launch feedback_frequency:=5.0
```

To run the included test node, in here some test commands can be easily send to the servos to check functionality:

```
roslaunch squirrel_servo_control test.launch
```

This also launches the main control node. The optional arguments are the same as above.

## Wiring

Simply connect the Teensy running [Feetech-STS-Teensy-Driver](https://github.com/alang321/Feetech-STS-Teensy-Driver) to the selected serial port. The serial port used on the Teensy can be configured in its driver and the correct pins for the serial port on the host can be found in its documentation. The connec Rx -> Tx and Tx -> Rx.

## Nodes

This repository contains two packages that each contain one node within. They are briefly described below.

### Node 1: squirrel_servo_node
This node communicates with the Teensy servo driver. On startup it disables the torque on the servos and then waits for commands. It continuously polls all servos provided to it in the list at the specified frequency.

It subscribes to 5 topics from which it awaits commands to pass on to the teensy driver.

- "servo_set_speed" - servo_speed.msg - Set the speed at which the servo moves to a position. Typical values are in the low single digit thousands.
- "servo_set_position" - servo_position.msg - Set the position of the servo in the rane 0-4096 for 0-360 degrees.
- "servo_enable_torque" - servo_enable_torque.msg - Enable or dfisable the torque of a servo.
- "servo_set_zero_position" - servo_calibrate_zero.msg - Sets the zero position (position 2048) of the servos to the current position.
- "motor_set_speed" - motor_speed.msg - Sets the PWM frequency of an optionally connected ESC or traditional servo with PWM control. The control value is the pulse width in microseconds. The motor_id field selects the PWM pin configured in the teensy driver. 0 is both PWm pins, 1 is PWM pin 1, 2 is PWN pin 2.

It publishes the feedback it receives from the servos.

- "servo_feedback" - servo_feedback.msg - Publishes current position, speed, load in 0-1000, the supply voltage and the temperature.

### Node 2: servo_test_node
This node is just included in order for people to quickly check that everything ir working propely for their setup. some test commands can be sent easily. The currently included code is just some usage examples.


## Building

To build this package first source your ROS:
```
source /opt/ros/noetic/setup.sh
```

Then, to create a workspace and clone the required files run this:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/alang321/squirrel_servo_control
cd ..
```

Finally, to build simply run:
```
catkin_make
source devel/setup.sh
```

