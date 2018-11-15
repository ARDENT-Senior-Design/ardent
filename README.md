# ardent

Complete robot package for ARDET

## Dependencies

Clone this package into the src of your workspace: https://github.com/dawonn/vectornav

Run the command ` chmod 777 /dev/ttyusb0` so your computer has access to the usb port

## Launch Files

```
roslaunch ardent ardent_gazebo.launch
```
will launch the basic simulation with motion to show the robot working.

## Directories

robot: Contains all information regarding gazebo robot .xml setup and any sensors, joints, etc. added

src and inlcude: library objects that handle most the softare architecture for the robot such as inverse kinematics

scripts: are for any kind of executable scripts that may need to be run to setup or operate the package

test: test executables to run with the robot

launch: launch files for specific robot operations
