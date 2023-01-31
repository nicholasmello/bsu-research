# Control Classes
## Arm Control
The default constructor creates an arm object for a vx300s arm (The one used in the lab) with gripper set to none and a default movement speed of 0.05 m/s.
```py
arm = BSUArmControl()
```
These options can be changed with kwargs however it has not been tested with a different robot.
```py
arm = BSUArmControl(type="different_robot", gripper="gripper", speed=0.03)
```
Speed can be set directly using the property
```py
arm = BSUArmControl()
arm.speed = 0.07
print(arm.speed)
```
Going directly to common home and sleep poses can be done using the go to functions
```py
arm = BSUArmControl()
arm.go_to_home_pose()
arm.go_to_sleep_pose()
```
The main way to adjust position is relative to the current position.
```py
arm = BSUArmControl()
arm.adjust_position(x=0.1, y=-0.2)
```
The arm can also be moved using absolute coordinates, made easier with an adjust absolute which uses the current position for anything that is not specified. This is different than adjust position because x=0 goes to a location where adjusting by 0 stays in the same spot.
```py
arm = BSUArmControl()
arm.adjust_absolute_position(y=0.2)
# Code below is same as home pose
arm.absolute_position(x=0, y=0, z=0, pitch=0, yaw=0, roll=0)
```
The final way to move is via rotation. You can rotate between -pi and pi and there are commands for relative and absolute just like with other forms of movement.
```py
from numpy import pi
arm = BSUArmControl()
arm.rotate(pi/2)
arm.rotate_absolute(-pi/2)
```
The script `bsu_arm_control_test.py` shows an example of arm movement and `bsu_basic_sweep.py` shows a basic sweeping motion.
## Base Control
The constructor has kwargs for speed with a default value of 0.2m/s and rospy rate (The frequency is sends commands) of 10Hz.
```py
base = BSUBaseControl()
```
Speed can be set directly using the property
```py
base = BSUBaseControl()
base.speed = 0.15
print(base.speed)
```
Shutdown is passed as a callback when the node started with rospy shuts down and it stops movement of the robot. It does not need to be called externally but is available to stop the robot.
```py
base = BSUBaseControl()
base.shutdown()
```
Movement is done by giving a distance to move and an amount to turn while moving.
```py
base = BSUBaseControl()
base.move(1, 0.5)
base.move(-1, -0.5)
```
## Module Manager
The module way provides a way to start ROS commands from python. The first argument is the function to be run while the module is running. This example from `bsu_test_with_module` shows the concept well.
```py
def main():
    arm = BSUArmControl()
    temp = arm._y
    arm.adjust_position(y=0.1)
    arm.adjust_absolute_position(y=temp)
    arm.rotate(pi/2)
    arm.adjust_position(z=0.2)
    arm.rotate(-pi/2)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()


if __name__ == "__main__":
    _ = ArmModuleManager(main)
```
In addition to ArmModuleManager there is a BaseModuleManager.
```py
BaseModuleManager(function)
```
Both module manager functions are wrappers for ProcessModuleManager which takes in a string with the command to execute.
```py
process = "roslaunch interbotix_xsarm_perception \
            xsarm_perception.launch \
            robot_model:=vx300s \
            use_gripper:=false"
ProcessModuleManager(process)
```
## Vision
Vision is still in progress
## Example Scripts
Example scripts in this folder give more examples on how to use these classes.
* bsu_arm_control_test.py
* bsu_base_control_test.py
* bsu_basic_sweep.py
* bsu_pick_place.py
* bsu_pick_place.py
* bsu_test_with_module.py