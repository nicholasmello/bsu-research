from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np


class BSUArmControl:
    def __init__(self, type="vx300s", gripper=None, speed=0.05):
        """!
        Constructor for Arm Control

        @param type The type of arm, default vx300s is our robot arm
        @param gripper Gripper is "gripper" or None if not attached
        @param speed Speed of the arm in m/s
        """
        self._bot = InterbotixManipulatorXS(type, "arm", gripper_name=gripper)
        self.speed = speed
        self._x = 0.3
        self._y = 0.0
        self._z = 0.2
        self._pitch = 0.0
        self._yaw = 0.0
        self._roll = 0.0
        self._rotation = 0.0

        # Known good position to set initial position values
        self._bot.arm.set_ee_pose_components(x=self._x, y=self._y, z=self._z)

    def get__ee_transformation_matrix(self):
        return self._bot.arm.get_ee_pose_command()

    @property
    def speed(self):
        """!
        Get speed arm will travel at

        @return <float> Returns speed in m/s
        """
        return self._speed

    @speed.setter
    def speed(self, speed):
        """!
        Set speed if speed with max value of 0.1 m/s

        @param speed Speed of the arm in m/s
        """
        if speed > 0.1:
            raise ValueError("Speed above maximum value of 0.1 m/s")
        self._speed = speed

    def _update_trajectory_time(self, distance):
        """!
        Update the interbotix trajectory time based on the distance

        @param distance Distance the arm has to travel
        """
        distance = np.abs(distance)
        time = distance/self._speed
        if time > 40:
            raise ValueError("Action will take longer than 40 seconds to \
                complete")
        # Use default value of 2 if only rotation
        if distance == 0:
            time = 2
        self._bot.arm.set_trajectory_time(time)

    def go_to_home_pose(self):
        """!
        Wraps and exposes interbotix go to home pose
        """
        # Always 2 seconds for control poses
        self._bot.arm.set_trajectory_time(2)
        self._bot.arm.go_to_home_pose()

    def go_to_sleep_pose(self):
        """!
        Wraps and exposes interbotix go to sleep pose
        """
        # Always 2 seconds for control poses
        self._bot.arm.set_trajectory_time(2)
        self._bot.arm.go_to_sleep_pose()

    def set_absolute_position(self, x=0, y=0, z=0, pitch=0, yaw=None, roll=0):
        """!
        Go to specific position in space measured from center of base

        @param x X coordinate in space in m
        @param y Y coordinate in space in m
        @param z Z coordinate in space in m
        @param pitch Gripper or Array pitch
        @param yaw Gripper or Array yaw
        @param roll Gripper or Array roll
        @return <bool> If the position is valid and arm moved
        """
        # Radius as viewed from top (only x and y)
        current_radius = np.sqrt(self._x*self._x + self._y*self._y)
        desired_radius = np.sqrt(x*x + y*y)
        average_radius = (current_radius + desired_radius) / 2

        # Angle as viewed from top (only x and y)
        angle_between = np.arccos((self._x*x + self._y*y)
                                  / (current_radius * desired_radius))

        # If either radius is 0, angle_between = 0
        if np.isnan(angle_between):
            angle_between = 0

        # Adjustment for robot going the most efficient angle
        if angle_between > np.pi:
            angle_between = 2*np.pi - angle_between

        # Arc length of rotation
        length = average_radius * angle_between

        # Update speed given approx distance arm will travel
        dz = self._z-z
        dr = current_radius - desired_radius
        length = np.abs(length)
        distance = np.sqrt(dz*dz + dr*dr + length*length)
        self._update_trajectory_time(distance)

        # Attempt to set positions
        _, success = self._bot.arm.set_ee_pose_components(
            x=x, y=y, z=z,
            pitch=pitch, roll=roll, yaw=yaw)
        if success:
            self._x = x
            self._y = y
            self._z = z
            self._pitch = pitch
            if yaw is None:
                self._yaw = 0
            else:
                self._yaw = yaw
            self._roll = roll
        return success

    def adjust_absolute_position(
        self, x=None, y=None, z=None,
        pitch=None, yaw=None, roll=None
    ):
        """!
        Go to specific position in space measured from center of base

        @param x X coordinate in space in m
        @param y Y coordinate in space in m
        @param z Z coordinate in space in m
        @param pitch Gripper or Array pitch
        @param yaw Gripper or Array yaw
        @param roll Gripper or Array roll
        @return <bool> If the position is valid and arm moved
        """
        if x is None:
            x = self._x
        if y is None:
            y = self._y
        if z is None:
            z = self._z
        if pitch is None:
            pitch = self._pitch
        if roll is None:
            roll = self._roll
        return self.set_absolute_position(x, y, z, pitch, yaw, roll)

    def adjust_position(self, x=0, y=0, z=0, pitch=0, yaw=0, roll=0):
        """!
        Move to position relative to current position

        @param x X coordinate in space in m
        @param y Y coordinate in space in m
        @param z Z coordinate in space in m
        @param pitch Gripper or Array pitch
        @param yaw Gripper or Array yaw
        @param roll Gripper or Array roll
        @return <bool> If the position is valid and arm moved
        """
        distance = np.sqrt(x*x + y*y + z*z)
        self._update_trajectory_time(distance)
        success = self._bot.arm.set_ee_cartesian_trajectory(
            x=x, y=y, z=z,
            pitch=pitch, yaw=yaw, roll=roll
            )
        if success:
            self._x += x
            self._y += y
            self._z += z
            self._pitch += pitch
            self._yaw += yaw
            self._roll += roll
        return success

    def rotate(self, rotation):
        """!
        Rotate base relative to current rotation

        @param rotation Rotation amount from current position
        """
        return self.rotate_absolute(self._rotation+rotation)

    def rotate_absolute(self, rotation):
        """!
        Rotate base where 0 is straight forward

        @param rotation Rotation amount in radians between -pi and pi
        """
        radius = np.sqrt(self._x*self._x + self._y*self._y)
        distance = radius * np.abs(self._rotation-rotation)
        self._update_trajectory_time(distance)
        if not (-np.pi <= rotation and rotation <= np.pi):
            raise ValueError("Rotation outside of range -pi to pi")
        success = self._bot.arm.set_single_joint_position("waist", rotation)
        if success:
            self._rotation = rotation
            self._xpos = radius * np.cos(rotation)
            self._ypos = radius * np.cos(rotation)
        return success
