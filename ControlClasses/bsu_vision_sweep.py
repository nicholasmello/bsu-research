from BSUVision import BSUVision
from BSUVision import Point
from BSUArmControl import BSUArmControl
from numpy import pi


def main():
    # Declare Arm Module
    arm = BSUArmControl()

    # Movement to get in front of camera
    arm.rotate(pi/2)
    arm.adjust_position(x=0.05, pitch=-0.2)

    # Setup vision with AR tag
    vis = BSUVision(arm.get_ee_transformation_matrix())
    arm.rotate_absolute(0)
    points = vis.get_points()
    point = points[len(points)//2]
    arm.adjust_absolute_position(x=point.x, y=point.y, z=point.z)

    # Go home
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()


if __name__ == "__main__":
    main()
