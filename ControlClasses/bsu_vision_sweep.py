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
    print(vis.get_points())

    # Go home
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()


if __name__ == "__main__":
    main()
