from BSUArmControl import BSUArmControl
from numpy import pi


def main():
    arm = BSUArmControl()
    temp = arm._ypos
    arm.adjust_position(y=0.1)
    arm.adjust_absolute_position(y=temp)
    arm.rotate(pi/2)
    arm.adjust_position(z=0.2)
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()
