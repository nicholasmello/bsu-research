from BSUArmControl import BSUArmControl
from BSUVision import BSUVision
import numpy as np


def main():
    # Initialize the arm module along with the pointcloud and armtag modules
    arm = BSUArmControl(speed=0.1)
    vision = BSUVision(armtag_on_startup=False)
    print(vision._pcl.get_filepath())

    arm.rotate(-np.pi/2-0.2)
    arm.adjust_position(x=0.05, pitch=1, z=-0.1)
    vision.update_armtag()
    arm.go_to_home_pose()

    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
    success, clusters = vision.scan_for_objects()

    # pick up all the objects and drop them in a virtual basket in front of the robot
    for cluster in clusters:
        x, y, z = cluster["position"]
        arm.set_absolute_position(x=x, y=y, z=z+0.05, pitch=0.5)
        arm.set_absolute_position(x=x, y=y, z=z, pitch=0.5)
        arm.set_absolute_position(x=x, y=y, z=z+0.05, pitch=0.5)
        arm.set_absolute_position(x=0.3, z=0.2)
    arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()
