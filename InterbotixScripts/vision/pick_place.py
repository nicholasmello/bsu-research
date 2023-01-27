import time
from interbotix_xs_modules.arm import InterbotixManipulatorXS
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface


def main(tag=True):
    # Initialize the arm module along with the pointcloud and armtag modules
    bot = InterbotixManipulatorXS("vx300s", gripper_name=None, moving_time=1.5, accel_time=0.75)
    pcl = InterbotixPointCloudInterface()
    
    # set initial arm and gripper pose
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    
    if tag: 
        armtag = InterbotixArmTagInterface()
        # get the ArmTag pose
        bot.arm.set_ee_pose_components(y=-0.3, z=0.2)
        bot.arm.set_ee_cartesian_trajectory(pitch=1)
        bot.arm.set_ee_cartesian_trajectory(z=-0.1)
        time.sleep(0.5)
        armtag.find_ref_to_arm_base_transform()
        bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    else:
        # set the xyz coords for the arm manually
        pass

    # get the cluster positions
    # sort them from max to min 'x' position w.r.t. the 'wx200/base_link' frame
    success, clusters = pcl.get_cluster_positions(ref_frame="vx300s/base_link", sort_axis="x", reverse=True)

    # pick up all the objects and drop them in a virtual basket in front of the robot
    for cluster in clusters:
        x, y, z = cluster["position"]
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z, pitch=0.5)
        bot.arm.set_ee_pose_components(x=x, y=y, z=z+0.05, pitch=0.5)
        bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()
