from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np
import timeit


def main():
    bot = InterbotixManipulatorXS("vx300s", "arm", gripper_name=None)
    start_time = timeit.default_timer()
    bot.arm.set_ee_pose_components(x=0.3, y=0, z=0.2, pitch=0, yaw=None, roll=0)
    print(timeit.default_timer() - start_time)
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
