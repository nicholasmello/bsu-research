from interbotix_xs_modules.arm import InterbotixManipulatorXS
import numpy as np


def sweep(bot, distance, height, sweep_size=0.1):
    for i in range(np.floor(distance/sweep_size)):
        bot.arm.set_trajectory_time(10.0)
        bot.adjust_position(z=-height)
        bot.arm.set_trajectory_time(5.0)
        bot.adjust_position(y=sweep_size)
        bot.arm.set_trajectory_time(10.0)
        bot.adjust_position(z=height)
        bot.arm.set_trajectory_time(5.0)
        bot.adjust_position(y=sweep_size)
        bot.arm.set_trajectory_time(2.0)


def main():
    # Create Bot
    bot = InterbotixManipulatorXS("vx300s", "arm", gripper_name=None)

    # Get into starting position
    bot.arm.set_ee_pose_components(x=0.3, z=0.2)
    bot.arm.set_single_joint_position("waist", np.pi/2.0)
    bot.arm.set_ee_cartesian_trajectory(pitch=-0.13)
    bot.arm.set_ee_cartesian_trajectory(z=0.35)
    bot.arm.set_trajectory_time(10.0)
    bot.arm.set_ee_cartesian_trajectory(y=-0.3)

    # Sweep 0.3m by 0.3m area
    sweep(bot, distance=0.3, height=0.3)
    bot.arm.set_trajectory_time(8.0)

    # Return to Home/Sleep
    bot.arm.go_to_home_pose()
    bot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()
