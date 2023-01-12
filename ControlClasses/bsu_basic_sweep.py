from BSUArmControl import BSUArmControl
import numpy as np


def sweep(bot, distance, height, sweep_size=0.1):
    for i in range(int(np.floor(distance/sweep_size))):
        bot.adjust_position(z=-height)
        bot.adjust_position(y=sweep_size)
        bot.adjust_position(z=height)
        bot.adjust_position(y=sweep_size)


def main():
    # Create Bot
    bot = BSUArmControl()

    # Get into starting position
    bot.speed = 0.1
    bot.rotate(np.pi/2.0)
    bot.adjust_position(pitch=-0.13)
    bot.adjust_position(x=0.1, y=-0.3, z=0.35)

    # Sweep 0.3m by 0.3m area
    bot.speed = 0.01
    sweep(bot, distance=0.3, height=0.3)

    # Return to Home/Sleep
    bot.go_to_home_pose()
    bot.go_to_sleep_pose()


if __name__ == '__main__':
    main()
