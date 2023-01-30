import termios
import sys
import logging
import subprocess
import os
import signal
import multiprocessing
import tty
import select
import time
from BSUArmControl import BSUArmControl


def _get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    return key


def ProcessModuleManager(process,
                         execute_function,
                         suppress_output=True,
                         log_format="%(asctime)s: %(message)s\r"):
    # Setup logging
    logging.basicConfig(format=log_format, level=logging.INFO,
                        datefmt="%H:%M:%S")
    settings = termios.tcgetattr(sys.stdin)

    # Create Process
    if suppress_output:
        ROS = subprocess.Popen(
            process,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            shell=True,
            preexec_fn=os.setsid
            )
    else:
        ROS = subprocess.Popen(
            process,
            shell=True,
            preexec_fn=os.setsid
            )
    logging.info(f'Process Created with PID {ROS.pid}')

    # Start Arm Control
    process_control = multiprocessing.Process(target=execute_function, args=())
    process_control.start()

    # Loop getting keys until finished or quit signal
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    logging.info("Press q to quit")
    while (process_control.is_alive()):
        key = _get_key(settings)
        if key == 'q':
            # Terminate Early
            process_control.terminate()
            break
        elif key == '\x03':
            logging.info("Press q to quit")
    logging.info("Shutting down...")

    # Cleanup process
    time.sleep(1)
    arm = BSUArmControl()
    arm.go_to_sleep_pose()
    os.killpg(os.getpgid(ROS.pid), signal.SIGTERM)


def ArmModuleManager(execute_function,
                     suppress_output=True,
                     log_format="%(asctime)s: %(message)s\r"):
    process = "roslaunch interbotix_xsarm_perception \
                xsarm_perception.launch \
                robot_model:=vx300s \
                use_gripper:=false"
    ProcessModuleManager(
        process=process,
        execute_function=execute_function,
        suppress_output=suppress_output,
        log_format=log_format)


def BaseModuleManager(execute_function,
                      suppress_output=True,
                      log_format="%(asctime)s: %(message)s\r"):
    process = "roslaunch turn_on_wheeltec_robot \
        turn_on_wheeltec_robot.launch"
    ProcessModuleManager(
        process=process,
        execute_function=execute_function,
        suppress_output=suppress_output,
        log_format=log_format)