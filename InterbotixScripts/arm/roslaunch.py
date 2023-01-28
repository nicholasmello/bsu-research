import os
import signal
import subprocess
import time
import traceback

cmd = "roslaunch interbotix_xsarm_perception xsarm_perception.launch robot_model:=vx300s use_gripper:=false"
ros = subprocess.Popen(cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
print(ros.pid)
try:
    while True:
        line = ros.stdout.readline()
        if not line:
            break
        print(f"OUTPUT: {line.rstrip()}")
        if b'started with pid' in line.rstrip():
            print("READY TO START PROGRAM")
            break
except KeyboardInterrupt:
    print("")
    print("Interrupted: Shutting down...")

os.killpg(os.getpgid(ros.pid), signal.SIGTERM)