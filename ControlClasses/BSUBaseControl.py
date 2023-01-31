import rospy
from geometry_msgs.msg import Twist


class BSUBaseControl:
    def __init__(self, speed=0.2, rate=10.0):
        self._speed = speed
        self._pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.init_node('turtlebot_telop')
        rospy.on_shutdown(self.shutdown)
        self._rate = rate

    def move(self, distance, turn=0, forward=True):
        twist = Twist()
        twist.linear.x = self._speed
        twist.angular.z = turn

        if not forward:
            twist.linear.x = -1*twist.linear.x
        if distance < 0:
            twist.linear.x = -1*twist.linear.x

        time = abs(distance / self._speed)
        i = 0
        sleep = rospy.Rate(self._rate)
        while (i <= time*self._rate):
            self._pub.publish(twist)
            sleep.sleep()
            i = i+1

    def shutdown(self):
        self._pub.publish(Twist())

    def __del__(self):
        self.shutdown()

    @property
    def speed(self):
        return self._speed

    @speed.setter
    def speed(self, speed):
        self._speed = speed
