import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)


def shutdown():
    twist = Twist()
    pub.publish(twist)


def main():
    rospy.init_node('turtlebot_teleop')
    rospy.on_shutdown(shutdown)
    sleep_rate = rospy.Rate(10)
    twist = Twist()
    twist.linear.x = 1
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0.2
    while not rospy.is_shutdown():
        pub.publish(twist)
        sleep_rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except:
        rospy.loginfo("GoForward node terminated.")
