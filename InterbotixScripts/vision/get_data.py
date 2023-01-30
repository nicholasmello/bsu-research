# topic = /pc_filter/pointcloud/filtered
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import time
import numpy as np


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')


def get_xyz_points(cloud, skip_nans=True):
    points = np.array(list(point for point in pc2.read_points(cloud, skip_nans=skip_nans, field_names=("x", "y", "z"))))
    print(f"{points}\n")
    return points[:, 0], points[:, 1], points[:, 2]


def callback(data):
    global plt
    global ax
    xs, ys, zs = get_xyz_points(data)
    print(xs)
    ax.scatter(xs, ys, zs, marker='^')


def listener():
    global plt
    rospy.init_node('listener', anonymous=True)
    # sub = rospy.Subscriber("/pc_filter/pointcloud/filtered", PointCloud2, callback=callback)
    # time.sleep(5)
    # sub.unregister()
    callback(rospy.wait_for_message("/pc_filter/pointcloud/filtered", PointCloud2))
    plt.show()
    # rospy.spin()


if __name__ == '__main__':
    listener()
