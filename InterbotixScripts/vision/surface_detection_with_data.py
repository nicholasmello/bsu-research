# topic = /pc_filter/pointcloud/filtered
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import matplotlib.pyplot as plt
import numpy as np


fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')


def surface_detection(xs, ys, zs):
    # do fit
    tmp_A = []
    tmp_b = []
    for i in range(len(xs)):
        tmp_A.append([xs[i], ys[i], 1])
        tmp_b.append(zs[i])
    b = np.matrix(tmp_b).T
    A = np.matrix(tmp_A)

    # Manual solution
    fit = (A.T * A).I * A.T * b
    errors = b - A * fit
    residual = np.linalg.norm(errors)

    print("solution: %f x + %f y + %f = z" % (fit[0], fit[1], fit[2]))
    print("errors: \n", errors)
    print("residual:", residual)
    X = np.linspace(min(xs), max(xs), 100)
    Y = np.linspace(min(xs), max(xs), 100)
    X, Y = np.meshgrid(X, Y)
    Z = float(fit[0]) * X + float(fit[1]) * Y + float(fit[2])
    return X, Y, Z


def get_xyz_points(cloud, skip_nans=True):
    points = np.array(list(point for point in pc2.read_points(cloud, skip_nans=skip_nans, field_names=("x", "y", "z"))))
    print(f"{points}\n")
    return points[:, 0], points[:, 1], points[:, 2]


def apply_offset(xs_in, ys_in, zs_in, x, y, z, yaw):
    xs = xs_in
    ys = ys_in
    zs = zs_in
    return xs, ys, zs


def callback(data):
    global plt
    global ax
    xs, ys, zs = get_xyz_points(data)
    xs, ys, zs = apply_offset(xs, ys, zs, 0, 0, 0, 0)
    ax.scatter(xs, ys, zs, marker='^')
    ax.set_xlabel('$X$', fontsize=20, rotation=150)
    ax.set_ylabel('$Y$')
    ax.set_zlabel('$Z$', fontsize=30, rotation=60)
    return xs, ys, zs


def listener():
    global plt
    global ax
    rospy.init_node('listener', anonymous=True)
    # sub = rospy.Subscriber("/pc_filter/pointcloud/filtered", PointCloud2, callback=callback)
    # time.sleep(5)
    # sub.unregister()
    while (1):
        ax.clear()
        xs, ys, zs = callback(rospy.wait_for_message("/pc_filter/pointcloud/filtered", PointCloud2))
        X, Y, Z = surface_detection(xs, ys, zs)
        ax.plot_surface(X, Y, Z, color='k')
        plt.pause(1)
    # rospy.spin()


if __name__ == '__main__':
    listener()
