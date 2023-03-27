from BSUVision import BSUVision
from BSUVision import Point
from BSUArmControl import BSUArmControl
from numpy import pi
import matplotlib.pyplot as plt


def points_to_plottable(points):
    xs = []
    ys = []
    zs = []
    for point in points:
        xs.append(point.x)
        ys.append(point.y)
        zs.append(point.z)
    return xs, ys, zs


def main():
    # Declare Arm Module
    arm = BSUArmControl()

    # Movement to get in front of camera
    arm.rotate(pi/2)
    arm.adjust_position(x=0.05, pitch=-0.2)

    # Setup vision with AR tag
    vis = BSUVision(arm.get_ee_transformation_matrix(), arm._rotation)
    arm.rotate_absolute(0)
    points = points_to_plottable(vis.get_points())
    points_before = vis._get_points()

    # Go home
    arm.go_to_home_pose()
    arm.go_to_sleep_pose()

    print("Configure plot")
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    print("Plot points")
    ax.scatter(points[0], points[1], points[2], marker='^', color='red')
    # ax.scatter(points_before[0]-0.3, points_before[1], points_before[2], color='blue')
    print("Set labels")
    ax.set_xlabel('$X$', fontsize=20, rotation=150)
    ax.set_ylabel('$Y$')
    ax.set_zlabel('$Z$', fontsize=30, rotation=60)

    print("Call show")
    plt.show()


if __name__ == "__main__":
    main()
