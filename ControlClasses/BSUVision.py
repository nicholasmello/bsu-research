import rospy
import tf.transformations as tr
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from interbotix_perception_modules.armtag import InterbotixArmTagInterface
import numpy as np
from dataclasses import dataclass


@dataclass
class Point:
    x: float
    y: float
    z: float

    def __repr__(self):
        return f"\n({self.x}, {self.y}, {self.z})"

    def transformation(self, transformation_matrix):
        array = np.array([self.x, self.y, self.z, 1])
        array = np.reshape(array, (4, 1))
        transformed_point = transformation_matrix@array
        self.x = float(transformed_point[0])
        self.y = float(transformed_point[1])
        self.z = float(transformed_point[2])
        return self


class BSUVision:
    def __init__(self, T_AB, stored_value=False, init_node=False):
        if stored_value:
            # TODO: When the arm is mounted get a stored value
            raise NotImplementedError("No stored value")
            rospy.init_node('listener', anonymous=True)
            self._T_BC = None
        else:
            # AR Tag
            tag = InterbotixArmTagInterface(init_node=init_node)
            tag.find_ref_to_arm_base_transform()
            transform = tag.trans
            T_CB = self._transform_to_array(transform.transform)
            self._T_BC = tr.inverse_matrix(T_CB)
        self._T_AB = T_AB
        print("Initialized BSUVision!\n")

    def _transform_to_array(self, msg):
        p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
        q = np.array([msg.rotation.x, msg.rotation.y,
                      msg.rotation.z, msg.rotation.w])
        g = tr.quaternion_matrix(q)
        g[0:3, -1] = p
        return g

    def _get_points(self):
        cloud = rospy.wait_for_message("/pc_filter/pointcloud/filtered", PointCloud2)
        points = np.array(list(point for point in pc2.read_points(
            cloud, skip_nans=True, field_names=("x", "y", "z")
        )))
        return points[:, 0], points[:, 1], points[:, 2]

    def get_points(self):
        xs, ys, zs = self._get_points()
        points = []
        for i in range(len(xs)):
            points.append((Point(
                x=xs[i],
                y=ys[i],
                z=xs[i]
            ).transformation(self._T_BC)
            ).transformation(self._T_AB))
        return points
    
    def get_plane(self, points=None):
        if points is None:
            points = self.get_points()


if __name__ == "__main__":
    empty = np.matrix([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ])
    vis = BSUVision(empty)
    print(vis.get_points())
