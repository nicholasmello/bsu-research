from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
import time


class BSUVision:
    def __init__(self, armtag_on_startup=True, x=None, y=None, z=None):
        """!
        Constructor for Vision Control

        @param armtag_on_startup Check for armtag on creation
        @param x X coordinate of the arm in relation to the camera
        @param y Y coordinate of the arm in relation to the camera
        @param z Z coordinate of the arm in relation to the camera
        """
        self._armtag = InterbotixArmTagInterface()
        self._pcl = InterbotixPointCloudInterface()
        if armtag_on_startup:
            self.update_armtag()
        else:
            # TODO use xyz to set position
            pass

    def update_armtag(self):
        """!
        Update arm-camera relation using armtag
        """
        time.sleep(0.5)  # Hold still while taking picture
        self._armtag.find_ref_to_arm_base_transform()

    def scan_for_objects(self, sort_axis="x", num_samples=5, reverse=True):
        """!
        Scan for Objects based on current settings

        @param sort_axis Direction to sort objects
        @param num_samples Number of samples to increase accuracy of data
        @param reverse False is ascending order along axis, True is descending
        @return <bool> Success or failure
        @return final_clusters If successful, contains a list of objects
        """
        # TODO Figure out what ref_frame is
        return self._pcl.get_cluster_positions(
            ref_frame="vx300s/base_link",
            sort_axis=sort_axis,
            num_samples=num_samples,
            reverse=reverse
        )

    def surface_detection(self, xs, ys, zs):
        if max(xs)-min(xs) < max(ys)-min(ys):
            # do fit
            tmp_A = []
            tmp_b = []
            for i in range(len(zs)):
                tmp_A.append([zs[i], ys[i], 1])
                tmp_b.append(xs[i])
            b = np.matrix(tmp_b).T
            A = np.matrix(tmp_A)

            # Manual solution
            fit = (A.T * A).I * A.T * b
            errors = b - A * fit
            residual = np.linalg.norm(errors)

            print("solution: %f z + %f x + %f = y" % (fit[0], fit[1], fit[2]))

            # plot plane
            Z,Y = np.meshgrid(np.arange(min(zs), max(zs)),
                            np.arange(min(ys), max(ys)))
            X = np.zeros(Z.shape)
            for r in range(Z.shape[0]):
                for c in range(Z.shape[1]):
                    X[r,c] = fit[0] * Z[r,c] + fit[1] * Y[r,c] + fit[2]
        else:
            # do fit
            tmp_A = []
            tmp_b = []
            for i in range(len(zs)):
                tmp_A.append([zs[i], xs[i], 1])
                tmp_b.append(ys[i])
            b = np.matrix(tmp_b).T
            A = np.matrix(tmp_A)

            # Manual solution
            fit = (A.T * A).I * A.T * b
            errors = b - A * fit
            residual = np.linalg.norm(errors)

            # Best fit line equation
            print("solution: %f z + %f x + %f = y" % (fit[0], fit[1], fit[2]))

            # plot plane
            Z,X = np.meshgrid(np.arange(min(zs), max(zs)),
                            np.arange(min(xs), max(xs)))
            Y = np.zeros(Z.shape)
            for r in range(Z.shape[0]):
                for c in range(Z.shape[1]):
                    Y[r,c] = fit[0] * Z[r,c] + fit[1] * X[r,c] + fit[2]
        return X, Y, Z
