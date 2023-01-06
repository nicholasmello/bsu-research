from interbotix_perception_modules.armtag import InterbotixArmTagInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
import time


class BSUVision:
    def __init__(self, use_armtag=True, x=None, y=None, z=None):
        """!
        Constructor for Vision Control

        @param use_armtag If the robot has a tag on it
        @param x X coordinate of the arm in relation to the camera
        @param y Y coordinate of the arm in relation to the camera
        @param z Z coordinate of the arm in relation to the camera
        """
        self._armtag = InterbotixArmTagInterface()
        self._pcl = InterbotixPointCloudInterface()
        if use_armtag:
            self.update_armtag()
        else:
            #TODO use xyz to set position
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
        @param reverse False is ascending order along access, True is descending
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
