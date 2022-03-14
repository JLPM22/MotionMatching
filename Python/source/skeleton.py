import numpy as np
from pathlib import Path
import serializer_helper as sh


class skeleton:
    def __init__(self, path):
        self.import_poses(path)

    def import_poses(self, path):
        # Open as read binary
        with open(path, "rb") as f:
            # Read Skeleton
            self.number_joints = sh.read_uint(f)
            self.joints = []
            for i in range(self.number_joints):
                name = sh.read_string(f)
                index = sh.read_uint(f)
                parent_index = sh.read_uint(f)
                local_offset = np.zeros(3)
                local_offset[0] = sh.read_float(f)
                local_offset[1] = sh.read_float(f)
                local_offset[2] = sh.read_float(f)
                type = sh.read_uint(f)
                self.joints.append((name, index, parent_index, local_offset, type))


skeleton_path = Path(
    "C:/Users/JLPM/Desktop/Trabajo/TFM/MotionMatching/MotionMatchingUnity/Assets/Animations/MMData/JLData/JLData.mmskeleton"
)
sk = skeleton(skeleton_path)
