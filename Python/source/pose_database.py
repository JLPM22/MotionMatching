import numpy as np
from pathlib import Path
import serializer_helper as sh
import feature_database as fd


class pose_database:
    def __init__(self, path, is_valid_array):
        self.import_poses(path, is_valid_array)

    def import_poses(self, path, is_valid_array):
        # Open as read binary
        with open(path, "rb") as f:
            # Read Clips
            self.number_clips = sh.read_uint(f)
            self.clips = []
            for i in range(self.number_clips):
                start = sh.read_uint(f)
                end = sh.read_uint(f)
                frame_time = sh.read_float(f)
                self.clips.append((start, end, frame_time))
            # Read Poses
            self.number_poses = sh.read_uint(f)
            self.number_joints = sh.read_uint(f)
            self.poses = []
            for i in range(self.number_poses):
                if is_valid_array[i]:
                    # Joint Local Positions
                    local_positions = np.zeros((self.number_joints, 3))
                    for j in range(self.number_joints):
                        local_positions[j][0] = sh.read_float(f)
                        local_positions[j][1] = sh.read_float(f)
                        local_positions[j][2] = sh.read_float(f)
                    # Joint Local Rotations
                    local_rotations = np.zeros((self.number_joints, 4))
                    for j in range(self.number_joints):
                        local_rotations[j][0] = sh.read_float(f)
                        local_rotations[j][1] = sh.read_float(f)
                        local_rotations[j][2] = sh.read_float(f)
                        local_rotations[j][3] = sh.read_float(f)
                    # Joint Velocities
                    velocities = np.zeros((self.number_joints, 3))
                    for j in range(self.number_joints):
                        velocities[j][0] = sh.read_float(f)
                        velocities[j][1] = sh.read_float(f)
                        velocities[j][2] = sh.read_float(f)
                    # Joint Angular Velocities
                    angular_velocities = np.zeros((self.number_joints, 3))
                    for j in range(self.number_joints):
                        angular_velocities[j][0] = sh.read_float(f)
                        angular_velocities[j][1] = sh.read_float(f)
                        angular_velocities[j][2] = sh.read_float(f)
                    # Save Pose
                    self.poses.append(
                        np.concatenate(
                            (
                                local_positions.flatten(),
                                local_rotations.flatten(),
                                velocities.flatten(),
                                angular_velocities.flatten(),
                            )
                        )
                    )
            self.poses = np.array(self.poses)


def test():
    feature_database_path = Path(
        "C:/Users/JLPM/Desktop/Trabajo/TFM/MotionMatching/MotionMatchingUnity/Assets/Animations/MMData/JLData/JLData.mmfeatures"
    )
    features = fd.feature_database(feature_database_path)

    pose_database_path = Path(
        "C:/Users/JLPM/Desktop/Trabajo/TFM/MotionMatching/MotionMatchingUnity/Assets/Animations/MMData/JLData/JLData.mmpose"
    )
    poses = pose_database(pose_database_path, features.is_valid)


test()
