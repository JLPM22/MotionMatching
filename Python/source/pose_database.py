import numpy as np
from pathlib import Path
import serializer_helper as sh


class pose_database:
    def __init__(self, path):
        self.import_poses(path)

    def import_poses(self, path):
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
            for i in range(self.number_poses):
                # Joint Local Positions
                self.local_positions = np.zeros((self.number_joints, 3))
                for j in range(self.number_joints):
                    self.local_positions[j][0] = sh.read_float(f)
                    self.local_positions[j][1] = sh.read_float(f)
                    self.local_positions[j][2] = sh.read_float(f)
                # Joint Local Rotations
                self.local_rotations = np.zeros((self.number_joints, 4))
                for j in range(self.number_joints):
                    self.local_rotations[j][0] = sh.read_float(f)
                    self.local_rotations[j][1] = sh.read_float(f)
                    self.local_rotations[j][2] = sh.read_float(f)
                    self.local_rotations[j][3] = sh.read_float(f)
                # Joint Velocities
                self.velocities = np.zeros((self.number_joints, 3))
                for j in range(self.number_joints):
                    self.velocities[j][0] = sh.read_float(f)
                    self.velocities[j][1] = sh.read_float(f)
                    self.velocities[j][2] = sh.read_float(f)
                # Joint Angular Velocities
                self.angular_velocities = np.zeros((self.number_joints, 3))
                for j in range(self.number_joints):
                    self.angular_velocities[j][0] = sh.read_float(f)
                    self.angular_velocities[j][1] = sh.read_float(f)
                    self.angular_velocities[j][2] = sh.read_float(f)
                # Root Displacement
                self.root_displacement = np.zeros(3)
                self.root_displacement[0] = sh.read_float(f)
                self.root_displacement[1] = sh.read_float(f)
                self.root_displacement[2] = sh.read_float(f)
                # Root Rotation Displacement
                self.root_rot_displacement = np.zeros(4)
                self.root_rot_displacement[0] = sh.read_float(f)
                self.root_rot_displacement[1] = sh.read_float(f)
                self.root_rot_displacement[2] = sh.read_float(f)
                self.root_rot_displacement[3] = sh.read_float(f)
                # Root Rotation Angular Velocity
                self.root_rot_angular_velocity = np.zeros(3)
                self.root_rot_angular_velocity[0] = sh.read_float(f)
                self.root_rot_angular_velocity[1] = sh.read_float(f)
                self.root_rot_angular_velocity[2] = sh.read_float(f)
                # Root World
                self.root_world = np.zeros(3)
                self.root_world[0] = sh.read_float(f)
                self.root_world[1] = sh.read_float(f)
                self.root_world[2] = sh.read_float(f)
                # Root World Rotation
                self.root_world_rot = np.zeros(4)
                self.root_world_rot[0] = sh.read_float(f)
                self.root_world_rot[1] = sh.read_float(f)
                self.root_world_rot[2] = sh.read_float(f)
                self.root_world_rot[3] = sh.read_float(f)


pose_database_path = Path(
    "C:/Users/JLPM/Desktop/Trabajo/TFM/MotionMatching/MotionMatchingUnity/Assets/Animations/MMData/JLData/JLData.mmpose"
)
poses = pose_database(pose_database_path)
