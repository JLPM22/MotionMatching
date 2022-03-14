import torch
from torch.utils.data import Dataset
import numpy as np
from pathlib import Path
import feature_database as feature_db
import pose_database as pose_db
import random


class dataset_decompressor(Dataset):
    def __init__(self, feature_vectors, poses, device):
        assert feature_vectors.shape[0] == poses.shape[0]
        # Input
        self.features = torch.from_numpy(feature_vectors.astype(np.float32)).to(device)
        # Output
        self.poses = torch.from_numpy(poses.astype(np.float32)).to(device)

    def __len__(self):
        return self.features.shape[0]

    def __getitem__(self, idx):
        return self.features[idx], self.poses[idx]


# returns first the trainig set and then the test set
def get_training_and_test_dataset(filepath, filename, device):
    # Read datasets
    feature_db_path = Path(filepath) / (filename + ".mmfeatures")
    pose_db_path = Path(filepath) / (filename + ".mmpose")
    features = feature_db.feature_database(feature_db_path)
    poses = pose_db.pose_database(pose_db_path, features.is_valid)
    # Shuffle both datasets in the same way
    indices = np.arange(features.feature_vectors.shape[0])
    random.shuffle(indices)
    features.feature_vectors = features.feature_vectors[indices]
    poses.poses = poses.poses[indices]
    # Split into training and test set
    split_index = int(features.feature_vectors.shape[0] * 0.8)
    training_features = features.feature_vectors[:split_index]
    training_poses = poses.poses[:split_index]
    test_features = features.feature_vectors[split_index:]
    test_poses = poses.poses[split_index:]
    # Create datasets
    training_dataset = dataset_decompressor(training_features, training_poses, device)
    test_dataset = dataset_decompressor(test_features, test_poses, device)
    input_size = features.feature_vectors.shape[1]
    output_size = poses.poses.shape[1]
    return training_dataset, test_dataset, input_size, output_size
