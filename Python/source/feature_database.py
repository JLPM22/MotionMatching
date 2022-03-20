import numpy as np
from pathlib import Path
import serializer_helper as sh
import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt


class feature_database:
    def __init__(self, path):
        self.import_features(path)

    def import_features(self, path):
        # Open as read binary
        with open(path, "rb") as f:
            # Read Header
            self.number_feature_vectors = sh.read_uint(f)
            self.features_dimension = sh.read_uint(f)
            self.number_features = sh.read_uint(f)
            # Read Mean & Std
            self.means = np.zeros(self.features_dimension)
            self.stds = np.zeros(self.features_dimension)
            for i in range(self.features_dimension):
                self.means[i] = sh.read_float(f)
                self.stds[i] = sh.read_float(f)
            # Read Features
            # Each element is a tuple (name, float type, number of floats, offset)
            self.features = []
            offset = 0
            for i in range(self.number_features):
                name = sh.read_string(f)
                number_floats_type = sh.read_uint(f)
                number_elements = sh.read_uint(f)
                self.features.append(
                    (name, number_floats_type, number_elements, offset)
                )
                offset += number_elements * number_floats_type
            # Read Feature Vectors
            feature_vectors = []
            self.is_valid = np.zeros(self.number_feature_vectors)
            for i in range(self.number_feature_vectors):
                isValid = sh.read_uint(f) != 0
                self.is_valid[i] = isValid
                if isValid:
                    vector = []
                    for j in range(self.features_dimension):
                        vector.append(sh.read_float(f))
                    feature_vectors.append(vector)
            self.feature_vectors = np.array(feature_vectors)

    def get_feature_vector(self, index):
        return self.feature_vectors[index]

    def get_one_feature(self, feature_index):
        assert feature_index < self.number_features
        feature = self.features[feature_index]
        return self.feature_vectors[
            :, feature[3] : feature[3] + feature[1] * feature[2]
        ]

    def denormalize(self):
        self.feature_vectors = self.feature_vectors * self.stds + self.means


def test():
    feature_database_path = Path(
        "C:/Users/JLPM/Desktop/Trabajo/TFM/MotionMatching/MotionMatchingUnity/Assets/Animations/MMData/JLData/JLData.mmfeatures"
    )
    features = feature_database(feature_database_path)
    # features.denormalize()

    # Visualize feature database
    trajectory_positions = features.get_one_feature(0)
    features_df = pd.DataFrame(trajectory_positions[:, 0:2], columns=["x", "y"])
    features_df["prediction"] = ["p1"] * features_df.shape[0]
    features_df_aux = pd.DataFrame(trajectory_positions[:, 2:4], columns=["x", "y"])
    features_df_aux["prediction"] = ["p2"] * features_df_aux.shape[0]
    features_df = features_df.append(features_df_aux)
    features_df_aux = pd.DataFrame(trajectory_positions[:, 4:6], columns=["x", "y"])
    features_df_aux["prediction"] = ["p3"] * features_df_aux.shape[0]
    features_df = features_df.append(features_df_aux)
    fig = px.scatter(features_df, x="x", y="y", opacity=0.1, facet_col="prediction")
    fig.add_shape(
        type="rect",
        x0=-1,
        y0=-1,
        x1=1,
        y1=1,
        line=dict(color="red", width=2),
        row="all",
        col="all",
    )
    fig.show()

    trajectory_directions = features.get_one_feature(1)
    features_df = pd.DataFrame(trajectory_directions[:, 0:2], columns=["x", "y"])
    features_df["prediction"] = ["p1"] * features_df.shape[0]
    features_df_aux = pd.DataFrame(trajectory_directions[:, 2:4], columns=["x", "y"])
    features_df_aux["prediction"] = ["p2"] * features_df_aux.shape[0]
    features_df = features_df.append(features_df_aux)
    features_df_aux = pd.DataFrame(trajectory_directions[:, 4:6], columns=["x", "y"])
    features_df_aux["prediction"] = ["p3"] * features_df_aux.shape[0]
    features_df = features_df.append(features_df_aux)
    fig = px.scatter(features_df, x="x", y="y", opacity=0.1, facet_col="prediction")
    fig.add_shape(
        type="rect",
        x0=-1,
        y0=-1,
        x1=1,
        y1=1,
        line=dict(color="red", width=2),
        row="all",
        col="all",
    )
    fig.show()

    left_foot_pos = features.get_one_feature(2)
    features_df = pd.DataFrame(left_foot_pos, columns=["x", "y", "z"])
    fig = px.scatter(features_df, x="x", y="z", opacity=0.1)
    fig.add_shape(
        type="rect",
        x0=-1,
        y0=-1,
        x1=1,
        y1=1,
        line=dict(color="red", width=2),
        row="all",
        col="all",
    )
    fig.show()


# test()
