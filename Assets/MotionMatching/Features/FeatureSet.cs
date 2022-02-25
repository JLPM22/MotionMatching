using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    /// <summary>
    /// Stores all features vectors of all poses for Motion Matching
    /// </summary>
    public class FeatureSet
    {
        public int NumberFeatures { get { return Features.Length; } }

        private NativeArray<FeatureVector> Features;
        private float[] Mean;
        private float[] StandardDeviation;

        public FeatureSet(FeatureVector[] features)
        {
            Features = new NativeArray<FeatureVector>(features.Length, Allocator.Persistent);
            for (int i = 0; i < features.Length; i++) Features[i] = features[i];
        }

        public FeatureVector GetFeature(int index)
        {
            return Features[index];
        }

        public NativeArray<FeatureVector> GetFeatures()
        {
            return Features;
        }

        /// <summary>
        /// Returns a copty of the feature vector with only the trajectory features normalized
        /// </summary>
        public FeatureVector NormalizeTrajectory(FeatureVector featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");

            const int nTrajectoryDimensions = 6;

            FeatureVector normalizedFeatureVector = new FeatureVector();
            normalizedFeatureVector.IsValid = featureVector.IsValid;
            normalizedFeatureVector.LeftFootLocalPosition = featureVector.LeftFootLocalPosition;
            normalizedFeatureVector.RightFootLocalPosition = featureVector.RightFootLocalPosition;
            normalizedFeatureVector.LeftFootLocalVelocity = featureVector.LeftFootLocalVelocity;
            normalizedFeatureVector.RightFootLocalVelocity = featureVector.RightFootLocalVelocity;
            normalizedFeatureVector.HipsLocalVelocity = featureVector.HipsLocalVelocity;

            int offset = 0;
            // FutureTrajectoryLocalPosition
            for (int float2Index = 0; float2Index < nTrajectoryDimensions / 2; float2Index++)
            {
                float2 value = featureVector.GetFutureTrajectoryLocalPosition(float2Index);
                value.x = (value.x - Mean[offset + float2Index]) / StandardDeviation[offset + float2Index];
                value.y = (value.y - Mean[offset + float2Index + 1]) / StandardDeviation[offset + float2Index + 1];
                normalizedFeatureVector.SetFutureTrajectoryLocalPosition(float2Index, value);
            }
            offset += nTrajectoryDimensions;
            // FutureTrajectoryLocalDirection
            for (int float2Index = 0; float2Index < nTrajectoryDimensions / 2; float2Index++)
            {
                float2 value = featureVector.GetFutureTrajectoryLocalDirection(float2Index);
                value.x = (value.x - Mean[offset + float2Index]) / StandardDeviation[offset + float2Index];
                value.y = (value.y - Mean[offset + float2Index + 1]) / StandardDeviation[offset + float2Index + 1];
                normalizedFeatureVector.SetFutureTrajectoryLocalDirection(float2Index, value);
            }
            // offset += nTrajectoryDimensions;

            return normalizedFeatureVector;
        }

        /// <summary>
        /// Returns a copy of the feature vector with normalized features
        /// </summary>
        public FeatureVector NormalizeFeatureVector(FeatureVector featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");

            const int nTrajectoryDimensions = 6;
            const int nPosAndVelDimensions = 3;

            FeatureVector normalizedFeatureVector = new FeatureVector();
            normalizedFeatureVector.IsValid = featureVector.IsValid;

            int offset = 0;
            // FutureTrajectoryLocalPosition
            for (int float2Index = 0; float2Index < nTrajectoryDimensions / 2; float2Index++)
            {
                float2 value = featureVector.GetFutureTrajectoryLocalPosition(float2Index);
                value.x = (value.x - Mean[offset + float2Index]) / StandardDeviation[offset + float2Index];
                value.y = (value.y - Mean[offset + float2Index + 1]) / StandardDeviation[offset + float2Index + 1];
                normalizedFeatureVector.SetFutureTrajectoryLocalPosition(float2Index, value);
            }
            offset += nTrajectoryDimensions;
            // FutureTrajectoryLocalDirection
            for (int float2Index = 0; float2Index < nTrajectoryDimensions / 2; float2Index++)
            {
                float2 value = featureVector.GetFutureTrajectoryLocalDirection(float2Index);
                value.x = (value.x - Mean[offset + float2Index]) / StandardDeviation[offset + float2Index];
                value.y = (value.y - Mean[offset + float2Index + 1]) / StandardDeviation[offset + float2Index + 1];
                normalizedFeatureVector.SetFutureTrajectoryLocalDirection(float2Index, value);
            }
            offset += nTrajectoryDimensions;
            // LeftFootLocalPosition
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.LeftFootLocalPosition[j];
                value = (value - Mean[offset + j]) / StandardDeviation[offset + j];
                normalizedFeatureVector.LeftFootLocalPosition[j] = value;
            }
            offset += nPosAndVelDimensions;
            // RightFootLocalPosition
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.RightFootLocalPosition[j];
                value = (value - Mean[offset + j]) / StandardDeviation[offset + j];
                normalizedFeatureVector.RightFootLocalPosition[j] = value;
            }
            offset += nPosAndVelDimensions;
            // LeftFootLocalVelocity
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.LeftFootLocalVelocity[j];
                value = (value - Mean[offset + j]) / StandardDeviation[offset + j];
                normalizedFeatureVector.LeftFootLocalVelocity[j] = value;
            }
            offset += nPosAndVelDimensions;
            // RightFootLocalVelocity
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.RightFootLocalVelocity[j];
                value = (value - Mean[offset + j]) / StandardDeviation[offset + j];
                normalizedFeatureVector.RightFootLocalVelocity[j] = value;
            }
            offset += nPosAndVelDimensions;
            // HipsLocalVelocity
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.HipsLocalVelocity[j];
                value = (value - Mean[offset + j]) / StandardDeviation[offset + j];
                normalizedFeatureVector.HipsLocalVelocity[j] = value;
            }
            // offset += nPosAndVelDimensions;

            return normalizedFeatureVector;
        }

        /// <summary>
        /// Returns a copy of the feature vector with the features before normalization
        /// </summary>
        public FeatureVector DenormalizeFeatureVector(FeatureVector featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");

            const int nTrajectoryDimensions = 6;
            const int nPosAndVelDimensions = 3;

            FeatureVector denormalizedFeatureVector = new FeatureVector();
            denormalizedFeatureVector.IsValid = featureVector.IsValid;

            int offset = 0;
            // FutureTrajectoryLocalPosition
            for (int float2Index = 0; float2Index < nTrajectoryDimensions / 2; float2Index++)
            {
                float2 value = featureVector.GetFutureTrajectoryLocalPosition(float2Index);
                value.x = value.x * StandardDeviation[offset + float2Index] + Mean[offset + float2Index];
                value.y = value.y * StandardDeviation[offset + float2Index + 1] + Mean[offset + float2Index + 1];
                denormalizedFeatureVector.SetFutureTrajectoryLocalPosition(float2Index, value);
            }
            offset += nTrajectoryDimensions;
            // FutureTrajectoryLocalDirection
            for (int float2Index = 0; float2Index < nTrajectoryDimensions / 2; float2Index++)
            {
                float2 value = featureVector.GetFutureTrajectoryLocalDirection(float2Index);
                value.x = value.x * StandardDeviation[offset + float2Index] + Mean[offset + float2Index];
                value.y = value.y * StandardDeviation[offset + float2Index + 1] + Mean[offset + float2Index + 1];
                denormalizedFeatureVector.SetFutureTrajectoryLocalDirection(float2Index, value);
            }
            offset += nTrajectoryDimensions;
            // LeftFootLocalPosition
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.LeftFootLocalPosition[j];
                value = value * StandardDeviation[offset + j] + Mean[offset + j];
                denormalizedFeatureVector.LeftFootLocalPosition[j] = value;
            }
            offset += nPosAndVelDimensions;
            // RightFootLocalPosition
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.RightFootLocalPosition[j];
                value = value * StandardDeviation[offset + j] + Mean[offset + j];
                denormalizedFeatureVector.RightFootLocalPosition[j] = value;
            }
            offset += nPosAndVelDimensions;
            // LeftFootLocalVelocity
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.LeftFootLocalVelocity[j];
                value = value * StandardDeviation[offset + j] + Mean[offset + j];
                denormalizedFeatureVector.LeftFootLocalVelocity[j] = value;
            }
            offset += nPosAndVelDimensions;
            // RightFootLocalVelocity
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.RightFootLocalVelocity[j];
                value = value * StandardDeviation[offset + j] + Mean[offset + j];
                denormalizedFeatureVector.RightFootLocalVelocity[j] = value;
            }
            offset += nPosAndVelDimensions;
            // HipsLocalVelocity
            for (int j = 0; j < nPosAndVelDimensions; j++)
            {
                float value = featureVector.HipsLocalVelocity[j];
                value = value * StandardDeviation[offset + j] + Mean[offset + j];
                denormalizedFeatureVector.HipsLocalVelocity[j] = value;
            }
            // offset += nPosAndVelDimensions;

            return denormalizedFeatureVector;
        }

        /// <summary>
        /// Normalizes the features by subtracting mean and dividing by the standard deviation
        /// </summary>
        public void NormalizeFeatures()
        {
            // Compute Mean and Standard Deviation
            ComputeMeanAndStandardDeviation();

            // Normalize all feature vectors
            for (int i = 0; i < Features.Length; i++)
            {
                if (Features[i].IsValid)
                {
                    Features[i] = NormalizeFeatureVector(Features[i]);
                }
            }
        }

        private void ComputeMeanAndStandardDeviation()
        {
            const int nFeaturesTrajectory = 2;
            const int nFeaturesPosAndVel = 5;
            const int nTrajectoryDimensions = 6;
            const int nPosAndVelDimensions = 3;
            const int nTotalDimensions = nFeaturesTrajectory * nTrajectoryDimensions + nFeaturesPosAndVel * nPosAndVelDimensions;
            // Mean for each dimension
            Mean = new float[nTotalDimensions];
            // Variance for each dimension
            Span<float> variance = stackalloc float[nTotalDimensions];
            // Standard Deviation for each dimension
            StandardDeviation = new float[nTotalDimensions];

            // Compute Means for each dimension of each feature
            Span<int> counts = stackalloc int[nTotalDimensions]; // counts is only updated in the first loop (then the values are always the same)
            int offset = 0;
            for (int i = 0; i < Features.Length; i++)
            {
                if (Features[i].IsValid)
                {
                    // Future Trajectory Local Position
                    for (int j = 0; j < nTrajectoryDimensions; j++)
                    {
                        int float2Index = j / 2;
                        int floatIndex = j % 2;
                        Mean[offset + j] += Features[i].GetFutureTrajectoryLocalPosition(float2Index)[floatIndex];
                        counts[offset + j]++;
                    }
                    offset += nTrajectoryDimensions;
                    // Future Trajectory Local Direction
                    for (int j = 0; j < nTrajectoryDimensions; j++)
                    {
                        int float2Index = j / 2;
                        int floatIndex = j % 2;
                        Mean[offset + j] += Features[i].GetFutureTrajectoryLocalDirection(float2Index)[floatIndex];
                        counts[offset + j]++;
                    }
                    offset += nTrajectoryDimensions;
                    // Left Foot Local Position
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        Mean[offset + j] += Features[i].LeftFootLocalPosition[j];
                        counts[offset + j]++;
                    }
                    offset += nPosAndVelDimensions;
                    // Right Foot Local Position
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        Mean[offset + j] += Features[i].RightFootLocalPosition[j];
                        counts[offset + j]++;
                    }
                    offset += nPosAndVelDimensions;
                    // Left Foot Local Velocity
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        Mean[offset + j] += Features[i].LeftFootLocalVelocity[j];
                        counts[offset + j]++;
                    }
                    offset += nPosAndVelDimensions;
                    // Right Foot Local Velocity
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        Mean[offset + j] += Features[i].RightFootLocalVelocity[j];
                        counts[offset + j]++;
                    }
                    offset += nPosAndVelDimensions;
                    // Hips Local Velocity
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        Mean[offset + j] += Features[i].HipsLocalVelocity[j];
                        counts[offset + j]++;
                    }
                    offset = 0;
                }
            }
            for (int i = 0; i < nTotalDimensions; i++)
            {
                Mean[i] /= counts[i];
            }

            // Compute Variance for each dimension of each feature - variance = (x - mean)^2 / n
            offset = 0;
            for (int i = 0; i < Features.Length; i++)
            {
                if (Features[i].IsValid)
                {
                    // Future Trajectory Local Position
                    for (int j = 0; j < nTrajectoryDimensions; j++)
                    {
                        int float2Index = j / 2;
                        int floatIndex = j % 2;
                        float x = Features[i].GetFutureTrajectoryLocalPosition(float2Index)[floatIndex] - Mean[offset + j];
                        variance[offset + j] += x * x;
                    }
                    offset += nTrajectoryDimensions;
                    // Future Trajectory Local Direction
                    for (int j = 0; j < nTrajectoryDimensions; j++)
                    {
                        int float2Index = j / 2;
                        int floatIndex = j % 2;
                        float x = Features[i].GetFutureTrajectoryLocalDirection(float2Index)[floatIndex] - Mean[offset + j];
                        variance[offset + j] += x * x;
                    }
                    offset += nTrajectoryDimensions;
                    // Left Foot Local Position
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        float x = Features[i].LeftFootLocalPosition[j] - Mean[offset + j];
                        variance[offset + j] += x * x;
                    }
                    offset += nPosAndVelDimensions;
                    // Right Foot Local Position
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        float x = Features[i].RightFootLocalPosition[j] - Mean[offset + j];
                        variance[offset + j] += x * x;
                    }
                    offset += nPosAndVelDimensions;
                    // Left Foot Local Velocity
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        float x = Features[i].LeftFootLocalVelocity[j] - Mean[offset + j];
                        variance[offset + j] += x * x;
                    }
                    offset += nPosAndVelDimensions;
                    // Right Foot Local Velocity
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        float x = Features[i].RightFootLocalVelocity[j] - Mean[offset + j];
                        variance[offset + j] += x * x;
                    }
                    offset += nPosAndVelDimensions;
                    // Hips Local Velocity
                    for (int j = 0; j < nPosAndVelDimensions; j++)
                    {
                        float x = Features[i].HipsLocalVelocity[j] - Mean[offset + j];
                        variance[offset + j] += x * x;
                    }
                    offset = 0;
                }
            }
            for (int i = 0; i < nTotalDimensions; i++)
            {
                variance[i] /= counts[i];
            }

            // Compute Standard Deviations of a feature as the average std across all dimensions - std = sqrt(variance)
            offset = 0;
            for (int i = 0; i < nFeaturesTrajectory; i++)
            {
                float std = 0;
                for (int j = 0; j < nTrajectoryDimensions; j++)
                {
                    std += math.sqrt(variance[offset + j]);
                }
                std /= nTrajectoryDimensions;
                Debug.Assert(std > 0, "Standard deviation is zero, feature with no variation is probably a bug");
                for (int j = 0; j < nTrajectoryDimensions; j++)
                {
                    StandardDeviation[offset + j] = std;
                }
                offset += nTrajectoryDimensions;
            }
            for (int i = 0; i < nFeaturesPosAndVel; i++)
            {
                float std = 0;
                for (int j = 0; j < nPosAndVelDimensions; j++)
                {
                    std += math.sqrt(variance[offset + j]);
                }
                std /= nPosAndVelDimensions;
                Debug.Assert(std > 0, "Standard deviation is zero, feature with no variation is probably a bug");
                for (int j = 0; j < nPosAndVelDimensions; j++)
                {
                    StandardDeviation[offset + j] = std;
                }
                offset += nPosAndVelDimensions;
            }
        }

        public void Dispose()
        {
            if (Features != null && Features.IsCreated) Features.Dispose();
        }
    }
}