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
        public FeatureVector[] Features { get; private set; }
        private float[] Mean;
        private float[] StandardDeviation;
        private NormalizeType NormType;


        public FeatureSet(FeatureVector[] features)
        {
            // Features = new NativeArray<FeatureVector>(features.Length, Allocator.Persistent);
            // for (int i = 0; i < features.Length; i++) Features[i] = features[i];
            Features = features;
        }

        public void NormalizeFeatureVector(ref FeatureVector featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");

            if (NormType == NormalizeType.Magnitude)
            {
                // Trajectory
                for (int i = 0; i < featureVector.FutureTrajectoryLocalPosition.Length; i++)
                {
                    float2 v = featureVector.FutureTrajectoryLocalPosition[i];
                    float2 unit = math.normalize(v);
                    featureVector.FutureTrajectoryLocalPosition[i] = (v - unit * Mean[0]) / StandardDeviation[0];
                }
                for (int i = 0; i < featureVector.FutureTrajectoryLocalDirection.Length; i++)
                {
                    float2 v = featureVector.FutureTrajectoryLocalDirection[i];
                    float2 unit = math.normalize(v);
                    featureVector.FutureTrajectoryLocalDirection[i] = (v - unit * Mean[1]) / StandardDeviation[1];
                }
                // Pose
                {
                    // LeftFootLocalPosition
                    float3 v = featureVector.LeftFootLocalPosition;
                    float3 unit = math.normalize(v);
                    featureVector.LeftFootLocalPosition = (v - unit * Mean[2]) / StandardDeviation[2];
                    // RightFootLocalPosition
                    v = featureVector.RightFootLocalPosition;
                    unit = math.normalize(v);
                    featureVector.RightFootLocalPosition = (v - unit * Mean[3]) / StandardDeviation[3];
                    // LeftFootLocalVelocity
                    v = featureVector.LeftFootLocalVelocity;
                    unit = math.normalize(v);
                    featureVector.LeftFootLocalVelocity = (v - unit * Mean[4]) / StandardDeviation[4];
                    // RightFootLocalVelocity
                    v = featureVector.RightFootLocalVelocity;
                    unit = math.normalize(v);
                    featureVector.RightFootLocalVelocity = (v - unit * Mean[5]) / StandardDeviation[5];
                    // HipsLocalVelocity
                    v = featureVector.HipsLocalVelocity;
                    unit = math.normalize(v);
                    featureVector.HipsLocalVelocity = (v - unit * Mean[6]) / StandardDeviation[6];
                }
            }
            else if (NormType == NormalizeType.Component)
            {
                // Trajectory
                for (int i = 0; i < featureVector.FutureTrajectoryLocalPosition.Length; i++)
                {
                    featureVector.FutureTrajectoryLocalPosition[i].x = (featureVector.FutureTrajectoryLocalPosition[i].x - Mean[0]) / StandardDeviation[0];
                    featureVector.FutureTrajectoryLocalPosition[i].y = (featureVector.FutureTrajectoryLocalPosition[i].y - Mean[1]) / StandardDeviation[1];
                }
                for (int i = 0; i < featureVector.FutureTrajectoryLocalDirection.Length; i++)
                {
                    featureVector.FutureTrajectoryLocalDirection[i].x = (featureVector.FutureTrajectoryLocalDirection[i].x - Mean[2]) / StandardDeviation[2];
                    featureVector.FutureTrajectoryLocalDirection[i].y = (featureVector.FutureTrajectoryLocalDirection[i].y - Mean[3]) / StandardDeviation[3];
                }
                // Pose
                {
                    // LeftFootLocalPosition
                    featureVector.LeftFootLocalPosition.x = (featureVector.LeftFootLocalPosition.x - Mean[4]) / StandardDeviation[4];
                    featureVector.LeftFootLocalPosition.y = (featureVector.LeftFootLocalPosition.y - Mean[5]) / StandardDeviation[5];
                    featureVector.LeftFootLocalPosition.z = (featureVector.LeftFootLocalPosition.z - Mean[6]) / StandardDeviation[6];
                    // RightFootLocalPosition
                    featureVector.RightFootLocalPosition.x = (featureVector.RightFootLocalPosition.x - Mean[7]) / StandardDeviation[7];
                    featureVector.RightFootLocalPosition.y = (featureVector.RightFootLocalPosition.y - Mean[8]) / StandardDeviation[8];
                    featureVector.RightFootLocalPosition.z = (featureVector.RightFootLocalPosition.z - Mean[9]) / StandardDeviation[9];
                    // LeftFootLocalVelocity
                    featureVector.LeftFootLocalVelocity.x = (featureVector.LeftFootLocalVelocity.x - Mean[10]) / StandardDeviation[10];
                    featureVector.LeftFootLocalVelocity.y = (featureVector.LeftFootLocalVelocity.y - Mean[11]) / StandardDeviation[11];
                    featureVector.LeftFootLocalVelocity.z = (featureVector.LeftFootLocalVelocity.z - Mean[12]) / StandardDeviation[12];
                    // RightFootLocalVelocity
                    featureVector.RightFootLocalVelocity.x = (featureVector.RightFootLocalVelocity.x - Mean[13]) / StandardDeviation[13];
                    featureVector.RightFootLocalVelocity.y = (featureVector.RightFootLocalVelocity.y - Mean[14]) / StandardDeviation[14];
                    featureVector.RightFootLocalVelocity.z = (featureVector.RightFootLocalVelocity.z - Mean[15]) / StandardDeviation[15];
                    // HipsLocalVelocity
                    featureVector.HipsLocalVelocity.x = (featureVector.HipsLocalVelocity.x - Mean[16]) / StandardDeviation[16];
                    featureVector.HipsLocalVelocity.y = (featureVector.HipsLocalVelocity.y - Mean[17]) / StandardDeviation[17];
                    featureVector.HipsLocalVelocity.z = (featureVector.HipsLocalVelocity.z - Mean[18]) / StandardDeviation[18];
                }
            }
        }

        /// <summary>
        /// Normalizes the features by subtracting mean and dividing by standard deviation
        /// </summary>
        public void NormalizeFeatures(NormalizeType normType)
        {
            NormType = normType;
            if (NormType == NormalizeType.Magnitude)
            {
                ComputeMeanAndStandardDeviationMagnitude();
            }
            else if (NormType == NormalizeType.Component)
            {
                ComputeMeanAndStandardDeviationComponent();
            }
            for (int i = 0; i < Features.Length; i++)
            {
                if (Features[i].Valid)
                {
                    NormalizeFeatureVector(ref Features[i]);
                }
            }
        }

        private void ComputeMeanAndStandardDeviationComponent()
        {
            const int nFeatures = 19;
            Mean = new float[nFeatures];
            StandardDeviation = new float[nFeatures];
            // Compute Means
            Span<int> counts = stackalloc int[nFeatures];
            for (int i = 0; i < nFeatures; i++)
            {
                if (Features[i].Valid)
                {
                    // FutureTrajectoryLocalPosition
                    Mean[0] += Features[i].FutureTrajectoryLocalPosition[0].x;
                    counts[0]++;
                    Mean[0] += Features[i].FutureTrajectoryLocalPosition[1].x;
                    counts[0]++;
                    Mean[1] += Features[i].FutureTrajectoryLocalPosition[0].y;
                    counts[1]++;
                    Mean[1] += Features[i].FutureTrajectoryLocalPosition[1].y;
                    counts[1]++;
                    // FutureTrajectoryLocalDirection
                    Mean[2] += Features[i].FutureTrajectoryLocalDirection[0].x;
                    counts[2]++;
                    Mean[2] += Features[i].FutureTrajectoryLocalDirection[1].x;
                    counts[2]++;
                    Mean[3] += Features[i].FutureTrajectoryLocalDirection[0].y;
                    counts[3]++;
                    Mean[3] += Features[i].FutureTrajectoryLocalDirection[1].y;
                    counts[3]++;
                    // LeftFootLocalPosition
                    Mean[4] += Features[i].LeftFootLocalPosition.x;
                    counts[4]++;
                    Mean[5] += Features[i].LeftFootLocalPosition.y;
                    counts[5]++;
                    Mean[6] += Features[i].LeftFootLocalPosition.z;
                    counts[6]++;
                    // RightFootLocalPosition
                    Mean[7] += Features[i].RightFootLocalPosition.x;
                    counts[7]++;
                    Mean[8] += Features[i].RightFootLocalPosition.y;
                    counts[8]++;
                    Mean[9] += Features[i].RightFootLocalPosition.z;
                    counts[9]++;
                    // LeftFootLocalVelocity
                    Mean[10] += Features[i].LeftFootLocalVelocity.x;
                    counts[10]++;
                    Mean[11] += Features[i].LeftFootLocalVelocity.y;
                    counts[11]++;
                    Mean[12] += Features[i].LeftFootLocalVelocity.z;
                    counts[12]++;
                    // RightFootLocalVelocity
                    Mean[13] += Features[i].RightFootLocalVelocity.x;
                    counts[13]++;
                    Mean[14] += Features[i].RightFootLocalVelocity.y;
                    counts[14]++;
                    Mean[15] += Features[i].RightFootLocalVelocity.z;
                    counts[15]++;
                    // HipsLocalVelocity
                    Mean[16] += Features[i].HipsLocalVelocity.x;
                    counts[16]++;
                    Mean[17] += Features[i].HipsLocalVelocity.y;
                    counts[17]++;
                    Mean[18] += Features[i].HipsLocalVelocity.z;
                    counts[18]++;
                }
            }
            for (int i = 0; i < nFeatures; i++)
            {
                Mean[i] /= counts[i];
            }
            // Compute Standard Deviations - std = sqrt(sum((x - mean)^2) / n)
            for (int i = 0; i < Features.Length; i++)
            {
                if (Features[i].Valid)
                {
                    // FutureTrajectoryLocalPosition
                    StandardDeviation[0] += (Features[i].FutureTrajectoryLocalPosition[0].x - Mean[0]) * (Features[i].FutureTrajectoryLocalPosition[0].x - Mean[0]);
                    StandardDeviation[0] += (Features[i].FutureTrajectoryLocalPosition[1].x - Mean[0]) * (Features[i].FutureTrajectoryLocalPosition[1].x - Mean[0]);
                    StandardDeviation[1] += (Features[i].FutureTrajectoryLocalPosition[0].y - Mean[1]) * (Features[i].FutureTrajectoryLocalPosition[0].y - Mean[1]);
                    StandardDeviation[1] += (Features[i].FutureTrajectoryLocalPosition[1].y - Mean[1]) * (Features[i].FutureTrajectoryLocalPosition[1].y - Mean[1]);
                    // FutureTrajectoryLocalDirection
                    StandardDeviation[2] += (Features[i].FutureTrajectoryLocalDirection[0].x - Mean[2]) * (Features[i].FutureTrajectoryLocalDirection[0].x - Mean[2]);
                    StandardDeviation[2] += (Features[i].FutureTrajectoryLocalDirection[1].x - Mean[2]) * (Features[i].FutureTrajectoryLocalDirection[1].x - Mean[2]);
                    StandardDeviation[3] += (Features[i].FutureTrajectoryLocalDirection[0].y - Mean[3]) * (Features[i].FutureTrajectoryLocalDirection[0].y - Mean[3]);
                    StandardDeviation[3] += (Features[i].FutureTrajectoryLocalDirection[1].y - Mean[3]) * (Features[i].FutureTrajectoryLocalDirection[1].y - Mean[3]);
                    // LeftFootLocalPosition
                    StandardDeviation[4] += (Features[i].LeftFootLocalPosition.x - Mean[4]) * (Features[i].LeftFootLocalPosition.x - Mean[4]);
                    StandardDeviation[5] += (Features[i].LeftFootLocalPosition.y - Mean[5]) * (Features[i].LeftFootLocalPosition.y - Mean[5]);
                    StandardDeviation[6] += (Features[i].LeftFootLocalPosition.z - Mean[6]) * (Features[i].LeftFootLocalPosition.z - Mean[6]);
                    // RightFootLocalPosition
                    StandardDeviation[7] += (Features[i].RightFootLocalPosition.x - Mean[7]) * (Features[i].RightFootLocalPosition.x - Mean[7]);
                    StandardDeviation[8] += (Features[i].RightFootLocalPosition.y - Mean[8]) * (Features[i].RightFootLocalPosition.y - Mean[8]);
                    StandardDeviation[9] += (Features[i].RightFootLocalPosition.z - Mean[9]) * (Features[i].RightFootLocalPosition.z - Mean[9]);
                    // LeftFootLocalVelocity
                    StandardDeviation[10] += (Features[i].LeftFootLocalVelocity.x - Mean[10]) * (Features[i].LeftFootLocalVelocity.x - Mean[10]);
                    StandardDeviation[11] += (Features[i].LeftFootLocalVelocity.y - Mean[11]) * (Features[i].LeftFootLocalVelocity.y - Mean[11]);
                    StandardDeviation[12] += (Features[i].LeftFootLocalVelocity.z - Mean[12]) * (Features[i].LeftFootLocalVelocity.z - Mean[12]);
                    // RightFootLocalVelocity
                    StandardDeviation[13] += (Features[i].RightFootLocalVelocity.x - Mean[13]) * (Features[i].RightFootLocalVelocity.x - Mean[13]);
                    StandardDeviation[14] += (Features[i].RightFootLocalVelocity.y - Mean[14]) * (Features[i].RightFootLocalVelocity.y - Mean[14]);
                    StandardDeviation[15] += (Features[i].RightFootLocalVelocity.z - Mean[15]) * (Features[i].RightFootLocalVelocity.z - Mean[15]);
                    // HipsLocalVelocity
                    StandardDeviation[16] += (Features[i].HipsLocalVelocity.x - Mean[16]) * (Features[i].HipsLocalVelocity.x - Mean[16]);
                    StandardDeviation[17] += (Features[i].HipsLocalVelocity.y - Mean[17]) * (Features[i].HipsLocalVelocity.y - Mean[17]);
                    StandardDeviation[18] += (Features[i].HipsLocalVelocity.z - Mean[18]) * (Features[i].HipsLocalVelocity.z - Mean[18]);
                }
            }
            for (int i = 0; i < nFeatures; i++)
            {
                StandardDeviation[i] = Mathf.Sqrt(StandardDeviation[i] / counts[i]);
            }
        }

        private void ComputeMeanAndStandardDeviationMagnitude()
        {
            const int nFeatures = 7;
            Mean = new float[nFeatures];
            StandardDeviation = new float[nFeatures];
            // Compute Means
            Span<int> counts = stackalloc int[nFeatures];
            for (int i = 0; i < Features.Length; i++)
            {
                if (Features[i].Valid)
                {
                    // FutureTrajectoryLocalPosition
                    Mean[0] += math.length(Features[i].FutureTrajectoryLocalPosition[0]);
                    counts[0]++;
                    Mean[0] += math.length(Features[i].FutureTrajectoryLocalPosition[1]);
                    counts[0]++;
                    // FutureTrajectoryLocalDirection
                    Mean[1] += math.length(Features[i].FutureTrajectoryLocalDirection[0]);
                    counts[1]++;
                    Mean[1] += math.length(Features[i].FutureTrajectoryLocalDirection[1]);
                    counts[1]++;
                    // LeftFootLocalPosition
                    Mean[2] += math.length(Features[i].LeftFootLocalPosition);
                    counts[2]++;
                    // RightFootLocalPosition
                    Mean[3] += math.length(Features[i].RightFootLocalPosition);
                    counts[3]++;
                    // LeftFootLocalVelocity
                    Mean[4] += math.length(Features[i].LeftFootLocalVelocity);
                    counts[4]++;
                    // RightFootLocalVelocity
                    Mean[5] += math.length(Features[i].RightFootLocalVelocity);
                    counts[5]++;
                    // HipsLocalVelocity
                    Mean[6] += math.length(Features[i].HipsLocalVelocity);
                    counts[6]++;
                }
            }
            for (int i = 0; i < nFeatures; i++)
            {
                Mean[i] /= counts[i];
            }
            // Compute Standard Deviations - std = sqrt(sum((x - mean)^2) / n)
            for (int i = 0; i < Features.Length; i++)
            {
                if (Features[i].Valid)
                {
                    // FutureTrajectoryLocalPosition
                    StandardDeviation[0] += (math.length(Features[i].FutureTrajectoryLocalPosition[0]) - Mean[0]) * (math.length(Features[i].FutureTrajectoryLocalPosition[0]) - Mean[0]);
                    StandardDeviation[0] += (math.length(Features[i].FutureTrajectoryLocalPosition[1]) - Mean[0]) * (math.length(Features[i].FutureTrajectoryLocalPosition[1]) - Mean[0]);
                    // FutureTrajectoryLocalDirection
                    StandardDeviation[1] += (math.length(Features[i].FutureTrajectoryLocalDirection[0]) - Mean[1]) * (math.length(Features[i].FutureTrajectoryLocalDirection[0]) - Mean[1]);
                    StandardDeviation[1] += (math.length(Features[i].FutureTrajectoryLocalDirection[1]) - Mean[1]) * (math.length(Features[i].FutureTrajectoryLocalDirection[1]) - Mean[1]);
                    // LeftFootLocalPosition
                    StandardDeviation[2] += (math.length(Features[i].LeftFootLocalPosition) - Mean[2]) * (math.length(Features[i].LeftFootLocalPosition) - Mean[2]);
                    // RightFootLocalPosition
                    StandardDeviation[3] += (math.length(Features[i].RightFootLocalPosition) - Mean[3]) * (math.length(Features[i].RightFootLocalPosition) - Mean[3]);
                    // LeftFootLocalVelocity
                    StandardDeviation[4] += (math.length(Features[i].LeftFootLocalVelocity) - Mean[4]) * (math.length(Features[i].LeftFootLocalVelocity) - Mean[4]);
                    // RightFootLocalVelocity
                    StandardDeviation[5] += (math.length(Features[i].RightFootLocalVelocity) - Mean[5]) * (math.length(Features[i].RightFootLocalVelocity) - Mean[5]);
                    // HipsLocalVelocity
                    StandardDeviation[6] += (math.length(Features[i].HipsLocalVelocity) - Mean[6]) * (math.length(Features[i].HipsLocalVelocity) - Mean[6]);
                }
            }
            for (int i = 0; i < nFeatures; i++)
            {
                StandardDeviation[i] = Mathf.Sqrt(StandardDeviation[i] / counts[i]);
            }
        }

        public enum NormalizeType
        {
            // FIXME: Magnitude normalization does not work well
            Magnitude, // Normalize the magnitude of each vector
            Component // Normalize per component
        }
    }
}