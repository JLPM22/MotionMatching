using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using UnityEngine;

namespace MotionMatching
{
    // Burst-based job for linearly search the best feature vector given a query feature vector
    [BurstCompile]
    public struct LinearMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid;
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> QueryFeature;
        [ReadOnly] public NativeArray<float> FeatureWeights; // Size = FeatureSize
        [ReadOnly] public float Responsiveness;
        [ReadOnly] public float Quality;
        [ReadOnly] public int NumberFeatureVectors;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int PoseOffset;

        [WriteOnly] public NativeArray<int> BestIndex;

        public void Execute()
        {
            float min = float.MaxValue;
            int bestIndex = -1;
            for (int i = 0; i < NumberFeatureVectors; ++i)
            {
                if (Valid[i])
                {
                    int featureIndex = i * FeatureSize;
                    // Trajectory
                    float sqrDistanceTrajectory = 0.0f;
                    for (int j = 0; j < PoseOffset; ++j)
                    {
                        float diff = Features[featureIndex + j] - QueryFeature[j];
                        sqrDistanceTrajectory += diff * diff * FeatureWeights[j];
                    }
                    sqrDistanceTrajectory *= Responsiveness;
                    // Pose
                    float sqrDistance = 0.0f;
                    for (int j = PoseOffset; j < FeatureSize; ++j)
                    {
                        float diff = Features[featureIndex + j] - QueryFeature[j];
                        sqrDistance += diff * diff * FeatureWeights[j];
                    }
                    sqrDistance *= Quality;
                    sqrDistance += sqrDistanceTrajectory;
                    // Compare
                    if (sqrDistance < min)
                    {
                        min = sqrDistance;
                        bestIndex = i;
                    }
                }
            }
            BestIndex[0] = bestIndex;
        }
    }
}