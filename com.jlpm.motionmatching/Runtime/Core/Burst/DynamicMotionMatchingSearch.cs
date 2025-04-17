using System.Diagnostics;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace MotionMatching
{
    [System.Serializable]
    public struct DynamicAccelerationConsts
    {
        public float PercentageThreshold; // [0.0f, 1.0f]
        public int MinimumStepSize;
        public int LocalSearchRadius;

        public DynamicAccelerationConsts(float percentageThreshold, int minimumStepSize, int localSearchRadius)
        {
            PercentageThreshold = percentageThreshold;
            MinimumStepSize = minimumStepSize;
            LocalSearchRadius = localSearchRadius;
        }
    }

    [BurstCompile]
    public struct DynamicAccelerationComputeAdaptativeIndices : IJob
    {
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<bool> Valid;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int PoseOffset;
        [ReadOnly] public int FeatureStaticSize;
        [ReadOnly] public DynamicAccelerationConsts DynamicAccelerationConsts;

        [WriteOnly] public NativeList<int> AdaptativeIndices;

        public void Execute()
        {
            float PercentageThreshold = DynamicAccelerationConsts.PercentageThreshold;
            int MinimumStepSize = DynamicAccelerationConsts.MinimumStepSize;
            int numberFrames = (int)(Features.Length / FeatureSize);

            // Compute distribution of each feature to find the adaptative threshold
            NativeArray<(float, float)> minMaxRange = new(FeatureSize, Allocator.Temp);
            bool firstFrame = true;
            for (int i = 0; i < numberFrames; ++i)
            {
                if (!Valid[i])
                {
                    continue;
                }
                for (int j = 0; j < FeatureSize; ++j)
                {
                    float feature = Features[i * FeatureSize + j];
                    if (firstFrame)
                    {
                        minMaxRange[j] = (feature, feature);
                        firstFrame = false;
                    }
                    else
                    {
                        minMaxRange[j] = (math.min(minMaxRange[j].Item1, feature), math.max(minMaxRange[j].Item2, feature));
                    }
                }
            }
            NativeArray<float> relativeThresholds = new(FeatureSize, Allocator.Temp);
            for (int j = 0; j < FeatureSize; ++j)
            {
                float min = minMaxRange[j].Item1;
                float max = minMaxRange[j].Item2;
                relativeThresholds[j] = math.abs(max - min) * PercentageThreshold;
            }

            // Compute the adaptative indices
            NativeArray<float> lastFrame = new(FeatureSize, Allocator.Temp);
            firstFrame = true;
            for (int i = 0; i < numberFrames; i += MinimumStepSize)
            {
                if (!Valid[i])
                {
                    continue;
                }

                bool isIncluded = false;
                for (int j = 0; j < FeatureSize; ++j)
                {
                    float feature = Features[i * FeatureSize + j];
                    if (firstFrame || math.abs(feature - lastFrame[j]) > relativeThresholds[j])
                    {
                        isIncluded = true;
                        firstFrame = false;
                        break;
                    }
                }

                if (isIncluded)
                {
                    AdaptativeIndices.Add(i);
                    for (int j = 0; j < FeatureSize; ++j)
                    {
                        lastFrame[j] = Features[i * FeatureSize + j];
                    }
                }
            }
            // Clean up
            minMaxRange.Dispose();
            relativeThresholds.Dispose();
            lastFrame.Dispose();
        }
    }
}