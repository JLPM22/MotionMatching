using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    public static class BVHConsts
    {
        public static readonly int LargeBVHSize = 64;
        public static readonly int SmallBVHSize = 16;
    }

    // AABB 2-layer BVH Acceleration Structure
    [BurstCompile]
    public struct BVHMotionMatchingComputeBounds : IJob
    {
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int NumberBoundingBoxLarge;
        [ReadOnly] public int NumberBoundingBoxSmall;

        public NativeArray<float> LargeBoundingBoxMin; // Size = NumberBoundingBoxLarge x FeatureSize
        public NativeArray<float> LargeBoundingBoxMax; // Size = NumberBoundingBoxLarge x FeatureSize
        public NativeArray<float> SmallBoundingBoxMin; // Size = NumberBoundingBoxSmall x FeatureSize
        public NativeArray<float> SmallBoundingBoxMax; // Size = NumberBoundingBoxSmall x FeatureSize

        public void Execute()
        {
            int LargeBoxSize = BVHConsts.LargeBVHSize;
            int SmallBoxSize = BVHConsts.SmallBVHSize;

            int numberFrames = (int)(Features.Length / FeatureSize);

            // Initialize
            for (int i = 0; i < LargeBoundingBoxMin.Length; i++) LargeBoundingBoxMin[i] = float.MaxValue;
            for (int i = 0; i < LargeBoundingBoxMax.Length; i++) LargeBoundingBoxMax[i] = float.MinValue;
            for (int i = 0; i < SmallBoundingBoxMin.Length; i++) SmallBoundingBoxMin[i] = float.MaxValue;
            for (int i = 0; i < SmallBoundingBoxMax.Length; i++) SmallBoundingBoxMax[i] = float.MinValue;

            for (int i = 0; i < numberFrames; ++i)
            {
                int iSmall = i / SmallBoxSize;
                int iSmallIndex = iSmall * FeatureSize;
                int iLarge = i / LargeBoxSize;
                int iLargeIndex = iLarge * FeatureSize;

                for (int j = 0; j < FeatureSize; ++j)
                {
                    float feature = Features[i * FeatureSize + j];
                    LargeBoundingBoxMin[iLargeIndex + j] = math.min(LargeBoundingBoxMin[iLargeIndex + j], feature);
                    LargeBoundingBoxMax[iLargeIndex + j] = math.max(LargeBoundingBoxMax[iLargeIndex + j], feature);
                    SmallBoundingBoxMin[iSmallIndex + j] = math.min(SmallBoundingBoxMin[iSmallIndex + j], feature);
                    SmallBoundingBoxMax[iSmallIndex + j] = math.max(SmallBoundingBoxMax[iSmallIndex + j], feature);
                }
            }
        }
    }

    // Burst-based job for linearly search the best feature vector given a query feature vector
    [BurstCompile]
    public struct BVHMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid; // TODO: If all features are valid, this will be unnecessary
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> QueryFeature;
        [ReadOnly] public NativeArray<float> FeatureWeights; // Size = FeatureSize
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int PoseOffset;
        [ReadOnly] public float CurrentDistance;
        // BVH
        [ReadOnly] public NativeArray<float> LargeBoundingBoxMin; // Size = NumberBoundingBoxLarge x FeatureSize
        [ReadOnly] public NativeArray<float> LargeBoundingBoxMax; // Size = NumberBoundingBoxLarge x FeatureSize
        [ReadOnly] public NativeArray<float> SmallBoundingBoxMin; // Size = NumberBoundingBoxSmall x FeatureSize
        [ReadOnly] public NativeArray<float> SmallBoundingBoxMax; // Size = NumberBoundingBoxSmall x FeatureSize

        [WriteOnly] public NativeArray<int> BestIndex;

        public void Execute()
        {
            int LargeBoxSize = BVHConsts.LargeBVHSize;
            int SmallBoxSize = BVHConsts.SmallBVHSize;

            float min = CurrentDistance;
            int bestIndex = -1;
            const int startIndex = 0;
            int endIndex = Valid.Length;
            int i = startIndex;
            while (i < endIndex)
            {
                // Current and next large box
                int iLarge = i / LargeBoxSize;
                int iLargeIndex = iLarge * FeatureSize;
                int iLargeNext = (iLarge + 1) * LargeBoxSize;

                // Find distance to box
                float currentCost = 0.0f;
                for (int j = 0; j < FeatureSize; ++j)
                {
                    float query = QueryFeature[j];
                    float cost = query - math.clamp(query, LargeBoundingBoxMin[iLargeIndex + j], LargeBoundingBoxMax[iLargeIndex + j]);
                    currentCost += cost * cost * FeatureWeights[j];
                    if (currentCost >= min)
                    {
                        break;
                    }
                }

                // If distance is already greater... next box
                if (currentCost >= min)
                {
                    i = iLargeNext;
                    continue;
                }

                // Search small box
                while (i < iLargeNext && i < endIndex)
                {
                    // Current and next small box
                    int iSmall = i / SmallBoxSize;
                    int iSmallIndex = iSmall * FeatureSize;
                    int iSmallNext = (iSmall + 1) * SmallBoxSize;

                    // Find distance to box
                    currentCost = 0.0f;
                    for (int j = 0; j < FeatureSize; ++j)
                    {
                        float query = QueryFeature[j];
                        float cost = query - math.clamp(query, SmallBoundingBoxMin[iSmallIndex + j], SmallBoundingBoxMax[iSmallIndex + j]);
                        currentCost += cost * cost * FeatureWeights[j];
                        if (currentCost >= min)
                        {
                            break;
                        }
                    }

                    // If distance is already greater... next box
                    if (currentCost >= min)
                    {
                        i = iSmallNext;
                        continue;
                    }

                    // Search inside small box
                    while (i < iSmallNext && i < endIndex)
                    {
                        // Skip non-valid
                        if (!Valid[i])
                        {
                            i += 1;
                            continue;
                        }

                        // Test all frames
                        currentCost = 0.0f;
                        for (int j = 0; j < FeatureSize; ++j)
                        {
                            float query = QueryFeature[j];
                            float cost = query - Features[i * FeatureSize + j];
                            currentCost += cost * cost * FeatureWeights[j];
                            if (currentCost >= min)
                            {
                                break;
                            }
                        }

                        // If cost is lower than best... update
                        if (currentCost < min)
                        {
                            bestIndex = i;
                            min = currentCost;
                        }

                        i += 1;
                    }
                }
            }
            BestIndex[0] = bestIndex;
        }
    }
}