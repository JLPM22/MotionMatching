using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    // Burst-based job for linearly search the best feature vector given a query feature vector
    [BurstCompile]
    public struct LinearMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid; // TODO: If all features are valid, this will be unnecessary
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> QueryFeature;
        [ReadOnly] public NativeArray<float> FeatureWeights; // Size = FeatureSize
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int PoseOffset;
        [ReadOnly] public float CurrentDistance;

        [WriteOnly] public NativeArray<int> BestIndex;

        // I use float4 (even if the code is less readable) to let Burst vectorize the code
        // after profiling... it is way faster than using float directly
        public void Execute()
        {
            float4 min = new float4(CurrentDistance);
            int4 bestIndex = new int4(-1);
            int count4 = Valid.Length >> 2; // /4
            int lastCount = Valid.Length & 0b011; // %4
            for (int i = 0; i < count4; ++i)
            {
                int index = i << 2;
                int featureIndex = index * FeatureSize;
                // Trajectory
                float4 sqrDistance = new float4(0.0f);
                for (int j = 0; j < FeatureSize; ++j)
                {
                    float4 query = new float4(QueryFeature[j]);
                    float4 cost = new float4(Features[featureIndex + j], Features[featureIndex + j + FeatureSize], Features[featureIndex + j + 2 * FeatureSize], Features[featureIndex + j + 3 * FeatureSize]);
                    cost = cost - query;
                    sqrDistance += cost * cost * FeatureWeights[j];
                }
                // Compare
                if (math.any(sqrDistance < min)) // most of the time this will be false... (profiling: 5-10% speedup)
                {
                    if (sqrDistance.x < min.x && Valid[index]) // Checking Valid here is more performant than using it to avoid calculations... probably most of the time sqrDistance < min is false and reduces memory accesses (Valid is not used)
                    {
                        min.x = sqrDistance.x;
                        bestIndex.x = index;
                    }
                    if (sqrDistance.y < min.y && Valid[index + 1])
                    {
                        min.y = sqrDistance.y;
                        bestIndex.y = index + 1;
                    }
                    if (sqrDistance.z < min.z && Valid[index + 2])
                    {
                        min.z = sqrDistance.z;
                        bestIndex.z = index + 2;
                    }
                    if (sqrDistance.w < min.w && Valid[index + 3])
                    {
                        min.w = sqrDistance.w;
                        bestIndex.w = index + 3;
                    }
                }
            }
            const float eps = 0.000001f;
            float _min = CurrentDistance - eps;
            int _bestIndex = -1;
            if (min.x < _min) { _min = min.x; _bestIndex = bestIndex.x; }
            if (min.y < _min) { _min = min.y; _bestIndex = bestIndex.y; }
            if (min.z < _min) { _min = min.z; _bestIndex = bestIndex.z; }
            if (min.w < _min) { _min = min.w; _bestIndex = bestIndex.w; }
            // Last items (not multiple of 4)
            for (int i = 0; i < lastCount; ++i)
            {
                int index = Valid.Length - lastCount + i;
                int featureIndex = index * FeatureSize;
                float sqrDistance = 0.0f;
                for (int j = 0; j < FeatureSize; ++j)
                {
                    float diff = Features[featureIndex + j] - QueryFeature[j];
                    sqrDistance += diff * diff * FeatureWeights[j];
                }
                if (sqrDistance < _min && Valid[index])
                {
                    _min = sqrDistance;
                    _bestIndex = index;
                }
            }
            BestIndex[0] = _bestIndex;
        }
    }
}