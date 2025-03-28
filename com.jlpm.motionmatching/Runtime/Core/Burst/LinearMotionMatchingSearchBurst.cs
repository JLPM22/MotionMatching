using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    // TODO: review... with new dynamic features some things must be changed
    //// Burst-based job for linearly search the best feature vector given a query feature vector
    //[BurstCompile]
    //public struct LinearMotionMatchingSearchBurst : IJob
    //{
    //    [ReadOnly] public NativeArray<bool> Valid; // TODO: If all features are valid, this will be unnecessary
    //    [ReadOnly] public NativeArray<bool> TagMask; // TODO: convert to a bitmask to optimize memory
    //    [ReadOnly] public NativeArray<float> Features;
    //    [ReadOnly] public NativeArray<float> QueryFeature;
    //    [ReadOnly] public NativeArray<float> FeatureWeights; // Size = FeatureSize
    //    [ReadOnly] public int FeatureSize;
    //    [ReadOnly] public int PoseOffset;
    //    [ReadOnly] public float CurrentDistance;

    //    [WriteOnly] public NativeArray<int> BestIndex;

    //    // I use float4 (even if the code is less readable) to let Burst vectorize the code
    //    // after profiling... it is way faster than using float directly
    //    public void Execute()
    //    {
    //        float4 min = new float4(CurrentDistance);
    //        int4 bestIndex = new int4(-1);
    //        int count4 = Valid.Length >> 2; // /4
    //        int lastCount = Valid.Length & 0b011; // %4
    //        for (int i = 0; i < count4; ++i)
    //        {
    //            int index = i << 2;
    //            int featureIndex = index * FeatureSize;
    //            // Trajectory
    //            float4 sqrDistance = new float4(0.0f);
    //            for (int j = 0; j < FeatureSize; ++j)
    //            {
    //                float4 query = new float4(QueryFeature[j]);
    //                float4 cost = new float4(Features[featureIndex + j], Features[featureIndex + j + FeatureSize], Features[featureIndex + j + 2 * FeatureSize], Features[featureIndex + j + 3 * FeatureSize]);
    //                cost = cost - query;
    //                sqrDistance += cost * cost * FeatureWeights[j];
    //            }
    //            // Compare
    //            if (math.any(sqrDistance < min)) // most of the time this will be false... (profiling: 5-10% speedup)
    //            {
    //                if (sqrDistance.x < min.x && Valid[index] && TagMask[index]) // Checking Valid here is more performant than using it to avoid calculations... probably most of the time sqrDistance < min is false and reduces memory accesses (Valid is not used)
    //                {
    //                    min.x = sqrDistance.x;
    //                    bestIndex.x = index;
    //                }
    //                if (sqrDistance.y < min.y && Valid[index + 1] && TagMask[index + 1])
    //                {
    //                    min.y = sqrDistance.y;
    //                    bestIndex.y = index + 1;
    //                }
    //                if (sqrDistance.z < min.z && Valid[index + 2] && TagMask[index + 1])
    //                {
    //                    min.z = sqrDistance.z;
    //                    bestIndex.z = index + 2;
    //                }
    //                if (sqrDistance.w < min.w && Valid[index + 3] && TagMask[index + 1])
    //                {
    //                    min.w = sqrDistance.w;
    //                    bestIndex.w = index + 3;
    //                }
    //            }
    //        }
    //        const float eps = 0.000001f;
    //        float _min = CurrentDistance - eps;
    //        int _bestIndex = -1;
    //        if (min.x < _min) { _min = min.x; _bestIndex = bestIndex.x; }
    //        if (min.y < _min) { _min = min.y; _bestIndex = bestIndex.y; }
    //        if (min.z < _min) { _min = min.z; _bestIndex = bestIndex.z; }
    //        if (min.w < _min) { _min = min.w; _bestIndex = bestIndex.w; }
    //        // Last items (not multiple of 4)
    //        for (int i = 0; i < lastCount; ++i)
    //        {
    //            int index = Valid.Length - lastCount + i;
    //            int featureIndex = index * FeatureSize;
    //            float sqrDistance = 0.0f;
    //            for (int j = 0; j < FeatureSize; ++j)
    //            {
    //                float diff = Features[featureIndex + j] - QueryFeature[j];
    //                sqrDistance += diff * diff * FeatureWeights[j];
    //            }
    //            if (sqrDistance < _min && Valid[index] && TagMask[index])
    //            {
    //                _min = sqrDistance;
    //                _bestIndex = index;
    //            }
    //        }
    //        BestIndex[0] = _bestIndex;
    //    }
    //}

    [BurstCompile]
    public struct LinearMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid;
        [ReadOnly] public NativeArray<bool> TagMask;
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> QueryFeature;
        [ReadOnly] public NativeArray<float> FeatureWeights;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int FeatureStaticSize;
        [ReadOnly] public float CurrentDistance;

        [WriteOnly] public NativeArray<int> BestIndex;
        [WriteOnly] public NativeArray<float> Distances;

        public void Execute()
        {
            float minDistance = CurrentDistance;
            int bestIndex = -1;

            for (int i = 0; i < Valid.Length; ++i)
            {
                if (Valid[i] && TagMask[i])
                {
                    float sqrDistance = 0.0f;
                    int featureIndex = i * FeatureSize;

                    for (int j = 0; j < FeatureStaticSize; ++j)
                    {
                        float diff = Features[featureIndex + j] - QueryFeature[j];
                        sqrDistance += diff * diff * FeatureWeights[j];
                    }

                    Distances[i] = sqrDistance;
                    if (sqrDistance < minDistance)
                    {
                        minDistance = sqrDistance;
                        bestIndex = i;
                    }
                }
            }

            BestIndex[0] = bestIndex;
        }
    }

    [BurstCompile]
    public struct CrowdMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid;
        [ReadOnly] public NativeArray<bool> TagMask;
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> FeatureWeights;
        [ReadOnly] public float CrowdThreshold;
        [ReadOnly] public float CrowdSecondTrajectoryWeight;
        [ReadOnly] public float CrowdThirdTrajectoryWeight;
        [ReadOnly] public NativeArray<float> Mean;
        [ReadOnly] public NativeArray<float> Std;
        [ReadOnly] public NativeArray<(float2, float)> Obstacles;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int FeatureStaticSize;

        public NativeArray<int> BestIndex;
        public NativeArray<float> DebugCrowdDistance;
        public NativeArray<float> Distances;

        // Debug Visuals
        [WriteOnly] public NativeArray<float3> PointsOnEllipse;
        [WriteOnly] public NativeArray<float3> PointsOnObstacle;
        [WriteOnly] public NativeArray<float> ObstacleDistance;
        [WriteOnly] public NativeArray<float> ObstaclePenalization;
        public NativeArray<int> NumberDebugPoints;
        [ReadOnly] public bool IsDebug;
        [ReadOnly] public int DebugIndex;

        private float DistanceFunction(float distance, float threshold)
        {
            if (distance > threshold)
            {
                return 0.0f;
            }
            return -math.pow((threshold - distance), 4.0f) * math.log(math.max(distance / threshold, 1e-3f));
        }

        private float ComputePenalization(float2 pos, float2 primaryAxisUnit, float2 secondaryAxisUnit, float2 ellipse, int obstacle, float penalizationFactor, bool saveDebug)
        {
            const int maxIterationsRootFinder = 5;

            float distance = UtilitiesBurst.DistanceToEllipse(pos, primaryAxisUnit, secondaryAxisUnit, ellipse, Obstacles[obstacle].Item1, out float2 closest, maxIterationsRootFinder);
            distance = math.max(distance - Obstacles[obstacle].Item2, 1e-9f);
            float penalization = DistanceFunction(distance, CrowdThreshold) * penalizationFactor;
            // DEBUG
            if (saveDebug && distance < CrowdThreshold)
            {
                PointsOnEllipse[NumberDebugPoints[0]] = new float3(closest.x, 0.0f, closest.y);
                PointsOnObstacle[NumberDebugPoints[0]] = new float3(Obstacles[obstacle].Item1.x, 0.0f, Obstacles[obstacle].Item1.y);
                ObstacleDistance[NumberDebugPoints[0]] = distance;
                ObstaclePenalization[NumberDebugPoints[0]] = penalization;
                NumberDebugPoints[0] += 1;
            }
            return penalization;
        }

        private float FeatureCheck(int i, float minDistance, bool saveDebug)
        {
            int featureIndex = i * FeatureSize;
            float sqrDistance = Distances[i];
            float auxDebugTrajectory = sqrDistance;

            // HARDCODED: crowd forces
            float2 pos1 = new(Features[featureIndex + 0] * Std[0] + Mean[0],
                              Features[featureIndex + 1] * Std[1] + Mean[1]);
            float2 pos2 = new(Features[featureIndex + 2] * Std[2] + Mean[2],
                              Features[featureIndex + 3] * Std[3] + Mean[3]);
            float2 pos3 = new(Features[featureIndex + 4] * Std[4] + Mean[4],
                              Features[featureIndex + 5] * Std[5] + Mean[5]);

            float4 ellipse1 = new(Features[featureIndex + FeatureStaticSize + 0], Features[featureIndex + FeatureStaticSize + 1],
                                  Features[featureIndex + FeatureStaticSize + 2], Features[featureIndex + FeatureStaticSize + 3]);
            float4 ellipse2 = new(Features[featureIndex + FeatureStaticSize + 4], Features[featureIndex + FeatureStaticSize + 5],
                                  Features[featureIndex + FeatureStaticSize + 6], Features[featureIndex + FeatureStaticSize + 7]);
            float4 ellipse3 = new(Features[featureIndex + FeatureStaticSize + 8], Features[featureIndex + FeatureStaticSize + 9],
                                  Features[featureIndex + FeatureStaticSize + 10], Features[featureIndex + FeatureStaticSize + 11]);

            float2 primaryAxisUnit1 = new(ellipse1.z, ellipse1.w);
            float2 primaryAxisUnit2 = new(ellipse2.z, ellipse2.w);
            float2 primaryAxisUnit3 = new(ellipse3.z, ellipse3.w);

            float2 secondaryAxisUnit1 = new(-primaryAxisUnit1.y, primaryAxisUnit1.x);
            float2 secondaryAxisUnit2 = new(-primaryAxisUnit2.y, primaryAxisUnit2.x);
            float2 secondaryAxisUnit3 = new(-primaryAxisUnit3.y, primaryAxisUnit3.x);

            float debugTotalCrowdDistance = 0.0f;
            for (int obstacle = 0; obstacle < Obstacles.Length; obstacle++)
            {
                float penalization1 = ComputePenalization(pos1, primaryAxisUnit1, secondaryAxisUnit1, ellipse1.xy, obstacle, 1.0f, saveDebug);
                debugTotalCrowdDistance += penalization1;
                sqrDistance += penalization1 * FeatureWeights[FeatureStaticSize];
                if (!saveDebug && sqrDistance > minDistance)
                {
                    break;
                }
                float penalization2 = ComputePenalization(pos2, primaryAxisUnit2, secondaryAxisUnit2, ellipse2.xy, obstacle, CrowdSecondTrajectoryWeight, saveDebug);
                debugTotalCrowdDistance += penalization2;
                sqrDistance += penalization2 * FeatureWeights[FeatureStaticSize];
                if (!saveDebug && sqrDistance > minDistance)
                {
                    break;
                }
                float penalization3 = ComputePenalization(pos3, primaryAxisUnit3, secondaryAxisUnit3, ellipse3.xy, obstacle, CrowdThirdTrajectoryWeight, saveDebug);
                debugTotalCrowdDistance += penalization3;
                sqrDistance += penalization3 * FeatureWeights[FeatureStaticSize];
                if (!saveDebug && sqrDistance > minDistance)
                {
                    break;
                }
            }

            Distances[i] = sqrDistance;
            if (sqrDistance < minDistance)
            {
                minDistance = sqrDistance;
                BestIndex[0] = i;
            }
            if (saveDebug)
            {
                DebugCrowdDistance[0] = debugTotalCrowdDistance;
                DebugCrowdDistance[1] = auxDebugTrajectory;
            }

            return minDistance;
        }

        // Features
        // 0, 1 -> Position 0
        // 2, 3 -> Position 1
        // 4, 5 -> Position 2
        // ---
        // 6, 7   -> Direction 0
        // 8, 9   -> Direction 1
        // 10, 11 -> Direction 2
        // ---
        // ... Pose Features
        // ---
        // FeatureStaticSize +
        // 0, 1, 2, 3   -> Ellipse 0
        // 4, 5, 6, 7   -> Ellipse 1
        // 8, 9, 10, 11 -> Ellipse 2
        // --
        public void Execute()
        {
            // DEBUG
            if (IsDebug)
            {
                NumberDebugPoints[0] = 0;
                FeatureCheck(DebugIndex, float.MinValue, true);
                DebugCrowdDistance[2] = DebugCrowdDistance[0] * FeatureWeights[FeatureStaticSize];
            }
            else
            {
                float minDistance = float.MaxValue;
                BestIndex[0] = -1;

                for (int i = 0; i < Valid.Length; ++i)
                {
                    if (Valid[i] && TagMask[i])
                    {
                        minDistance = FeatureCheck(i, minDistance, false);
                    }
                }
            }
        }
    }
}