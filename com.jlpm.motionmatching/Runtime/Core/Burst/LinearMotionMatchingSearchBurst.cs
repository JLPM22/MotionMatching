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
        [ReadOnly] public NativeArray<bool> TagMask; // TODO: convert to a bitmask to optimize memory
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
                    if (sqrDistance.x < min.x && Valid[index] && TagMask[index]) // Checking Valid here is more performant than using it to avoid calculations... probably most of the time sqrDistance < min is false and reduces memory accesses (Valid is not used)
                    {
                        min.x = sqrDistance.x;
                        bestIndex.x = index;
                    }
                    if (sqrDistance.y < min.y && Valid[index + 1] && TagMask[index + 1])
                    {
                        min.y = sqrDistance.y;
                        bestIndex.y = index + 1;
                    }
                    if (sqrDistance.z < min.z && Valid[index + 2] && TagMask[index + 1])
                    {
                        min.z = sqrDistance.z;
                        bestIndex.z = index + 2;
                    }
                    if (sqrDistance.w < min.w && Valid[index + 3] && TagMask[index + 1])
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
                if (sqrDistance < _min && Valid[index] && TagMask[index])
                {
                    _min = sqrDistance;
                    _bestIndex = index;
                }
            }
            BestIndex[0] = _bestIndex;
        }
    }


    [BurstCompile]
    public struct CrowdLinearMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid;
        [ReadOnly] public NativeArray<bool> TagMask;
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> QueryFeature;
        [ReadOnly] public NativeArray<float> FeatureWeights;
        [ReadOnly] public NativeArray<float> Mean;
        [ReadOnly] public NativeArray<float> Std;
        [ReadOnly] public NativeArray<(float2, float)> Obstacles;
        [ReadOnly] public float ObstacleRadius;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int PoseOffset;
        [ReadOnly] public float CurrentDistance;
        [ReadOnly] public float CrowdWeight;

        [WriteOnly] public NativeArray<int> BestIndex;
        public NativeArray<float> DebugCrowdDistance;

        private float DistanceFunction(float distance)
        {
            return -math.pow((1.0f - distance), 4.0f) * math.log(math.max(distance, 1e-3f));
        }

        // Features
        // 0, 1 -> Position 0
        // 2, 3 -> Position 1
        // 4, 5 -> Position 2
        // --
        // 6, 7 -> Direction 0
        // 8, 9 -> Direction 1
        // 10, 11 -> Direction 2
        // --
        // 12, 13 -> Ellipse 0
        // 14, 15 -> Ellipse 1
        // 16, 17 -> Ellipse 2
        // --
        // ... Pose Features
        public void Execute()
        {
            float minDistance = float.MaxValue;
            float debugCrowd = DebugCrowdDistance[0]; // DEBUG
            float debugTrajectory = DebugCrowdDistance[1]; // DEBUG
            int bestIndex = -1;

            for (int i = 0; i < Valid.Length; ++i)
            {
                if (Valid[i] && TagMask[i])
                {
                    float sqrDistance = 0.0f;
                    int featureIndex = i * FeatureSize;

                    for (int j = 0; j < FeatureSize; ++j)
                    {
                        if (j >= 12 && j <= 17) continue; // DEBUG
                        float diff = Features[featureIndex + j] - QueryFeature[j];
                        sqrDistance += diff * diff * FeatureWeights[j];
                    }
                    float auxDebugTrajectory = sqrDistance;

                    // crowd forces
                    float2 pos1 = new(Features[featureIndex + 0] * Std[0] + Mean[0],
                                      Features[featureIndex + 1] * Std[1] + Mean[1]);
                    float2 pos2 = new(Features[featureIndex + 2] * Std[2] + Mean[2],
                                      Features[featureIndex + 3] * Std[3] + Mean[3]);
                    float2 pos3 = new(Features[featureIndex + 4] * Std[4] + Mean[4],
                                      Features[featureIndex + 5] * Std[5] + Mean[5]);
                    float2 ellipse1 = new(Features[featureIndex + 12], Features[featureIndex + 13]);
                    float2 ellipse2 = new(Features[featureIndex + 14], Features[featureIndex + 15]);
                    float2 ellipse3 = new(Features[featureIndex + 16], Features[featureIndex + 17]);
                    float2 primaryAxisUnit1 = math.normalize(pos2 - pos1);
                    float2 primaryAxisUnit2 = math.normalize(pos3 - pos2);
                    float2 secondaryAxisUnit1 = new(-primaryAxisUnit1.y, primaryAxisUnit1.x);
                    float2 secondaryAxisUnit2 = new(-primaryAxisUnit2.y, primaryAxisUnit2.x);

                    float debugTotalCrowdDistance = 0.0f;
                    const float threshold = 1.0f;
                    for (int obstacle = 0; obstacle < Obstacles.Length; obstacle++)
                    {
                        float distance1 = UtilitiesBurst.DistanceToEllipse(pos1, primaryAxisUnit1, secondaryAxisUnit1, ellipse1, Obstacles[obstacle].Item1, out _) - Obstacles[obstacle].Item2;
                        float distance2 = UtilitiesBurst.DistanceToEllipse(pos2, primaryAxisUnit2, secondaryAxisUnit2, ellipse2, Obstacles[obstacle].Item1, out _) - Obstacles[obstacle].Item2;
                        float distance3 = UtilitiesBurst.DistanceToEllipse(pos3, primaryAxisUnit2, secondaryAxisUnit2, ellipse3, Obstacles[obstacle].Item1, out _) - Obstacles[obstacle].Item2;
                        float crowdDistance = 0.0f;
                        if (distance1 < threshold)
                        {
                            crowdDistance += DistanceFunction(1.0f - ((threshold - distance1) / threshold));
                        }
                        if (distance2 < threshold)
                        {
                            crowdDistance += DistanceFunction(1.0f - ((threshold - distance2) / threshold));
                        }
                        if (distance3 < threshold)
                        {
                            crowdDistance += DistanceFunction(1.0f - ((threshold - distance3) / threshold));
                        }

                        debugTotalCrowdDistance += crowdDistance;
                        sqrDistance += crowdDistance * CrowdWeight;
                    }

                    if (sqrDistance < minDistance)
                    {
                        minDistance = sqrDistance;
                        bestIndex = i;
                    }
                    if (bestIndex == i)
                    {
                        debugCrowd = debugTotalCrowdDistance;
                        debugTrajectory = auxDebugTrajectory;
                    }
                }
            }

            BestIndex[0] = bestIndex;
            DebugCrowdDistance[0] = debugCrowd;
            DebugCrowdDistance[1] = debugTrajectory;
            DebugCrowdDistance[2] = debugCrowd * CrowdWeight;
        }
    }


    [BurstCompile]
    public struct VisCrowdLinearMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid;
        [ReadOnly] public NativeArray<bool> TagMask;
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> QueryFeature;
        [ReadOnly] public NativeArray<float> FeatureWeights;
        [ReadOnly] public NativeArray<float> Mean;
        [ReadOnly] public NativeArray<float> Std;
        [ReadOnly] public NativeArray<(float2, float)> Obstacles;
        [ReadOnly] public float ObstacleRadius;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int PoseOffset;
        [ReadOnly] public float CurrentDistance;
        [ReadOnly] public float CrowdWeight;

        [WriteOnly] public NativeArray<int> BestIndex;
        public NativeArray<float> DebugCrowdDistance;
        [WriteOnly] public NativeArray<float> Distances;

        private float DistanceFunction(float distance, float threshold)
        {
            if (distance > threshold)
            {
                return 0.0f;
            }
            return -math.pow((threshold - distance), 4.0f) * math.log(math.max(distance / threshold, 1e-3f));
        }

        // Features
        // 0, 1 -> Position 0
        // 2, 3 -> Position 1
        // 4, 5 -> Position 2
        // --
        // 6, 7 -> Direction 0
        // 8, 9 -> Direction 1
        // 10, 11 -> Direction 2
        // --
        // 12, 13 -> Ellipse 0
        // 14, 15 -> Ellipse 1
        // 16, 17 -> Ellipse 2
        // --
        // ... Pose Features
        public void Execute()
        {
            float minDistance = float.MaxValue;
            float debugCrowd = DebugCrowdDistance[0]; // DEBUG
            float debugTrajectory = DebugCrowdDistance[1]; // DEBUG
            int bestIndex = -1;

            for (int i = 0; i < Valid.Length; ++i)
            {
                if (Valid[i] && TagMask[i])
                {
                    float sqrDistance = 0.0f;
                    int featureIndex = i * FeatureSize;

                    for (int j = 0; j < FeatureSize; ++j)
                    {
                        if (j >= 12 && j <= 17) continue; // DEBUG
                        float diff = Features[featureIndex + j] - QueryFeature[j];
                        sqrDistance += diff * diff * FeatureWeights[j];
                    }
                    float auxDebugTrajectory = sqrDistance;

                    // crowd forces
                    float2 pos1 = new(Features[featureIndex + 0] * Std[0] + Mean[0],
                                      Features[featureIndex + 1] * Std[1] + Mean[1]);
                    float2 pos2 = new(Features[featureIndex + 2] * Std[2] + Mean[2],
                                      Features[featureIndex + 3] * Std[3] + Mean[3]);
                    float2 pos3 = new(Features[featureIndex + 4] * Std[4] + Mean[4],
                                      Features[featureIndex + 5] * Std[5] + Mean[5]);
                    float2 ellipse1 = new(Features[featureIndex + 12], Features[featureIndex + 13]);
                    float2 ellipse2 = new(Features[featureIndex + 14], Features[featureIndex + 15]);
                    float2 ellipse3 = new(Features[featureIndex + 16], Features[featureIndex + 17]);
                    float2 primaryAxisUnit1 = math.normalize(pos2 - pos1);
                    float2 primaryAxisUnit2 = math.normalize(pos3 - pos2);
                    float2 secondaryAxisUnit1 = new(-primaryAxisUnit1.y, primaryAxisUnit1.x);
                    float2 secondaryAxisUnit2 = new(-primaryAxisUnit2.y, primaryAxisUnit2.x);

                    float debugTotalCrowdDistance = 0.0f;
                    const float threshold = 0.6f;
                    for (int obstacle = 0; obstacle < Obstacles.Length; obstacle++)
                    {
                        float distance1 = UtilitiesBurst.DistanceToEllipse(pos1, primaryAxisUnit1, secondaryAxisUnit1, ellipse1, Obstacles[obstacle].Item1, out _) - Obstacles[obstacle].Item2;
                        float distance2 = UtilitiesBurst.DistanceToEllipse(pos2, primaryAxisUnit2, secondaryAxisUnit2, ellipse2, Obstacles[obstacle].Item1, out _) - Obstacles[obstacle].Item2;
                        float distance3 = UtilitiesBurst.DistanceToEllipse(pos3, primaryAxisUnit2, secondaryAxisUnit2, ellipse3, Obstacles[obstacle].Item1, out _) - Obstacles[obstacle].Item2;
                        float crowdDistance = 0.0f;
                        crowdDistance += DistanceFunction(distance1, threshold);
                        crowdDistance += DistanceFunction(distance2, threshold);
                        crowdDistance += DistanceFunction(distance3, threshold);

                        debugTotalCrowdDistance += crowdDistance;
                        sqrDistance += crowdDistance * CrowdWeight;
                    }

                    Distances[i] = sqrDistance;
                    if (sqrDistance < minDistance)
                    {
                        minDistance = sqrDistance;
                        bestIndex = i;
                    }
                    if (bestIndex == i)
                    {
                        debugCrowd = debugTotalCrowdDistance;
                        debugTrajectory = auxDebugTrajectory;
                    }
                }
            }

            BestIndex[0] = bestIndex;
            DebugCrowdDistance[0] = debugCrowd;
            DebugCrowdDistance[1] = debugTrajectory;
            DebugCrowdDistance[2] = debugCrowd * CrowdWeight;
        }
    }

    [BurstCompile]
    public struct CircleCrowdLinearMotionMatchingSearchBurst : IJob
    {
        [ReadOnly] public NativeArray<bool> Valid;
        [ReadOnly] public NativeArray<bool> TagMask;
        [ReadOnly] public NativeArray<float> Features;
        [ReadOnly] public NativeArray<float> QueryFeature;
        [ReadOnly] public NativeArray<float> FeatureWeights;
        [ReadOnly] public NativeArray<float> Mean;
        [ReadOnly] public NativeArray<float> Std;
        [ReadOnly] public float2 ObstaclePos;
        [ReadOnly] public float ObstacleRadius;
        [ReadOnly] public int FeatureSize;
        [ReadOnly] public int PoseOffset;
        [ReadOnly] public float CurrentDistance;

        [WriteOnly] public NativeArray<int> BestIndex;

        // Features
        // 0, 1 -> Position 0
        // 2, 3 -> Position 1
        // 4, 5 -> Position 2
        // --
        // 6, 7 -> Direction 0
        // 8, 9 -> Direction 1
        // 10, 11 -> Direction 2
        // --
        // 12 -> Circle 0
        // 13 -> Circle 1
        // 14 -> Circle 2
        // --
        // ... Pose Features
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

                    for (int j = 0; j < FeatureSize; ++j)
                    {
                        if (j >= 12 && j <= 14) continue;
                        float diff = Features[featureIndex + j] - QueryFeature[j];
                        sqrDistance += diff * diff * FeatureWeights[j];
                    }
                    // crowd forces
                    float2 pos1 = new(Features[featureIndex + 0] * Std[0] + Mean[0],
                                      Features[featureIndex + 1] * Std[1] + Mean[1]);
                    float2 pos2 = new(Features[featureIndex + 2] * Std[2] + Mean[2],
                                      Features[featureIndex + 3] * Std[3] + Mean[3]);
                    float2 pos3 = new(Features[featureIndex + 4] * Std[4] + Mean[4],
                                      Features[featureIndex + 5] * Std[5] + Mean[5]);
                    float circle1 = Features[featureIndex + 12];
                    float circle2 = Features[featureIndex + 13];
                    float circle3 = Features[featureIndex + 14];
                    float distance1 = math.distance(ObstaclePos, pos1) - circle1 - ObstacleRadius;
                    float distance2 = math.distance(ObstaclePos, pos2) - circle2 - ObstacleRadius;
                    float distance3 = math.distance(ObstaclePos, pos3) - circle3 - ObstacleRadius;
                    const float threshold = 4.0f;
                    float crowdDistance = 0.0f;
                    if (distance1 < threshold)
                    {
                        crowdDistance = (threshold - distance1) / threshold;
                    }
                    if (distance2 < threshold)
                    {
                        crowdDistance += (threshold - distance2) / threshold;
                    }
                    if (distance3 < threshold)
                    {
                        crowdDistance += (threshold - distance3) / threshold;
                    }

                    const float crowdWeight = 5.0f;
                    sqrDistance += (crowdDistance * crowdDistance) * crowdWeight;

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
}