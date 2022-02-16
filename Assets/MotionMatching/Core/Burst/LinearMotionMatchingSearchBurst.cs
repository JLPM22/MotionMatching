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
        [ReadOnly]
        public NativeArray<FeatureVector> Features;
        [ReadOnly]
        public FeatureVector QueryFeature;
        [ReadOnly]
        public float Responsiveness;
        [ReadOnly]
        public float Quality;

        [WriteOnly]
        public NativeArray<int> BestIndex;

        public void Execute()
        {
            float min = float.MaxValue;
            int bestIndex = -1;
            for (int i = 0; i < Features.Length; ++i)
            {
                FeatureVector fv = Features[i];
                if (fv.Valid)
                {
                    float sqrDistance = QueryFeature.SqrDistance(fv, Responsiveness, Quality);
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