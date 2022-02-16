using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace MotionMatching
{
    /// <summary>
    /// Stores features of a pose for use in Motion Matching
    /// </summary>
    public struct FeatureVector
    {
        public bool Valid;
        // Vectors are local to the character
        // Trajectory ---
        public float2[] FutureTrajectoryLocalPosition; // 2D projected on the ground
        public float2[] FutureTrajectoryLocalDirection; // 2D projected on the ground
        // Pose ---------
        public float3 LeftFootLocalPosition;
        public float3 RightFootLocalPosition;
        public float3 LeftFootLocalVelocity;
        public float3 RightFootLocalVelocity;
        public float3 HipsLocalVelocity;

        // TODO: Check property per property that they are correctly working and affecting the final result
        public float SqrDistance(FeatureVector other, float responsiveness, float quality)
        {
            float sum = 0.0f;
            for (int i = 0; i < FutureTrajectoryLocalPosition.Length; i++)
            {
                sum += lengthsq(FutureTrajectoryLocalPosition[i] - other.FutureTrajectoryLocalPosition[i]) * responsiveness;
                sum += lengthsq(FutureTrajectoryLocalDirection[i] - other.FutureTrajectoryLocalDirection[i]) * responsiveness;
            }
            sum += lengthsq(LeftFootLocalPosition - other.LeftFootLocalPosition) * quality;
            sum += lengthsq(RightFootLocalPosition - other.RightFootLocalPosition) * quality;
            sum += lengthsq(LeftFootLocalVelocity - other.LeftFootLocalVelocity) * quality;
            sum += lengthsq(RightFootLocalVelocity - other.RightFootLocalVelocity) * quality;
            sum += lengthsq(HipsLocalVelocity - other.HipsLocalVelocity) * quality;
            return sum;
        }
    }
}