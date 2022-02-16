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
    public unsafe struct FeatureVector
    {
        public bool Valid;
        // Vectors are local to the character
        // Trajectory ---
        // Using two floats instead of a float2 is slower in standard c#... it gets faster when using Burst (because it uses SIMD)
        private fixed float FutureTrajectoryLocalPosition[6]; // 2D projected on the ground
        private fixed float FutureTrajectoryLocalDirection[6]; // 2D projected on the ground
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
            for (int i = 0; i < GetFutureTrajectoryLength(); i++)
            {
                sum += lengthsq(GetFutureTrajectoryLocalPosition(i) - other.GetFutureTrajectoryLocalPosition(i)) * responsiveness;
                sum += lengthsq(GetFutureTrajectoryLocalDirection(i) - other.GetFutureTrajectoryLocalDirection(i)) * responsiveness;
            }
            sum += lengthsq(LeftFootLocalPosition - other.LeftFootLocalPosition) * quality;
            sum += lengthsq(RightFootLocalPosition - other.RightFootLocalPosition) * quality;
            sum += lengthsq(LeftFootLocalVelocity - other.LeftFootLocalVelocity) * quality;
            sum += lengthsq(RightFootLocalVelocity - other.RightFootLocalVelocity) * quality;
            sum += lengthsq(HipsLocalVelocity - other.HipsLocalVelocity) * quality;
            return sum;
        }

        public static int GetFutureTrajectoryLength()
        {
            return 3;
        }

        public float2 GetFutureTrajectoryLocalPosition(int index)
        {
            int offset = index * 2;
            return new float2(FutureTrajectoryLocalPosition[offset], FutureTrajectoryLocalPosition[offset + 1]);
        }
        public float2 GetFutureTrajectoryLocalDirection(int index)
        {
            int offset = index * 2;
            return new float2(FutureTrajectoryLocalDirection[offset], FutureTrajectoryLocalDirection[offset + 1]);
        }
        public void SetFutureTrajectoryLocalPosition(int index, float2 position)
        {
            int offset = index * 2;
            FutureTrajectoryLocalPosition[offset] = position.x;
            FutureTrajectoryLocalPosition[offset + 1] = position.y;
        }
        public void SetFutureTrajectoryLocalDirection(int index, float2 direction)
        {
            int offset = index * 2;
            FutureTrajectoryLocalDirection[offset] = direction.x;
            FutureTrajectoryLocalDirection[offset + 1] = direction.y;
        }
    }
}