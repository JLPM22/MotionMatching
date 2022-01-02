using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
        public Vector2[] FutureTrajectoryLocalPosition; // 2D projected on the ground
        public Vector2[] FutureTrajectoryLocalDirection; // 2D projected on the ground
        // Pose ---------
        public Vector3 LeftFootLocalPosition;
        public Vector3 RightFootLocalPosition;
        public Vector3 LeftFootLocalVelocity;
        public Vector3 RightFootLocalVelocity;
        public Vector3 HipsLocalVelocity;

        // TODO: Check property per property that they are correctly working and affecting the final result
        public float SqrDistance(FeatureVector other, float responsiveness, float quality)
        {
            float sum = 0.0f;
            for (int i = 0; i < FutureTrajectoryLocalPosition.Length; i++)
            {
                sum += Vector2.SqrMagnitude(FutureTrajectoryLocalPosition[i] - other.FutureTrajectoryLocalPosition[i]) * responsiveness;
                sum += Vector2.SqrMagnitude(FutureTrajectoryLocalDirection[i] - other.FutureTrajectoryLocalDirection[i]) * responsiveness;
            }
            sum += Vector3.SqrMagnitude(LeftFootLocalPosition - other.LeftFootLocalPosition) * quality;
            sum += Vector3.SqrMagnitude(RightFootLocalPosition - other.RightFootLocalPosition) * quality;
            sum += Vector3.SqrMagnitude(LeftFootLocalVelocity - other.LeftFootLocalVelocity) * quality;
            sum += Vector3.SqrMagnitude(RightFootLocalVelocity - other.RightFootLocalVelocity) * quality;
            sum += Vector3.SqrMagnitude(HipsLocalVelocity - other.HipsLocalVelocity) * quality;
            return sum;
        }
    }
}