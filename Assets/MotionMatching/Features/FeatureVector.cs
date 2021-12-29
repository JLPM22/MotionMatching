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
    }
}