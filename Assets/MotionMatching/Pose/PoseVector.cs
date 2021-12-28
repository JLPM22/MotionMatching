using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    /// <summary>
    /// Stores full pose representation for one pose
    /// </summary>
    public struct PoseVector
    {
        public Vector3[] JointLocalPositions;
        public Quaternion[] JointLocalRotations;
        public Vector3 RootMotion; // World Space

        public PoseVector(Vector3[] jointLocalPositions, Quaternion[] jointLocalRotations, Vector3 rootMotion)
        {
            JointLocalPositions = jointLocalPositions;
            JointLocalRotations = jointLocalRotations;
            RootMotion = rootMotion;
        }
    }
}