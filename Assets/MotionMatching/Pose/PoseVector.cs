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
        public Vector3[] JointLocalPositions; // [0] contains root world information
        public Quaternion[] JointLocalRotations; // [0] contains root world information
        public Vector3 RootVelocity; // Local to character forward facing direction
        public Quaternion RootRotVelocity; // Local to character forward facing direction

        public PoseVector(Vector3[] jointLocalPositions, Quaternion[] jointLocalRotations, Vector3 rootVelocity, Quaternion rootRotVelocity)
        {
            JointLocalPositions = jointLocalPositions;
            JointLocalRotations = jointLocalRotations;
            RootVelocity = rootVelocity;
            RootRotVelocity = rootRotVelocity;
        }
    }
}