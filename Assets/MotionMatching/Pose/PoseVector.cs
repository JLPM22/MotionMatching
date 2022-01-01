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
        public Vector3[] JointLocalPositions; // JointLocalPositions[0] is (0,0,0), the actual position in world space is encoded in RootWorld
        public Quaternion[] JointLocalRotations; // JointLocalRotations[0] the Y axis has been removed
        public Vector3 RootVelocity; // Local to character forward facing direction
        public Quaternion RootRotVelocity; // Local to character forward facing direction (Only Y Axis)
        public Vector3 RootWorld;
        public Quaternion RootWorldRot;

        public PoseVector(Vector3[] jointLocalPositions, Quaternion[] jointLocalRotations, Vector3 rootVelocity, Quaternion rootRotVelocity, Vector3 rootWorld, Quaternion rootWorldRot)
        {
            JointLocalPositions = jointLocalPositions;
            JointLocalRotations = jointLocalRotations;
            RootVelocity = rootVelocity;
            RootRotVelocity = rootRotVelocity;
            RootWorld = rootWorld;
            RootWorldRot = rootWorldRot;
        }
    }
}