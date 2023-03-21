using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;

namespace MotionMatching
{
    /// <summary>
    /// Stores full pose representation for one pose
    /// </summary>
    public struct PoseVector
    {
        // The first element is the SimulationBone (added artificially), and the rest are the bones of the original skeleton
        public float3[] JointLocalPositions;
        public quaternion[] JointLocalRotations;
        public float3[] JointLocalVelocities; // Computed from World Positions
        public float3[] JointLocalAngularVelocities; // Computed from World Rotations
        public bool LeftFootContact; // True if the foot is in contact with the ground, false otherwise
        public bool RightFootContact;

        public PoseVector(float3[] jointLocalPositions, quaternion[] jointLocalRotations,
                          float3[] jointLocalVelocities, float3[] jointLocalAngularVelocities,
                          bool leftFootContact, bool rightFootContact)
        {
            JointLocalPositions = jointLocalPositions;
            JointLocalRotations = jointLocalRotations;
            JointLocalVelocities = jointLocalVelocities;
            JointLocalAngularVelocities = jointLocalAngularVelocities;
            LeftFootContact = leftFootContact;
            RightFootContact = rightFootContact;
        }
    }
}