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
        public float3[] JointLocalPositions; // JointLocalPositions[0] is (0,0,0), the actual position in world space is encoded in RootWorld
        public quaternion[] JointLocalRotations; // JointLocalRotations[0] the Y axis has been removed
        public float3[] JointVelocities; // Computed from World Positions
        public float3[] JointAngularVelocities; // Computed from World Rotations
        public float3 RootDisplacement; // Local to character forward facing direction
        public quaternion RootRotDisplacement; // Local to character forward facing direction (Only Y Axis)
        public float3 RootWorld;
        public quaternion RootWorldRot;

        public PoseVector(float3[] jointLocalPositions, quaternion[] jointLocalRotations,
                          float3[] jointVelocities, float3[] jointAngularVelocities,
                          float3 rootDisplacement, quaternion rootRotDisplacement,
                          float3 rootWorld, quaternion rootWorldRot)
        {
            JointLocalPositions = jointLocalPositions;
            JointLocalRotations = jointLocalRotations;
            JointVelocities = jointVelocities;
            JointAngularVelocities = jointAngularVelocities;
            RootDisplacement = rootDisplacement;
            RootRotDisplacement = rootRotDisplacement;
            RootWorld = rootWorld;
            RootWorldRot = rootWorldRot;
        }
    }
}