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
        // Displacements and Velocities are from the previous frame to the current one
        public float3[] JointLocalPositions; // JointLocalPositions[0] is (0,0,0), the actual position in world space is encoded in RootWorld
        public quaternion[] JointLocalRotations; // JointLocalRotations[0] the Y axis has been removed
        public float3[] JointVelocities; // Computed from World Positions, JointVelocities[0] contains the actual velocity of the root
        public float3[] JointAngularVelocities; // Computed from World Rotations, JointAngularVelocities[0] contains the angular velocity of the rotation of the root without Y axis
        public float3 RootDisplacement; // Local to character forward facing direction
        public quaternion RootRotDisplacement; // Local to character forward facing direction (only Y Axis)
        public float3 RootRotAngularVelocity; // Angular Velocity of the root rotation (only Y Axis)
        public float3 RootWorld;
        public quaternion RootWorldRot;

        public PoseVector(float3[] jointLocalPositions, quaternion[] jointLocalRotations,
                          float3[] jointVelocities, float3[] jointAngularVelocities,
                          float3 rootDisplacement, quaternion rootRotDisplacement, float3 rootRotAngularVelocity,
                          float3 rootWorld, quaternion rootWorldRot)
        {
            JointLocalPositions = jointLocalPositions;
            JointLocalRotations = jointLocalRotations;
            JointVelocities = jointVelocities;
            JointAngularVelocities = jointAngularVelocities;
            RootDisplacement = rootDisplacement;
            RootRotDisplacement = rootRotDisplacement;
            RootRotAngularVelocity = rootRotAngularVelocity;
            RootWorld = rootWorld;
            RootWorldRot = rootWorldRot;
        }
    }
}