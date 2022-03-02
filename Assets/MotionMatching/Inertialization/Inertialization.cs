using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    /// <summary>
    /// Inertialization is a type of blending between two poses.
    /// Typically to blend between two poses, we use crossfade. Both poses are stored queried during the transition
    /// and they are blended during the transition.
    /// The basic idea of inertialization is when we change the pose we change it for real but compute offsets that
    /// takes us from the new/target pose to the source one. Then, we decay this offset using a polynomial function or springs.
    /// Decaying the offset will progressively take us to the target pose.
    /// </summary>
    public class Inertialization
    {
        public quaternion[] InertializedRotations;
        public float3[] InertializedAngularVelocities;

        private quaternion[] OffsetRotations;
        private float3[] OffsetAngularVelocities;

        public Inertialization(Skeleton skeleton)
        {
            int numJoints = skeleton.Joints.Count;
            InertializedRotations = new quaternion[numJoints];
            InertializedAngularVelocities = new float3[numJoints];
            OffsetRotations = new quaternion[numJoints];
            for (int i = 0; i < numJoints; i++) OffsetRotations[i] = quaternion.identity; // init to a valid quaternion
            OffsetAngularVelocities = new float3[numJoints];
        }

        /// <summary>
        /// It takes as input the current state of the source pose and the target pose.
        /// It sets up the inertialization, which can then by updated by calling InertializeUpdate(...).
        /// Root rotation and all Joints rotations are inertialized. (No root position)
        /// </summary>
        public void PoseTransition(PoseSet poseSet, int sourcePoseIndex, int targetPoseIndex)
        {
            PoseVector sourcePose = poseSet.Poses[sourcePoseIndex];
            PoseVector targetPose = poseSet.Poses[targetPoseIndex];

            for (int i = 0; i < sourcePose.JointLocalRotations.Length; i++)
            {
                quaternion sourceJointRotation = sourcePose.JointLocalRotations[i];
                quaternion targetJointRotation = targetPose.JointLocalRotations[i];
                float3 sourceJointAngularVelocity = sourcePose.JointAngularVelocities[i];
                float3 targetJointAngularVelocity = targetPose.JointAngularVelocities[i];
                InertializeJointTransition(sourceJointRotation, sourceJointAngularVelocity,
                                           targetJointRotation, targetJointAngularVelocity,
                                           ref OffsetRotations[i], ref OffsetAngularVelocities[i]);
            }
        }

        /// <summary>
        /// Updates the inertialization decaying the offset from the source pose (specified in InertializePoseTransition(...))
        /// to the target pose.
        /// </summary>
        public void Update(PoseVector targetPose, float halfLife, float deltaTime)
        {
            for (int i = 0; i < targetPose.JointLocalRotations.Length; i++)
            {
                quaternion targetJointRotation = targetPose.JointLocalRotations[i];
                float3 targetAngularVelocity = targetPose.JointAngularVelocities[i];
                InertializeJointUpdate(targetJointRotation, targetAngularVelocity,
                                       halfLife, deltaTime,
                                       ref OffsetRotations[i], ref OffsetAngularVelocities[i],
                                       out InertializedRotations[i], out InertializedAngularVelocities[i]);
            }
        }

        /// <summary>
        /// Compute the offsets from the source pose to the target pose.
        /// Offsets are in/out since we may start a inertialization in the middle of another inertialization.
        /// </summary>
        private static void InertializeJointTransition(quaternion sourceRot, float3 sourceAngularVel,
                                                       quaternion targetRot, float3 targetAngularVel,
                                                       ref quaternion offsetRot, ref float3 offsetAngularVel)
        {
            offsetRot = math.normalizesafe(MathExtensions.Abs(math.mul(math.inverse(targetRot), math.mul(sourceRot, offsetRot))));
            offsetAngularVel = (sourceAngularVel + offsetAngularVel) - targetAngularVel;
        }

        /// <summary>
        /// Updates the inertialization decaying the offset and applying it to the target pose
        /// </summary>
        private static void InertializeJointUpdate(quaternion targetRot, float3 targetAngularVel,
                                                   float halfLife, float deltaTime,
                                                   ref quaternion offsetRot, ref float3 offsetAngularVel,
                                                   out quaternion newRot, out float3 newAngularVel)
        {
            Spring.DecaySpringDamperImplicit(ref offsetRot, ref offsetAngularVel, halfLife, deltaTime);
            newRot = math.mul(targetRot, offsetRot);
            newAngularVel = targetAngularVel + offsetAngularVel;
        }
    }
}