using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;

    /// <summary>
    /// Extracts full pose for Motion Matching from BVHAnimation
    /// </summary>
    public class PoseExtractor
    {
        /// <summary>
        /// Extract the poses from bvhAnimation and store it in poseSet
        /// poseSet is not cleared, it will add bvhAnimation the the existing poses
        /// Returns true if the bvhAnimation was added to the poseSet, false otherwise
        /// </summary>
        public bool Extract(BVHAnimation bvhAnimation, PoseSet poseSet, MotionMatchingData mmData)
        {
            // Set Poses
            int nFrames = bvhAnimation.Frames.Length;
            PoseVector[] poses = new PoseVector[nFrames];
            for (int i = 0; i < nFrames; i++)
            {
                poses[i] = ExtractPose(bvhAnimation, i, mmData, poseSet, poses);
            }
            return poseSet.AddClip(poses, bvhAnimation.FrameTime);
        }

        private PoseVector ExtractPose(BVHAnimation bvhAnimation, int frameIndex, MotionMatchingData mmData, PoseSet poseSet, PoseVector[] poseCache)
        {
            BVHAnimation.Frame frame = bvhAnimation.Frames[frameIndex];

            int nJoints = bvhAnimation.Skeleton.Joints.Count;
            int outNJoint = nJoints + 1; // +1 for SimulationBone

            // Native Arrays used by Burst for Output
            NativeArray<float3> jointLocalPositions = new NativeArray<float3>(outNJoint, Allocator.TempJob);
            NativeArray<quaternion> jointLocalRotations = new NativeArray<quaternion>(outNJoint, Allocator.TempJob);
            NativeArray<float3> jointVelocities = new NativeArray<float3>(outNJoint, Allocator.TempJob);
            NativeArray<float3> jointAngularVelocities = new NativeArray<float3>(outNJoint, Allocator.TempJob);

            // Native Arrays used by Burst for Input
            NativeArray<float3> jointOffsets = new NativeArray<float3>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) jointOffsets[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
            NativeArray<quaternion> frameLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) frameLocalRotations[i] = frame.LocalRotations[i];

            // Use Burst (way faster than doing it in regular C#)
            var jobSB = new AddSimulationBoneBurst
            {
                // Input
                Njoints = nJoints,
                JointOffsets = jointOffsets,
                FrameLocalRotations = frameLocalRotations,
                FrameRootMotion = frame.RootMotion,
                HipsForwardLocalVector = mmData.HipsForwardLocalVector,
                // Output
                OutJointLocalPositions = jointLocalPositions,
                OutJointLocalRotations = jointLocalRotations,
            };
            jobSB.Schedule().Complete();

            // Dispose
            jointOffsets.Dispose();
            frameLocalRotations.Dispose();

            if (frameIndex == 0)
            {
                for (int i = 0; i < jointVelocities.Length; i++) jointVelocities[i] = float3.zero;
                for (int i = 0; i < jointAngularVelocities.Length; i++) jointAngularVelocities[i] = float3.zero;
            }
            else
            {
                PoseVector prevPos = poseCache[frameIndex - 1];

                // Native Arrays used by Burst for Input
                NativeArray<int> jointParents = new NativeArray<int>(outNJoint, Allocator.TempJob);
                for (int i = 0; i < outNJoint; i++) jointParents[i] = poseSet.Skeleton.Joints[i].ParentIndex; // poseSet skeleton already includes SimulationBone
                NativeArray<float3> prevLocalPositions = new NativeArray<float3>(outNJoint, Allocator.TempJob);
                for (int i = 0; i < outNJoint; i++) prevLocalPositions[i] = prevPos.JointLocalPositions[i];
                NativeArray<quaternion> prevLocalRotations = new NativeArray<quaternion>(outNJoint, Allocator.TempJob);
                for (int i = 0; i < outNJoint; i++) prevLocalRotations[i] = prevPos.JointLocalRotations[i];

                var jobVelocities = new ExtractVelocitiesBurst
                {
                    // Input
                    Njoints = outNJoint,
                    FrameTime = bvhAnimation.FrameTime,
                    JointParents = jointParents,
                    LocalPositions = jointLocalPositions,
                    PrevLocalPositions = prevLocalPositions,
                    LocalRotations = jointLocalRotations,
                    PrevLocalRotations = prevLocalRotations,
                    // Output
                    OutJointVelocities = jointVelocities,
                    OutJointAngularVelocities = jointAngularVelocities,
                };
                jobVelocities.Schedule().Complete();

                // Dispose
                jointParents.Dispose();
                prevLocalPositions.Dispose();
                prevLocalRotations.Dispose();
            }

            // Copy to managed arrays
            float3[] _jointLocalPositions = new float3[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointLocalPositions[i] = jointLocalPositions[i];
            quaternion[] _jointLocalRotations = new quaternion[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointLocalRotations[i] = jointLocalRotations[i];
            float3[] _jointVelocities = new float3[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointVelocities[i] = jointVelocities[i];
            float3[] _jointAngularVelocities = new float3[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointAngularVelocities[i] = jointAngularVelocities[i];

            // Dispose Output Native Arrays
            jointLocalPositions.Dispose();
            jointLocalRotations.Dispose();
            jointVelocities.Dispose();
            jointAngularVelocities.Dispose();

            // Result
            return new PoseVector(_jointLocalPositions, _jointLocalRotations,
                                  _jointVelocities, _jointAngularVelocities);
        }

        [BurstCompile]
        public struct AddSimulationBoneBurst : IJob
        {
            // Input arrays are of size nJoints (not including SimulationBone)
            [ReadOnly] public int Njoints;
            [ReadOnly] public NativeArray<float3> JointOffsets;
            [ReadOnly] public NativeArray<quaternion> FrameLocalRotations;
            [ReadOnly] public float3 FrameRootMotion;
            [ReadOnly] public float3 HipsForwardLocalVector;

            // Output arrays are of size nJoints + 1 (including SimulationBone)
            [WriteOnly] public NativeArray<float3> OutJointLocalPositions;
            [WriteOnly] public NativeArray<quaternion> OutJointLocalRotations;

            public void Execute()
            {
                // Joints
                for (int i = 0; i < Njoints; i++)
                {
                    OutJointLocalPositions[i + 1] = JointOffsets[i];
                    OutJointLocalRotations[i + 1] = FrameLocalRotations[i];
                }
                // SimulationBone
                // position and direction are hips projected on the ground
                float3 sbPos = new float3(FrameRootMotion.x, 0.0f, FrameRootMotion.z);
                quaternion hipsForwardCorrection = MathExtensions.FromToRotation(math.mul(FrameLocalRotations[0], new float3(0, 0, 1)),
                                                                                 math.mul(FrameLocalRotations[0], HipsForwardLocalVector),
                                                                                 new float3(0, 1, 0));
                quaternion hipsOnlyY = math.normalize(MathExtensions.GetYAxisRotation(FrameLocalRotations[0]));
                quaternion sbRot = math.mul(math.normalize(MathExtensions.GetYAxisRotation(hipsForwardCorrection)), hipsOnlyY);
                OutJointLocalPositions[0] = sbPos;
                OutJointLocalRotations[0] = sbRot;
                // make first joint (hips) position and direction relative to the simulation bone
                OutJointLocalPositions[1] = math.mul(math.inverse(sbRot), FrameRootMotion - sbPos);
                OutJointLocalRotations[1] = math.mul(math.inverse(sbRot), FrameLocalRotations[0]);
            }
        }

        [BurstCompile]
        public struct ExtractVelocitiesBurst : IJob
        {
            // Input arrays include SimulationBone
            [ReadOnly] public int Njoints;
            [ReadOnly] public float FrameTime;
            [ReadOnly] public NativeArray<int> JointParents;
            [ReadOnly] public NativeArray<float3> LocalPositions;
            [ReadOnly] public NativeArray<float3> PrevLocalPositions;
            [ReadOnly] public NativeArray<quaternion> LocalRotations;
            [ReadOnly] public NativeArray<quaternion> PrevLocalRotations;

            // Output arrays include SimulationBone
            [WriteOnly] public NativeArray<float3> OutJointVelocities;
            [WriteOnly] public NativeArray<float3> OutJointAngularVelocities;

            public void Execute()
            {
                // Velocities
                for (int joint = 0; joint < Njoints; joint++)
                {
                    // Velocity
                    float3 pos = GetWorldPosition(joint, LocalPositions, LocalRotations);
                    float3 prevPos = GetWorldPosition(joint, PrevLocalPositions, PrevLocalRotations);
                    float3 vel = (pos - prevPos) / FrameTime;
                    OutJointVelocities[joint] = vel;
                    // Angular velocity
                    quaternion rot = GetWorldRotation(joint, LocalRotations);
                    quaternion prevRot = GetWorldRotation(joint, PrevLocalRotations);
                    float3 angularVel = MathExtensions.AngularVelocity(prevRot, rot, FrameTime);
                    OutJointAngularVelocities[joint] = angularVel;
                }
            }

            /// <summary>
            /// Apply forward kinematics to obtain the quaternion rotating from the local
            /// coordinate system of the joint to the world coordinate system.
            /// </summary>
            private quaternion GetWorldRotation(int joint, NativeArray<quaternion> localRotations)
            {
                quaternion worldRot = quaternion.identity;

                while (joint != 0) // while not root
                {
                    worldRot = math.mul(localRotations[joint], worldRot);
                    joint = JointParents[joint];
                }
                worldRot = math.mul(localRotations[0], worldRot); // root

                return worldRot;
            }

            /// <summary>
            /// Apply forward kinematics to obtain the position of the joint in the world coordinate system.
            /// </summary>
            private float3 GetWorldPosition(int joint, NativeArray<float3> localPositions, NativeArray<quaternion> localRotations)
            {
                float4x4 localToWorld = float4x4.identity;
                while (joint != 0) // while not root
                {
                    localToWorld = float4x4.TRS(localPositions[joint], localRotations[joint], new float3(1, 1, 1)) * localToWorld;
                    joint = JointParents[joint];
                }
                localToWorld = float4x4.TRS(localPositions[0], localRotations[0], new float3(1, 1, 1)) * localToWorld;
                return math.mul(localToWorld, new float4(0, 0, 0, 1)).xyz;
            }
        }
    }
}