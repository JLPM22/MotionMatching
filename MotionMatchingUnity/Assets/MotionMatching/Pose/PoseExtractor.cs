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
                poses[i] = ExtractPose(bvhAnimation, i, mmData, nFrames);
            }
            return poseSet.AddClip(bvhAnimation.Skeleton, poses, bvhAnimation.FrameTime);
        }

        private PoseVector ExtractPose(BVHAnimation bvhAnimation, int frameIndex, MotionMatchingData mmData, int nFrames)
        {
            BVHAnimation.Frame frame = bvhAnimation.Frames[frameIndex];
            int nJoints = bvhAnimation.Skeleton.Joints.Count;

            // Native Arrays used by Burst for Output
            NativeArray<float3> jointLocalPositions = new NativeArray<float3>(nJoints, Allocator.TempJob);
            NativeArray<quaternion> jointLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
            NativeArray<float3> jointVelocities = new NativeArray<float3>(nJoints, Allocator.TempJob);
            NativeArray<float3> jointAngularVelocities = new NativeArray<float3>(nJoints, Allocator.TempJob);
            NativeArray<float3> rootVelocity = new NativeArray<float3>(1, Allocator.TempJob);
            NativeArray<quaternion> rootRotDisplacement = new NativeArray<quaternion>(1, Allocator.TempJob);
            NativeArray<float3> rootRotAngularVelocity = new NativeArray<float3>(1, Allocator.TempJob);

            quaternion _hipsRotWorld = frame.LocalRotations[0]; // rotation before removing Y world axis rotation

            if (frameIndex < nFrames - 1) // we shouldn't use the last frame's root motion
            {
                BVHAnimation.Frame nextFrame = bvhAnimation.Frames[frameIndex + 1];

                // Native Arrays used by Burst for Input
                NativeArray<float3> jointOffsets = new NativeArray<float3>(nJoints, Allocator.TempJob);
                for (int i = 0; i < nJoints; i++) jointOffsets[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
                NativeArray<int> jointParents = new NativeArray<int>(nJoints, Allocator.TempJob);
                for (int i = 0; i < nJoints; i++) jointParents[i] = bvhAnimation.Skeleton.Joints[i].ParentIndex;
                NativeArray<quaternion> frameLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
                for (int i = 0; i < nJoints; i++) frameLocalRotations[i] = frame.LocalRotations[i];
                NativeArray<quaternion> nextFrameLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
                for (int i = 0; i < nJoints; i++) nextFrameLocalRotations[i] = nextFrame.LocalRotations[i];

                // Use Burst (way faster than doing it in regular C#)
                var job = new ExtractPoseBurst
                {
                    // Input
                    Njoints = nJoints,
                    FrameTime = bvhAnimation.FrameTime,
                    JointOffsets = jointOffsets,
                    JointParents = jointParents,
                    FrameLocalRotations = frameLocalRotations,
                    NextFrameLocalRotations = nextFrameLocalRotations,
                    FrameRootMotion = frame.RootMotion,
                    NextFrameRootMotion = nextFrame.RootMotion,
                    HipsRotWorld = _hipsRotWorld,
                    MinimumPoseVelocity = mmData.MinimumPoseVelocity,
                    HipsForwardLocalVector = mmData.HipsForwardLocalVector,
                    // Output
                    JointLocalPositions = jointLocalPositions,
                    JointLocalRotations = jointLocalRotations,
                    JointVelocities = jointVelocities,
                    JointAngularVelocities = jointAngularVelocities,
                    RootVelocity = rootVelocity,
                    RootRotDisplacement = rootRotDisplacement,
                    RootRotAngularVelocity = rootRotAngularVelocity
                };
                job.Schedule().Complete();

                // Dispose Input Native Arrays
                jointOffsets.Dispose();
                jointParents.Dispose();
                frameLocalRotations.Dispose();
                nextFrameLocalRotations.Dispose();
            }

            // Copy to managed arrays
            float3[] _jointLocalPositions = new float3[nJoints];
            for (int i = 0; i < nJoints; i++) _jointLocalPositions[i] = jointLocalPositions[i];
            quaternion[] _jointLocalRotations = new quaternion[nJoints];
            for (int i = 0; i < nJoints; i++) _jointLocalRotations[i] = jointLocalRotations[i];
            float3[] _jointVelocities = new float3[nJoints];
            for (int i = 0; i < nJoints; i++) _jointVelocities[i] = jointVelocities[i];
            float3[] _jointAngularVelocities = new float3[nJoints];
            for (int i = 0; i < nJoints; i++) _jointAngularVelocities[i] = jointAngularVelocities[i];
            float3 _rootVelocity = float3.zero;
            _rootVelocity = rootVelocity[0];
            quaternion _rootRotDisplacement = quaternion.identity;
            _rootRotDisplacement = rootRotDisplacement[0];
            float3 _rootRotAngularVelocity = float3.zero;
            _rootRotAngularVelocity = rootRotAngularVelocity[0];

            // Dispose Output Native Arrays
            jointLocalPositions.Dispose();
            jointLocalRotations.Dispose();
            jointVelocities.Dispose();
            jointAngularVelocities.Dispose();
            rootVelocity.Dispose();
            rootRotDisplacement.Dispose();
            rootRotAngularVelocity.Dispose();

            // Result
            return new PoseVector(_jointLocalPositions, _jointLocalRotations,
                                  _jointVelocities, _jointAngularVelocities,
                                  _rootVelocity, _rootRotDisplacement, _rootRotAngularVelocity,
                                  frame.RootMotion, _hipsRotWorld);
        }

        [BurstCompile]
        public struct ExtractPoseBurst : IJob
        {
            [ReadOnly] public int Njoints;
            [ReadOnly] public float FrameTime;
            [ReadOnly] public NativeArray<float3> JointOffsets;
            [ReadOnly] public NativeArray<int> JointParents;
            [ReadOnly] public NativeArray<quaternion> FrameLocalRotations;
            [ReadOnly] public float3 FrameRootMotion;
            [ReadOnly] public NativeArray<quaternion> NextFrameLocalRotations;
            [ReadOnly] public float3 NextFrameRootMotion;
            [ReadOnly] public quaternion HipsRotWorld;
            [ReadOnly] public float MinimumPoseVelocity;
            [ReadOnly] public float3 HipsForwardLocalVector;

            [WriteOnly] public NativeArray<float3> JointLocalPositions;
            [WriteOnly] public NativeArray<quaternion> JointLocalRotations;
            [WriteOnly] public NativeArray<float3> JointVelocities;
            [WriteOnly] public NativeArray<float3> JointAngularVelocities;
            [WriteOnly] public NativeArray<float3> RootVelocity;
            [WriteOnly] public NativeArray<quaternion> RootRotDisplacement;
            [WriteOnly] public NativeArray<float3> RootRotAngularVelocity;

            public void Execute()
            {
                // Joints
                for (int i = 0; i < Njoints; i++)
                {
                    JointLocalPositions[i] = JointOffsets[i];
                    JointLocalRotations[i] = FrameLocalRotations[i];
                }
                // Velocities
                for (int joint = 0; joint < Njoints; joint++)
                {
                    // Velocity
                    float3 pos = GetWorldPosition(joint, FrameLocalRotations, FrameRootMotion);
                    float3 posNext = GetWorldPosition(joint, NextFrameLocalRotations, NextFrameRootMotion);
                    float3 vel = (posNext - pos) / FrameTime;
                    JointVelocities[joint] = vel;
                    // Angular velocity
                    quaternion rot = GetWorldRotation(joint, FrameLocalRotations);
                    quaternion rotNext = GetWorldRotation(joint, NextFrameLocalRotations);
                    if (joint == 0)
                    {
                        // Remove Y world axis rotation from the root...
                        // so angularVel is consistent with jointLocalRotations[0]
                        quaternion inverseRotY = math.inverse(MathExtensions.GetYAxisRotation(rot));
                        rot = math.mul(inverseRotY, rotNext);
                        quaternion inverseRotNextY = math.inverse(MathExtensions.GetYAxisRotation(rotNext));
                        rotNext = math.mul(inverseRotNextY, rotNext);
                    }
                    // Source: https://theorangeduck.com/page/exponential-map-angle-axis-angular-velocity
                    float3 angularVel = MathExtensions.AngularVelocity(rot, rotNext, FrameTime);
                    JointAngularVelocities[joint] = angularVel;
                }
                // Root: remove Y world axis rotation
                quaternion hipsRotWorldOnlyY = MathExtensions.GetYAxisRotation(HipsRotWorld);
                quaternion inverseHipsRotWorldY = math.inverse(hipsRotWorldOnlyY);
                JointLocalRotations[0] = math.mul(inverseHipsRotWorldY, HipsRotWorld);
                // Local Root Displacement
                FeatureExtractor.GetWorldOriginCharacter(FrameRootMotion, HipsRotWorld, HipsForwardLocalVector, out _, out float3 characterForward);

                float3 hipsWorldXZ = new float3(FrameRootMotion.x, 0.0f, FrameRootMotion.z);
                float3 nextHipsWorldXZ = new float3(NextFrameRootMotion.x, 0.0f, NextFrameRootMotion.z);
                float3 tmpRootVelocity = nextHipsWorldXZ - hipsWorldXZ;
                if (math.length(tmpRootVelocity) < MinimumPoseVelocity) tmpRootVelocity = float3.zero; // Clamp Velocity if too small to avoid small numerical errors to move the character
                RootVelocity[0] = FeatureExtractor.GetLocalDirectionFromCharacter(tmpRootVelocity, characterForward);
                // Root Rot
                quaternion yRotNext = MathExtensions.GetYAxisRotation(NextFrameLocalRotations[0]);
                RootRotDisplacement[0] = math.mul(inverseHipsRotWorldY, yRotNext);
                RootRotAngularVelocity[0] = MathExtensions.AngularVelocity(hipsRotWorldOnlyY, yRotNext, FrameTime);
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
            private float3 GetWorldPosition(int joint, NativeArray<quaternion> localRotations, float3 rootMotion)
            {
                float4x4 localToWorld = float4x4.identity;
                while (JointParents[joint] != 0) // while not root
                {
                    int parent = JointParents[joint];
                    localToWorld = float4x4.TRS(JointOffsets[parent], localRotations[parent], new float3(1, 1, 1)) * localToWorld;
                    joint = parent;
                }
                localToWorld = float4x4.TRS(rootMotion, localRotations[0], new float3(1, 1, 1)) * localToWorld;
                return math.mul(localToWorld, new float4(JointOffsets[joint], 1)).xyz;
            }
        }
    }
}