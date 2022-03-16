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
            BVHAnimation.Frame prevFrame = frameIndex == 0 ? frame : bvhAnimation.Frames[frameIndex - 1];

            int nJoints = bvhAnimation.Skeleton.Joints.Count;

            // Native Arrays used by Burst for Output
            NativeArray<float3> jointLocalPositions = new NativeArray<float3>(nJoints, Allocator.TempJob);
            NativeArray<quaternion> jointLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
            NativeArray<float3> jointVelocities = new NativeArray<float3>(nJoints, Allocator.TempJob);
            NativeArray<float3> jointAngularVelocities = new NativeArray<float3>(nJoints, Allocator.TempJob);
            NativeArray<float3> rootDisplacement = new NativeArray<float3>(1, Allocator.TempJob);
            NativeArray<quaternion> rootRotDisplacement = new NativeArray<quaternion>(1, Allocator.TempJob);
            NativeArray<float3> rootRotAngularVelocity = new NativeArray<float3>(1, Allocator.TempJob);


            // Native Arrays used by Burst for Input
            NativeArray<float3> jointOffsets = new NativeArray<float3>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) jointOffsets[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
            NativeArray<int> jointParents = new NativeArray<int>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) jointParents[i] = bvhAnimation.Skeleton.Joints[i].ParentIndex;
            NativeArray<quaternion> frameLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) frameLocalRotations[i] = frame.LocalRotations[i];
            NativeArray<quaternion> prevFrameLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) prevFrameLocalRotations[i] = prevFrame.LocalRotations[i];

            // Use Burst (way faster than doing it in regular C#)
            var job = new ExtractPoseBurst
            {
                // Input
                Njoints = nJoints,
                FrameTime = bvhAnimation.FrameTime,
                JointOffsets = jointOffsets,
                JointParents = jointParents,
                FrameLocalRotations = frameLocalRotations,
                PrevFrameLocalRotations = prevFrameLocalRotations,
                FrameRootMotion = frame.RootMotion,
                PrevFrameRootMotion = prevFrame.RootMotion,
                HipsForwardLocalVector = mmData.HipsForwardLocalVector,
                // Output
                JointLocalPositions = jointLocalPositions,
                JointLocalRotations = jointLocalRotations,
                JointVelocities = jointVelocities,
                JointAngularVelocities = jointAngularVelocities,
                RootDisplacement = rootDisplacement,
                RootRotDisplacement = rootRotDisplacement,
                RootRotAngularVelocity = rootRotAngularVelocity
            };
            job.Schedule().Complete();

            // Dispose Input Native Arrays
            jointOffsets.Dispose();
            jointParents.Dispose();
            frameLocalRotations.Dispose();
            prevFrameLocalRotations.Dispose();

            // Copy to managed arrays
            float3[] _jointLocalPositions = new float3[nJoints];
            for (int i = 0; i < nJoints; i++) _jointLocalPositions[i] = jointLocalPositions[i];
            quaternion[] _jointLocalRotations = new quaternion[nJoints];
            for (int i = 0; i < nJoints; i++) _jointLocalRotations[i] = jointLocalRotations[i];
            float3[] _jointVelocities = new float3[nJoints];
            for (int i = 0; i < nJoints; i++) _jointVelocities[i] = jointVelocities[i];
            float3[] _jointAngularVelocities = new float3[nJoints];
            for (int i = 0; i < nJoints; i++) _jointAngularVelocities[i] = jointAngularVelocities[i];
            float3 _rootDisplacement = float3.zero;
            _rootDisplacement = rootDisplacement[0];
            quaternion _rootRotDisplacement = quaternion.identity;
            _rootRotDisplacement = rootRotDisplacement[0];
            float3 _rootRotAngularVelocity = float3.zero;
            _rootRotAngularVelocity = rootRotAngularVelocity[0];

            // Dispose Output Native Arrays
            jointLocalPositions.Dispose();
            jointLocalRotations.Dispose();
            jointVelocities.Dispose();
            jointAngularVelocities.Dispose();
            rootDisplacement.Dispose();
            rootRotDisplacement.Dispose();
            rootRotAngularVelocity.Dispose();

            // Result
            return new PoseVector(_jointLocalPositions, _jointLocalRotations,
                                  _jointVelocities, _jointAngularVelocities,
                                  _rootDisplacement, _rootRotDisplacement, _rootRotAngularVelocity,
                                  frame.RootMotion, frame.LocalRotations[0]);
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
            [ReadOnly] public NativeArray<quaternion> PrevFrameLocalRotations;
            [ReadOnly] public float3 PrevFrameRootMotion;
            [ReadOnly] public float3 HipsForwardLocalVector;

            public NativeArray<float3> JointLocalPositions;
            [WriteOnly] public NativeArray<quaternion> JointLocalRotations;
            [WriteOnly] public NativeArray<float3> JointVelocities;
            [WriteOnly] public NativeArray<float3> JointAngularVelocities;
            [WriteOnly] public NativeArray<float3> RootDisplacement;
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
                    float3 prevPos = GetWorldPosition(joint, PrevFrameLocalRotations, PrevFrameRootMotion);
                    float3 vel = (pos - prevPos) / FrameTime;
                    JointVelocities[joint] = vel;
                    // Angular velocity
                    quaternion rot = GetWorldRotation(joint, FrameLocalRotations);
                    quaternion prevRot = GetWorldRotation(joint, PrevFrameLocalRotations);
                    if (joint == 0)
                    {
                        // Remove Y world axis rotation from the root...
                        // so angularVel is consistent with jointLocalRotations[0]
                        quaternion inverseRotY = math.inverse(MathExtensions.GetYAxisRotation(rot));
                        rot = math.mul(inverseRotY, rot);
                        quaternion inversePrevRotY = math.inverse(MathExtensions.GetYAxisRotation(prevRot));
                        prevRot = math.mul(inversePrevRotY, prevRot);
                    }
                    float3 angularVel = MathExtensions.AngularVelocity(prevRot, rot, FrameTime);
                    JointAngularVelocities[joint] = angularVel;
                }
                // Root: remove Y world axis rotation
                quaternion hipsRotWorldOnlyY = MathExtensions.GetYAxisRotation(FrameLocalRotations[0]);
                JointLocalRotations[0] = math.mul(math.inverse(hipsRotWorldOnlyY), FrameLocalRotations[0]);
                // Local Root Displacement
                FeatureSet.GetWorldOriginCharacter(FrameRootMotion, FrameLocalRotations[0], HipsForwardLocalVector, out _, out float3 characterForward);

                float3 hipsWorldXZ = new float3(FrameRootMotion.x, 0.0f, FrameRootMotion.z);
                float3 prevHipsWorldXZ = new float3(PrevFrameRootMotion.x, 0.0f, PrevFrameRootMotion.z);
                float3 tmpRootDisplacement = hipsWorldXZ - prevHipsWorldXZ;
                RootDisplacement[0] = FeatureSet.GetLocalDirectionFromCharacter(tmpRootDisplacement, characterForward);
                // Root Rot
                quaternion yRot = MathExtensions.GetYAxisRotation(FrameLocalRotations[0]);
                quaternion yPrevRot = MathExtensions.GetYAxisRotation(PrevFrameLocalRotations[0]);
                RootRotDisplacement[0] = math.mul(math.inverse(yPrevRot), yRot);
                RootRotAngularVelocity[0] = MathExtensions.AngularVelocity(yPrevRot, yRot, FrameTime);
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
                    localToWorld = float4x4.TRS(JointLocalPositions[parent], localRotations[parent], new float3(1, 1, 1)) * localToWorld;
                    joint = parent;
                }
                localToWorld = float4x4.TRS(rootMotion, localRotations[0], new float3(1, 1, 1)) * localToWorld;
                return math.mul(localToWorld, new float4(JointLocalPositions[joint], 1)).xyz;
            }
        }
    }
}