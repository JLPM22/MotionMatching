using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

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
            float3[] jointLocalPositions = new float3[nJoints];
            quaternion[] jointLocalRotations = new quaternion[nJoints];
            float3[] jointVelocities = new float3[nJoints];
            float3[] jointAngularVelocities = new float3[nJoints];
            float3 rootVelocity = float3.zero;
            quaternion rootRotDisplacement = quaternion.identity;
            float3 rootRotAngularVelocity = float3.zero;
            quaternion hipsRotWorld = frame.LocalRotations[0]; // rotation before removing Y world axis rotation
            if (frameIndex < nFrames - 1) // we shouldn't use the last frame's root motion
            {
                BVHAnimation.Frame nextFrame = bvhAnimation.Frames[frameIndex + 1];
                // Joints
                for (int i = 0; i < nJoints; i++)
                {
                    jointLocalPositions[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
                    jointLocalRotations[i] = frame.LocalRotations[i];
                }
                // Velocities
                for (int i = 0; i < nJoints; i++)
                {
                    Joint joint = bvhAnimation.Skeleton.Joints[i];
                    // Velocity
                    float3 pos = bvhAnimation.GetWorldPosition(joint, frameIndex);
                    float3 posNext = bvhAnimation.GetWorldPosition(joint, frameIndex + 1);
                    float3 vel = (posNext - pos) / bvhAnimation.FrameTime;
                    jointVelocities[i] = vel;
                    // Angular velocity
                    quaternion rot = bvhAnimation.GetWorldRotation(joint, frameIndex);
                    quaternion rotNext = bvhAnimation.GetWorldRotation(joint, frameIndex + 1);
                    if (i == 0)
                    {
                        // Remove Y world axis rotation from the root...
                        // so angularVel is consistent with jointLocalRotations[0]
                        quaternion inverseRotY = math.inverse(MathExtensions.GetYAxisRotation(rot));
                        rot = math.mul(inverseRotY, rotNext);
                        quaternion inverseRotNextY = math.inverse(MathExtensions.GetYAxisRotation(rotNext));
                        rotNext = math.mul(inverseRotNextY, rotNext);
                    }
                    // Source: https://theorangeduck.com/page/exponential-map-angle-axis-angular-velocity
                    float3 angularVel = MathExtensions.AngularVelocity(rot, rotNext, bvhAnimation.FrameTime);
                    jointAngularVelocities[i] = angularVel;
                }
                // Root: remove Y world axis rotation
                quaternion hipsRotWorldOnlyY = MathExtensions.GetYAxisRotation(hipsRotWorld);
                quaternion inverseHipsRotWorldY = math.inverse(hipsRotWorldOnlyY);
                jointLocalRotations[0] = math.mul(inverseHipsRotWorldY, hipsRotWorld);
                // Local Root Displacement
                FeatureExtractor.GetWorldOriginCharacter(frame.RootMotion, hipsRotWorld, mmData.HipsForwardLocalVector, out _, out float3 characterForward);

                float3 hipsWorldXZ = new float3(frame.RootMotion.x, 0.0f, frame.RootMotion.z);
                float3 nextHipsWorldXZ = new float3(nextFrame.RootMotion.x, 0.0f, nextFrame.RootMotion.z);
                rootVelocity = nextHipsWorldXZ - hipsWorldXZ;
                if (math.length(rootVelocity) < mmData.MinimumPoseVelocity) rootVelocity = float3.zero; // Clamp Velocity if too small to avoid small numerical errors to move the character
                rootVelocity = FeatureExtractor.GetLocalDirectionFromCharacter(rootVelocity, characterForward);
                // Root Rot
                quaternion yRotNext = MathExtensions.GetYAxisRotation(nextFrame.LocalRotations[0]);
                rootRotDisplacement = math.mul(inverseHipsRotWorldY, yRotNext);
                rootRotAngularVelocity = MathExtensions.AngularVelocity(hipsRotWorldOnlyY, yRotNext, bvhAnimation.FrameTime);
            }
            return new PoseVector(jointLocalPositions, jointLocalRotations,
                                  jointVelocities, jointAngularVelocities,
                                  rootVelocity, rootRotDisplacement, rootRotAngularVelocity,
                                  frame.RootMotion, hipsRotWorld);
        }
    }
}