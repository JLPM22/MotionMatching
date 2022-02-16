using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
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
        public bool Extract(BVHAnimation bvhAnimation, PoseSet poseSet)
        {
            int nFrames = bvhAnimation.Frames.Length;
            PoseVector[] poses = new PoseVector[nFrames];
            for (int i = 0; i < nFrames; i++)
            {
                poses[i] = ExtractPose(bvhAnimation, i);
            }
            return poseSet.AddClip(bvhAnimation.Skeleton, poses, bvhAnimation.FrameTime);
        }

        private PoseVector ExtractPose(BVHAnimation bvhAnimation, int frameIndex)
        {
            BVHAnimation.Frame frame = bvhAnimation.Frames[frameIndex];
            int nJoints = bvhAnimation.Skeleton.Joints.Count;
            // Joints
            float3[] jointLocalPositions = new float3[nJoints];
            quaternion[] jointLocalRotations = new quaternion[nJoints];
            for (int i = 0; i < nJoints; i++)
            {
                jointLocalPositions[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
                jointLocalRotations[i] = frame.LocalRotations[i];
            }
            // Root: remove Y world axis rotation
            Quaternion hipsRotWorld = jointLocalRotations[0];
            float3 hipsRotEuler = hipsRotWorld.eulerAngles;
            float yRot = hipsRotEuler.y;
            hipsRotEuler.y = 0.0f;
            jointLocalRotations[0] = Quaternion.Euler(hipsRotEuler);
            // Local Root Velocity
            FeatureExtractor.GetWorldOriginCharacter(frame.RootMotion, hipsRotWorld, out _, out float3 characterForward);
            float3 rootVelocity = float3.zero;
            quaternion rootRotVelocity = Quaternion.identity;
            if (frameIndex > 0)
            {
                float3 hipsWorldXZ = new float3(frame.RootMotion.x, 0.0f, frame.RootMotion.z);
                float3 previousHipsWorldXZ = new float3(bvhAnimation.Frames[frameIndex - 1].RootMotion.x, 0.0f, bvhAnimation.Frames[frameIndex - 1].RootMotion.z);
                rootVelocity = hipsWorldXZ - previousHipsWorldXZ;
                rootVelocity = FeatureExtractor.GetLocalDirectionFromCharacter(rootVelocity, characterForward);
                float yRotPrevious = bvhAnimation.Frames[frameIndex - 1].LocalRotations[0].eulerAngles.y;
                rootRotVelocity = Quaternion.Inverse(Quaternion.Euler(0.0f, yRotPrevious, 0.0f)) * Quaternion.Euler(0.0f, yRot, 0.0f);
            }
            return new PoseVector(jointLocalPositions, jointLocalRotations, rootVelocity, rootRotVelocity, frame.RootMotion, hipsRotWorld);
        }
    }
}