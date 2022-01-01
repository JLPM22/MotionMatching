using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
            Vector3[] jointLocalPositions = new Vector3[nJoints];
            Quaternion[] jointLocalRotations = new Quaternion[nJoints];
            for (int i = 0; i < nJoints; i++)
            {
                jointLocalPositions[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
                jointLocalRotations[i] = frame.LocalRotations[i];
            }
            jointLocalPositions[0] = frame.RootMotion;
            // Local Root Velocity
            Vector3 hipsWorld = jointLocalPositions[0];
            Quaternion hipsRotWorld = jointLocalRotations[0];
            FeatureExtractor.GetWorldOriginCharacter(hipsWorld, hipsRotWorld, out _, out Vector3 characterForward);
            Vector3 rootVelocity = Vector3.zero;
            Quaternion rootRotVelocity = Quaternion.identity;
            if (frameIndex > 0)
            {
                rootVelocity = hipsWorld - bvhAnimation.Frames[frameIndex - 1].RootMotion;
                rootVelocity = FeatureExtractor.GetLocalDirectionFromCharacter(rootVelocity, characterForward);
                rootRotVelocity = Quaternion.Inverse(bvhAnimation.Frames[frameIndex - 1].LocalRotations[0]) * hipsRotWorld;
            }
            return new PoseVector(jointLocalPositions, jointLocalRotations, rootVelocity, rootRotVelocity);
        }
    }
}