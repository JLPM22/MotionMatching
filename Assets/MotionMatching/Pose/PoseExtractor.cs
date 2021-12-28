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
            Vector3[] jointLocalPositions = new Vector3[nJoints];
            Quaternion[] jointLocalRotations = new Quaternion[nJoints];
            for (int i = 0; i < nJoints; i++)
            {
                jointLocalPositions[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
                jointLocalRotations[i] = frame.LocalRotations[i];
            }
            return new PoseVector(jointLocalPositions, jointLocalRotations, frame.RootMotion);
        }
    }
}