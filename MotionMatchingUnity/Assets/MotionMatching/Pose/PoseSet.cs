using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    /// <summary>
    /// Stores the full pose representation of all poses for Motion Matching
    /// </summary>
    public class PoseSet
    {
        // Public ---
        public Skeleton Skeleton { get; private set; }
        public float FrameTime { get; private set; }
        public int NumberPoses { get { return Poses.Count; } }
        public int NumberClips { get { return Clips.Count; } }

        // Private ---
        private List<PoseVector> Poses;
        private List<AnimationClip> Clips;

        public PoseSet()
        {
            Poses = new List<PoseVector>();
            Clips = new List<AnimationClip>();
            FrameTime = -1.0f;
        }

        public void SetSkeleton(Skeleton skeleton)
        {
            Skeleton = skeleton;
        }

        /// <summary>
        /// Add the animation clip to the current pose set
        /// Returns true if the clip was added, false if the skeleton is not compatible and the clip was not added
        /// </summary>
        public bool AddClip(Skeleton skeleton, PoseVector[] poses, float frameTime)
        {
            // Check if the skeleton and frameTime are compatible
            Debug.Assert(skeleton != null, "Skeleton shold be set first. Use SetSkeleton(...)");
            if (FrameTime == -1.0f) FrameTime = frameTime;
            Debug.Assert(math.abs(FrameTime - frameTime) < 0.001f, "Frame time should be the same for all clips");
            if (!IsSkeletonCompatible(skeleton)) return false;

            // Add poses
            int start = Poses.Count;
            int nPoses = poses.Length;

            Clips.Add(new AnimationClip(start, start + nPoses, frameTime));
            Poses.AddRange(poses);

            return true;
        }

        /// <summary>
        /// Add the set of poses to the current pose set
        /// Unsafe version used when deserializing from binary format
        /// </summary>
        public void AddClipUnsafe(PoseVector[] poses)
        {
            Poses.AddRange(poses);
        }

        /// <summary>
        /// Add the animation clip to the current clips
        /// Unsafe version used when deserializing from binary format
        /// </summary>
        public void AddAnimationClipUnsafe(AnimationClip clip)
        {
            if (FrameTime == -1.0f) FrameTime = clip.FrameTime;
            Clips.Add(clip);
        }

        public bool IsPoseValidForPrediction(int poseIndex)
        {
            Debug.Assert(poseIndex >= 0 && poseIndex < Poses.Count, "Pose index out of range");
            // Check the validity of the pose
            bool isPredictionSafe = true;
            for (int i = 0; i < Clips.Count && isPredictionSafe; ++i)
            {
                AnimationClip clip = Clips[i];
                if (poseIndex >= clip.Start && poseIndex < clip.End)
                {
                    if (poseIndex >= clip.End - 60) isPredictionSafe = false;
                }
            }
            return isPredictionSafe;
        }

        /// <summary>
        /// Returns the pose at the given index.
        /// HARDCODED: return true if the pose can be used for prediction (i.e. the pose is not in the last 60 frames of the clip)
        /// </summary>
        public bool GetPose(int poseIndex, out PoseVector pose)
        {
            bool isPredictionSafe = IsPoseValidForPrediction(poseIndex);
            pose = Poses[poseIndex];
            return isPredictionSafe;
        }

        public AnimationClip GetClip(int clipIndex)
        {
            Debug.Assert(clipIndex >= 0 && clipIndex < Clips.Count, "Clip index out of range");
            return Clips[clipIndex];
        }

        /// <summary>
        /// True if the animation clip can be added to the pose set, False otherwise
        /// </summary>
        private bool IsSkeletonCompatible(Skeleton skeleton)
        {
            return Skeleton == null || Skeleton.Equals(skeleton);
        }

        public struct AnimationClip
        {
            public int Start; // Index of the first pose in the clip
            public int End; // End is exclusive
            public float FrameTime;

            public AnimationClip(int start, int end, float frameTime)
            {
                Start = start;
                End = end;
                FrameTime = frameTime;
            }
        }
    }
}