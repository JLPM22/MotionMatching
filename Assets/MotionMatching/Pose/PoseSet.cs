using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    /// <summary>
    /// Stores the full pose representation of all poses for Motion Matching
    /// </summary>
    public class PoseSet
    {
        public Skeleton Skeleton { get; private set; }
        public List<PoseVector> Poses { get; private set; }
        public List<AnimationClip> Clips { get; private set; }

        public PoseSet()
        {
            Poses = new List<PoseVector>();
            Clips = new List<AnimationClip>();
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
            Debug.Assert(skeleton != null, "Skeleton shold be set first. Use SetSkeleton(...)");

            if (!IsSkeletonCompatible(skeleton)) return false;

            int start = Poses.Count;
            int nPoses = poses.Length;

            Clips.Add(new AnimationClip(start, start + nPoses, frameTime));
            Poses.AddRange(poses);

            return true;
        }

        /// <summary>
        /// Add the animation clip to the current pose set
        /// Returns true if the clip was added, false otherwise
        /// </summary>
        public bool AddClip(PoseVector[] poses, float frameTime)
        {
            int start = Poses.Count;
            int nPoses = poses.Length;

            Clips.Add(new AnimationClip(start, start + nPoses, frameTime));
            Poses.AddRange(poses);

            return true;
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
            public int Start;
            public int End;
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