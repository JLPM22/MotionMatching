using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;

    /// <summary>
    /// Stores the full pose representation of all poses for Motion Matching
    /// </summary>
    public class PoseSet
    {
        // Public ---
        public Skeleton Skeleton { get; private set; } // No simulation bone
        public float FrameTime { get; private set; }
        public int NumberPoses { get { return Poses.Count; } }
        public int NumberClips { get { return Clips.Count; } }

        // Private ---
        private List<PoseVector> Poses;
        private List<AnimationClip> Clips;
        private int MaximumFramesPrediction;

        public PoseSet(MotionMatchingData mmData)
        {
            Poses = new List<PoseVector>();
            Clips = new List<AnimationClip>();
            FrameTime = -1.0f;
            MaximumFramesPrediction = 0;
            foreach (var t in mmData.TrajectoryFeatures)
            {
                if (t.FramesPrediction[t.FramesPrediction.Length - 1] > MaximumFramesPrediction)
                {
                    MaximumFramesPrediction = t.FramesPrediction[t.FramesPrediction.Length - 1];
                }
            }
        }

        /// <summary>
        /// Set skeleton from BVH. Adds simulation bone as root joint
        /// </summary>
        public void SetSkeletonFromBVH(Skeleton skeleton)
        {
            Skeleton = new Skeleton();
            // Add Simulation Bone
            Joint sb = new Joint("SimulationBone", 0, 0, Vector3.zero);
            Skeleton.AddJoint(sb);
            // Add Joints (adjusting indices, now SimulationBone is 0 and all indices are shifted by 1)
            for (int i = 0; i < skeleton.Joints.Count; ++i)
            {
                Joint j = skeleton.Joints[i];
                j.Index = j.Index + 1;
                if (i == 0) // Root
                {
                    j.ParentIndex = 0;
                }
                else // Other Joints
                {
                    j.ParentIndex = j.ParentIndex + 1;
                }
                Skeleton.Joints.Add(j);
            }
        }

        /// <summary>
        /// Set skeleton from File. The skeleton is not modified
        /// </summary>
        public void SetSkeletonFromFile(Skeleton skeleton)
        {
            Skeleton = skeleton;
        }

        /// <summary>
        /// Add the animation clip to the current pose set
        /// Returns true if the clip was added, false if the skeleton is not compatible and the clip was not added
        /// </summary>
        public bool AddClip(PoseVector[] poses, float frameTime)
        {
            // Check if the skeleton and frameTime are compatible
            Debug.Assert(Skeleton != null, "Skeleton shold be set first. Use SetSkeleton(...)");
            if (FrameTime == -1.0f) FrameTime = frameTime;
            Debug.Assert(math.abs(FrameTime - frameTime) < 0.001f, "Frame time should be the same for all clips");

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
            Debug.Assert(math.abs(FrameTime + 1.0f) < 0.001f || math.abs(clip.FrameTime - FrameTime) < 0.001f, "Mixed frame rates");
            FrameTime = clip.FrameTime;
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
                    if (poseIndex >= clip.End - MaximumFramesPrediction) isPredictionSafe = false;
                }
            }
            return isPredictionSafe;
        }

        /// <summary>
        /// Returns the pose at the given index.
        /// Teturn true if the pose can be used for prediction
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