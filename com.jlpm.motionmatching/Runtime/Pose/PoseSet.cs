using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
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
        public int NumberTags { get { return Tags.Count; } }

        public readonly int MaximumFramesPrediction; // Number of prediction frames of the longest trajectory feature

        // Private ---
        private readonly List<PoseVector> Poses;
        private readonly List<AnimationClip> Clips;
        private readonly List<Tag> Tags;
        private readonly Dictionary<string, int> TagNameToIndex;

        public PoseSet(MotionMatchingData mmData)
        {
            Poses = new List<PoseVector>();
            Clips = new List<AnimationClip>();
            Tags = new List<Tag>();
            TagNameToIndex = new Dictionary<string, int>();
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
        public bool AddClip(PoseVector[] poses, float frameTime, out int animationClip)
        {
            // Check if the skeleton and frameTime are compatible
            Debug.Assert(Skeleton != null, "Skeleton shold be set first. Use SetSkeleton(...)");
            if (FrameTime == -1.0f) FrameTime = frameTime;
            Debug.Assert(math.abs(FrameTime - frameTime) < 0.001f, "Frame time should be the same for all clips");

            // Add poses
            int start = Poses.Count;
            int nPoses = poses.Length;

            animationClip = Clips.Count;
            Clips.Add(new AnimationClip(start, start + nPoses, frameTime));
            Poses.AddRange(poses);

            return true;
        }

        /// <summary>
        /// Add a tag to the current pose set
        /// The corresponding animation clip should be added before using AddTag(...)
        /// </summary>
        public void AddTag(int animationClip, AnimationData.Tag dataTag)
        {
            // Tag Index
            if (!TagNameToIndex.TryGetValue(dataTag.Name, out int tagIndex))
            {
                tagIndex = Tags.Count;
                TagNameToIndex[dataTag.Name] = tagIndex;
                Tags.Add(new Tag(dataTag.Name));
            }
            // Write tag ranges
            Tag tag = Tags[tagIndex];
            int frameOffset = Clips[animationClip].Start;
            for (int i = 0; i < dataTag.Start.Length; ++i)
            {
                tag.AddRange(dataTag.Start[i] + frameOffset, dataTag.End[i] + frameOffset);
            }
        }

        /// <summary>
        /// Add a tag to the current pose set
        /// Used when deserializing from binary format
        /// </summary>
        public void AddTagDeserialized(string name, List<int> startRangesList, List<int> endRangesList)
        {
            TagNameToIndex[name] = Tags.Count;
            Tags.Add(new Tag(name, startRangesList, endRangesList));
        }

        /// <summary>
        /// Converts all tags-related data stored in C# data structures to NativeArrays
        /// Use this function after adding all tags with AddTag(...)
        /// </summary>
        public void ConvertTagsToNativeArrays()
        {
            foreach (Tag tag in Tags)
            {
                tag.ConvertToNativeArray();
            }
        }

        /// <summary>
        /// Add the set of poses to the current pose set
        /// Used when deserializing from binary format
        /// </summary>
        public void AddClipDeserialized(PoseVector[] poses)
        {
            Poses.AddRange(poses);
        }

        /// <summary>
        /// Add the animation clip to the current clips
        /// Used when deserializing from binary format
        /// </summary>
        public void AddAnimationClipDeserialized(AnimationClip clip)
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
        /// Return true if the pose can be used for prediction
        /// </summary>
        public bool GetPose(int poseIndex, out PoseVector pose)
        {
            bool isPredictionSafe = IsPoseValidForPrediction(poseIndex);
            pose = Poses[poseIndex];
            return isPredictionSafe;
        }

        /// <summary>
        /// Returns the pose at the given index.
        /// Return true if the pose can be used for prediction
        /// </summary>
        public bool GetPose(int poseIndex, out PoseVector pose, out int animationClip)
        {
            animationClip = -1;
            for (int clipIdx = 0; clipIdx < Clips.Count; ++clipIdx)
            {
                if (poseIndex >= Clips[clipIdx].Start && poseIndex < Clips[clipIdx].End)
                {
                    animationClip = clipIdx;
                    break;
                }
            }
            Debug.Assert(animationClip != -1, "Clip index not found");
            return GetPose(poseIndex, out pose);
        }

        /// <summary>
        /// Returns the tag at the given index
        /// </summary>
        public Tag GetTag(int index)
        {
            return Tags[index];
        }

        /// <summary>
        /// Returns the tag with the given name
        /// </summary>
        public Tag GetTag(string name)
        {
            return Tags[TagNameToIndex[name]];
        }

        /// <summary>
        /// Returns the animation clip at the given index
        /// </summary>
        public AnimationClip GetAnimationClip(int clipIndex)
        {
            Debug.Assert(clipIndex >= 0 && clipIndex < Clips.Count, "Clip index out of range");
            return Clips[clipIndex];
        }

        public void Dispose()
        {
            if (Tags != null)
            {
                foreach (Tag tag in Tags) 
                {
                    tag.Dispose();
                }
            }
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

        public class Tag
        {
            public readonly string Name;

            private List<int> StartRangesList; // Temporal lists until they are converted to NativeArrays
            private List<int> EndRangesList;

            private NativeArray<int> StartRanges;
            private NativeArray<int> EndRanges;

            public int NumberRanges { get { return StartRanges.Length; } }

            public Tag(string name)
            {
                Name = name;
                StartRangesList = new List<int>();
                EndRangesList = new List<int>();
            }

            public Tag(string name, List<int> startRangesList, List<int> endRangesList)
            {
                Name = name;
                StartRangesList = startRangesList;
                EndRangesList = endRangesList;
            }

            public void AddRange(int start, int end)
            {
                StartRangesList.Add(start);
                EndRangesList.Add(end);
            }

            public NativeArray<int> GetStartRanges()
            {
                return StartRanges;
            }

            public NativeArray<int> GetEndRanges()
            {
                return EndRanges;
            }

            public void ConvertToNativeArray()
            {
                StartRanges = new NativeArray<int>(StartRangesList.ToArray(), Allocator.Persistent);
                EndRanges = new NativeArray<int>(EndRangesList.ToArray(), Allocator.Persistent);

                StartRangesList = null;
                EndRangesList = null;
            }

            public void GetRange(int rangeIndex, out int start, out int end)
            {
                Debug.Assert(StartRanges.IsCreated && EndRanges.IsCreated, "Call first ConvertToNativeArray() before operating over the tags.");
                start = StartRanges[rangeIndex];
                end = EndRanges[rangeIndex];
            }

            public void Dispose()
            {
                if (StartRanges != null && StartRanges.IsCreated) StartRanges.Dispose();
                if (EndRanges != null && EndRanges.IsCreated) EndRanges.Dispose();
            }
        }
    }
}