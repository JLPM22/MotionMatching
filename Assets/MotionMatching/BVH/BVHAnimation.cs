using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    public class BVHAnimation
    {
        public float FrameTime { get; private set; }
        public List<Joint> Skeleton { get; private set; }
        public List<EndSite> EndSites { get; private set; }
        public Frame[] Frames { get; private set; }

        public BVHAnimation()
        {
            Skeleton = new List<Joint>();
            EndSites = new List<EndSite>();
        }

        public void SetFrameTime(float frameTime)
        {
            FrameTime = frameTime;
        }

        public void InitFrames(int numberFrames)
        {
            Frames = new Frame[numberFrames];
        }

        public void AddFrame(int index, Frame frame)
        {
            Frames[index] = frame;
        }

        public void AddJoint(Joint joint)
        {
            Skeleton.Add(joint);
        }

        public void AddEndSite(EndSite endSite)
        {
            EndSites.Add(endSite);
        }

        public struct Joint
        {
            public string Name;
            public int Index;
            public int ParentIndex; // 0 - Root
            public Vector3 Offset;

            public Joint(string name, int index, int parentIndex, Vector3 offset)
            {
                Name = name;
                Index = index;
                ParentIndex = parentIndex;
                Offset = offset;
            }
        }

        public struct EndSite
        {
            public int ParentIndex;
            public Vector3 Offset;

            public EndSite(int parentIndex, Vector3 offset)
            {
                ParentIndex = parentIndex;
                Offset = offset;
            }
        }

        public struct Frame
        {
            public Vector3 RootMotion;
            public Quaternion[] LocalRotations;

            public Frame(Vector3 rootMotion, Quaternion[] localRotations)
            {
                RootMotion = rootMotion;
                LocalRotations = localRotations;
            }
        }
    }
}