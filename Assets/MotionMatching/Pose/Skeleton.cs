using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    public class Skeleton : IEquatable<Skeleton>
    {
        public List<Joint> Joints { get; private set; }

        public Skeleton()
        {
            Joints = new List<Joint>();
        }

        public void AddJoint(Joint joint)
        {
            Joints.Add(joint);
        }

        public bool Equals(Skeleton other)
        {
            for (int i = 0; i < Joints.Count; i++)
            {
                if (!Joints[i].Equals(other.Joints[i]))
                {
                    return false;
                }
            }
            return true;
        }

        public Joint Find(JointType type)
        {
            for (int i = 0; i < Joints.Count; i++)
            {
                if (Joints[i].Type == type) return Joints[i];
            }
            Debug.Assert(false, "This skeleton does not contain any joint of type " + type);
            return new Joint();
        }

        public Joint GetParent(Joint joint)
        {
            return Joints[joint.ParentIndex];
        }

        public struct Joint : IEquatable<Joint>
        {
            public string Name;
            public int Index;
            public int ParentIndex; // 0 - Root
            public Vector3 LocalOffset;
            public JointType Type;

            public Joint(string name, int index, int parentIndex, Vector3 localOffset)
            {
                Name = name;
                Index = index;
                ParentIndex = parentIndex;
                LocalOffset = localOffset;
                Type = JointType.Other;
            }

            public bool Equals(Joint other)
            {
                return Name == other.Name && Index == other.Index && ParentIndex == other.ParentIndex && LocalOffset == other.LocalOffset && Type == other.Type;
            }
        }

        public enum JointType
        {
            Hips,
            LeftFoot,
            RightFoot,
            Other
        }
    }
}