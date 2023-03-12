using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    public class Skeleton
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

        public bool Find(HumanBodyBones type, out Joint joint)
        {
            for (int i = 0; i < Joints.Count; i++)
            {
                if (Joints[i].Type == type)
                {
                    joint = Joints[i];
                    return true;
                }
            }
            joint = new Joint();
            return false;
        }

        public bool Find(string jointName, out Joint joint)
        {
            for (int i = 0; i < Joints.Count; i++)
            {
                if (Joints[i].Name == jointName)
                {
                    joint = Joints[i];
                    return true;
                }
            }
            joint = new Joint();
            return true;
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
            public HumanBodyBones Type;

            public Joint(string name, int index, int parentIndex, Vector3 localOffset)
            {
                Name = name;
                Index = index;
                ParentIndex = parentIndex;
                LocalOffset = localOffset;
                Type = HumanBodyBones.LastBone;
            }

            public Joint(string name, int index, int parentIndex, Vector3 localOffset, HumanBodyBones type)
            {
                Name = name;
                Index = index;
                ParentIndex = parentIndex;
                LocalOffset = localOffset;
                Type = type;
            }

            public bool Equals(Joint other)
            {
                return Name == other.Name && Index == other.Index && ParentIndex == other.ParentIndex && LocalOffset == other.LocalOffset && Type == other.Type;
            }
        }
    }
}