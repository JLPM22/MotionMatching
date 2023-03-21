using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System.IO;
using System.Text;
using System;

namespace MotionMatching
{
    using static BinarySerializerExtensions;

    public class PoseSerializer
    {
        /// <summary>
        /// Stores the full pose representation of all poses for Motion Matching in a binary format
        /// in the specified path with name filename and extension .mmpose
        /// It also stores the skeleton used in poseSet with extension .mmskeleton
        /// </summary>
        public void Serialize(PoseSet poseSet, string path, string fileName)
        {
            Directory.CreateDirectory(path); // create directory and parent directories if they don't exist

            // Write Skeleton
            using (var stream = File.Open(Path.Combine(path, fileName + ".mmskeleton"), FileMode.Create))
            {
                using (var writer = new BinaryWriter(stream, System.Text.Encoding.UTF8))
                {
                    // Write Number Joints
                    writer.Write((uint)poseSet.Skeleton.Joints.Count);
                    // Write Joints
                    foreach (var joint in poseSet.Skeleton.Joints)
                    {
                        writer.Write(joint.Name);
                        writer.Write((uint)joint.Index);
                        writer.Write((uint)joint.ParentIndex);
                        WriteFloat3(writer, (float3)joint.LocalOffset);
                        writer.Write((uint)joint.Type);
                    }
                }
            }

            // Write Poses
            using (var stream = File.Open(Path.Combine(path, fileName + ".mmpose"), FileMode.Create))
            {
                using (var writer = new BinaryWriter(stream, System.Text.Encoding.UTF8))
                {
                    // Serialize Number Animation Clips
                    writer.Write((uint)poseSet.NumberClips);
                    // Serialize Animation Clips
                    for (int i = 0; i < poseSet.NumberClips; ++i)
                    {
                        PoseSet.AnimationClip clip = poseSet.GetClip(i);
                        writer.Write((uint)clip.Start);
                        writer.Write((uint)clip.End);
                        writer.Write(clip.FrameTime);
                    }
                    // Serialize Number Poses & Number Joints
                    writer.Write((uint)poseSet.NumberPoses);
                    writer.Write((uint)poseSet.Skeleton.Joints.Count);
                    // Serialize Poses
                    for (int i = 0; i < poseSet.NumberPoses; ++i)
                    {
                        poseSet.GetPose(i, out PoseVector pose);
                        WriteFloat3Array(writer, pose.JointLocalPositions);
                        WriteQuaternionArray(writer, pose.JointLocalRotations);
                        WriteFloat3Array(writer, pose.JointLocalVelocities);
                        WriteFloat3Array(writer, pose.JointLocalAngularVelocities);
                        writer.Write(pose.LeftFootContact ? 1u : 0u);
                        writer.Write(pose.RightFootContact ? 1u : 0u);
                    }
                }
            }
        }

        /// <summary>
        /// Reads the full pose representation of all poses for Motion Matching from a binary format
        /// in the specified path with name filename and extension .mmpose and .mmskeleton
        /// Returns true if poseSet was successfully deserialized, false otherwise
        /// </summary>
        public bool Deserialize(string path, string fileName, MotionMatchingData mmData, out PoseSet poseSet)
        {
            poseSet = new PoseSet(mmData);

            // Read Skeleton
            Skeleton skeleton = new Skeleton();
            string skeletonPath = Path.Combine(path, fileName + ".mmskeleton");
            if (File.Exists(skeletonPath))
            {
                using (var stream = File.Open(skeletonPath, FileMode.Open))
                {
                    using (var reader = new BinaryReader(stream, System.Text.Encoding.UTF8))
                    {
                        // Read Number Joints
                        uint nJoints = reader.ReadUInt32();
                        // Read Joints
                        for (int i = 0; i < nJoints; i++)
                        {
                            string jointName = reader.ReadString();
                            uint jointIndex = reader.ReadUInt32();
                            uint jointParentIndex = reader.ReadUInt32();
                            float3 jointLocalOffset = ReadFloat3(reader);
                            HumanBodyBones jointType = (HumanBodyBones)reader.ReadUInt32();
                            skeleton.AddJoint(new Skeleton.Joint(jointName, (int)jointIndex, (int)jointParentIndex, jointLocalOffset, jointType));
                        }
                    }
                }
            }
            else return false;
            // Set skeleton in poseSet
            poseSet.SetSkeletonFromFile(skeleton);

            // Read Poses
            string posePath = Path.Combine(path, fileName + ".mmpose");
            if (File.Exists(posePath))
            {
                using (var stream = File.Open(posePath, FileMode.Open))
                {
                    using (var reader = new BinaryReader(stream, System.Text.Encoding.UTF8))
                    {
                        // Deserialize Number Animation Clips
                        uint nClips = reader.ReadUInt32();
                        // Deserialize Animation Clips
                        for (int i = 0; i < nClips; i++)
                        {
                            uint start = reader.ReadUInt32();
                            uint end = reader.ReadUInt32();
                            float frameTime = reader.ReadSingle();
                            poseSet.AddAnimationClipUnsafe(new PoseSet.AnimationClip((int)start, (int)end, frameTime));
                        }
                        // Deserialize Number Poses & Number Joints
                        uint nPoses = reader.ReadUInt32();
                        uint nJoints = reader.ReadUInt32();
                        Debug.Assert(nJoints == skeleton.Joints.Count, "Number of joints in skeleton and pose do not match");
                        // Deserialize Poses
                        PoseVector[] poses = new PoseVector[nPoses];
                        for (int i = 0; i < nPoses; i++)
                        {
                            PoseVector pose = new PoseVector();
                            pose.JointLocalPositions = ReadFloat3Array(reader, nJoints);
                            pose.JointLocalRotations = ReadQuaternionArray(reader, nJoints);
                            pose.JointLocalVelocities = ReadFloat3Array(reader, nJoints);
                            pose.JointLocalAngularVelocities = ReadFloat3Array(reader, nJoints);
                            pose.LeftFootContact = reader.ReadUInt32() == 1u;
                            pose.RightFootContact = reader.ReadUInt32() == 1u;
                            poses[i] = pose;
                        }
                        // Set Poses in poseSet
                        poseSet.AddClipUnsafe(poses);
                    }
                }
            }
            else return false;

            return true;
        }
    }
}
