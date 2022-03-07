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
            Directory.CreateDirectory(path); // creatae directory and parent directories if they don't exist

            // Write Skeleton
            using (var stream = File.Open(Path.Combine(path, fileName + ".mmskeleton"), FileMode.Create))
            {
                using (var writer = new BinaryWriter(stream))
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
                using (var writer = new BinaryWriter(stream))
                {
                    // Serialize Number Animation Clips
                    writer.Write((uint)poseSet.Clips.Count);
                    // Serialize Animation Clips
                    foreach (var clip in poseSet.Clips)
                    {
                        writer.Write((uint)clip.Start);
                        writer.Write((uint)clip.End);
                        writer.Write(clip.FrameTime);
                    }
                    // Serialize Number Poses & Number Joints
                    writer.Write((uint)poseSet.Poses.Count);
                    writer.Write((uint)poseSet.Skeleton.Joints.Count);
                    // Serialize Poses
                    foreach (var pose in poseSet.Poses)
                    {
                        WriteFloat3Array(writer, pose.JointLocalPositions);
                        WriteQuaternionArray(writer, pose.JointLocalRotations);
                        WriteFloat3Array(writer, pose.JointVelocities);
                        WriteFloat3Array(writer, pose.JointAngularVelocities);
                        WriteFloat3(writer, pose.RootDisplacement);
                        WriteQuaternion(writer, pose.RootRotDisplacement);
                        WriteFloat3(writer, pose.RootRotAngularVelocity);
                        WriteFloat3(writer, pose.RootWorld);
                        WriteQuaternion(writer, pose.RootWorldRot);
                    }
                }
            }
        }

        /// <summary>
        /// Reads the full pose representation of all poses for Motion Matching from a binary format
        /// in the specified path with name filename and extension .mmpose and .mmskeleton
        /// Returns true if poseSet was successfully deserialized, false otherwise
        /// </summary>
        public bool Deserialize(string path, string filename, out PoseSet poseSet)
        {
            poseSet = new PoseSet();

            // Read Skeleton
            Skeleton skeleton = new Skeleton();
            string skeletonPath = Path.Combine(path, filename + ".mmskeleton");
            if (File.Exists(skeletonPath))
            {
                using (var stream = File.Open(skeletonPath, FileMode.Open))
                {
                    using (var reader = new BinaryReader(stream))
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
            poseSet.SetSkeleton(skeleton);

            // Read Poses
            string posePath = Path.Combine(path, filename + ".mmpose");
            if (File.Exists(posePath))
            {
                using (var stream = File.Open(posePath, FileMode.Open))
                {
                    using (var reader = new BinaryReader(stream))
                    {
                        // Deserialize Number Animation Clips
                        uint nClips = reader.ReadUInt32();
                        Debug.Assert(nClips == 1, "Only one animation clip is supported"); // TODO: support more animation clips
                        // Deserialize Animation Clips
                        float frameTime = 0.0f;
                        for (int i = 0; i < nClips; i++)
                        {
                            // TODO: for now we ignore animation clips...
                            uint start = reader.ReadUInt32();
                            uint end = reader.ReadUInt32();
                            frameTime = reader.ReadSingle();
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
                            pose.JointVelocities = ReadFloat3Array(reader, nJoints);
                            pose.JointAngularVelocities = ReadFloat3Array(reader, nJoints);
                            pose.RootDisplacement = ReadFloat3(reader);
                            pose.RootRotDisplacement = ReadQuaternion(reader);
                            pose.RootRotAngularVelocity = ReadFloat3(reader);
                            pose.RootWorld = ReadFloat3(reader);
                            pose.RootWorldRot = ReadQuaternion(reader);
                            poses[i] = pose;
                        }
                        // Set Poses in poseSet
                        poseSet.AddClip(poses, frameTime);
                    }
                }
            }
            else return false;

            return true;
        }
    }
}
