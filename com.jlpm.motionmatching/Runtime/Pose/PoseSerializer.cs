using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System.IO;
using System.Text;
using System;
using System.Runtime.InteropServices;

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
                        PoseSet.AnimationClip clip = poseSet.GetAnimationClip(i);
                        writer.Write((uint)clip.Start);
                        writer.Write((uint)clip.End);
                        writer.Write(clip.FrameTime);
                    }
                    // Serialize Number Poses & Number Joints & Number Tags
                    writer.Write((uint)poseSet.NumberPoses);
                    writer.Write((uint)poseSet.Skeleton.Joints.Count);
                    writer.Write((uint)poseSet.NumberTags);
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
                    // Serialize Tags
                    for (int i = 0; i < poseSet.NumberTags; ++i)
                    {
                        PoseSet.Tag tag = poseSet.GetTag(i);
                        writer.Write(tag.Name);
                        writer.Write((uint)tag.NumberRanges);
                        for (int r = 0; r < tag.NumberRanges; ++r)
                        {
                            tag.GetRange(r, out int startRange, out int endRange);
                            writer.Write((uint)startRange);
                            writer.Write((uint)endRange);
                        }
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

            // --------------------
            // Read Skeleton File
            // --------------------
            string skeletonPath = Path.Combine(path, fileName + ".mmskeleton");
            if (!File.Exists(skeletonPath))
                return false;

            Skeleton skeleton = new Skeleton();
            byte[] skeletonData = File.ReadAllBytes(skeletonPath);
            using (var ms = new MemoryStream(skeletonData))
            {
                using (var reader = new BinaryReader(ms, Encoding.UTF8))
                {
                    uint nJoints = reader.ReadUInt32();
                    skeleton.Joints.Capacity = (int)nJoints;
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
            poseSet.SetSkeletonFromFile(skeleton);

            // --------------------
            // Read Pose File
            // --------------------
            string posePath = Path.Combine(path, fileName + ".mmpose");
            if (!File.Exists(posePath))
                return false;

            byte[] poseData = File.ReadAllBytes(posePath);
            using (var ms = new MemoryStream(poseData))
            {
                using (var reader = new BinaryReader(ms, Encoding.UTF8))
                {
                    uint nClips = reader.ReadUInt32();
                    poseSet.SetClipCapacity(nClips);
                    for (int i = 0; i < nClips; i++)
                    {
                        uint start = reader.ReadUInt32();
                        uint end = reader.ReadUInt32();
                        float frameTime = reader.ReadSingle();
                        poseSet.AddAnimationClipDeserialized(new PoseSet.AnimationClip((int)start, (int)end, frameTime));
                    }

                    uint nPoses = reader.ReadUInt32();
                    uint nJoints = reader.ReadUInt32();
                    uint nTags = reader.ReadUInt32();
                    Debug.Assert(nJoints == skeleton.Joints.Count, "Number of joints in skeleton and pose do not match");

                    // Precompute sizes for the buffers (they remain constant across iterations)
                    int float3BufferSize = (int)nJoints * 3 * sizeof(float);
                    int quaternionBufferSize = (int)nJoints * 4 * sizeof(float);

                    // Allocate reusable buffers once outside the loop
                    byte[] float3Buffer = new byte[float3BufferSize];
                    byte[] quaternionBuffer = new byte[quaternionBufferSize];

                    poseSet.SetPoseCapacity(nPoses);
                    for (int i = 0; i < nPoses; i++)
                    {
                        PoseVector pose = new PoseVector();

                        // --- Read JointLocalPositions ---
                        reader.Read(float3Buffer, 0, float3BufferSize);
                        Span<float> positionsSpan = MemoryMarshal.Cast<byte, float>(float3Buffer);
                        pose.JointLocalPositions = new float3[nJoints];
                        for (int j = 0; j < nJoints; j++)
                        {
                            pose.JointLocalPositions[j] = new float3(
                                positionsSpan[j * 3],
                                positionsSpan[j * 3 + 1],
                                positionsSpan[j * 3 + 2]
                            );
                        }

                        // --- Read JointLocalRotations ---
                        reader.Read(quaternionBuffer, 0, quaternionBufferSize);
                        Span<float> rotationsSpan = MemoryMarshal.Cast<byte, float>(quaternionBuffer);
                        pose.JointLocalRotations = new quaternion[nJoints];
                        for (int j = 0; j < nJoints; j++)
                        {
                            pose.JointLocalRotations[j] = new quaternion(
                                rotationsSpan[j * 4],
                                rotationsSpan[j * 4 + 1],
                                rotationsSpan[j * 4 + 2],
                                rotationsSpan[j * 4 + 3]
                            );
                        }

                        // --- Read JointLocalVelocities ---
                        reader.Read(float3Buffer, 0, float3BufferSize);
                        Span<float> velocitiesSpan = MemoryMarshal.Cast<byte, float>(float3Buffer);
                        pose.JointLocalVelocities = new float3[nJoints];
                        for (int j = 0; j < nJoints; j++)
                        {
                            pose.JointLocalVelocities[j] = new float3(
                                velocitiesSpan[j * 3],
                                velocitiesSpan[j * 3 + 1],
                                velocitiesSpan[j * 3 + 2]
                            );
                        }

                        // --- Read JointLocalAngularVelocities ---
                        reader.Read(float3Buffer, 0, float3BufferSize);
                        Span<float> angularVelocitiesSpan = MemoryMarshal.Cast<byte, float>(float3Buffer);
                        pose.JointLocalAngularVelocities = new float3[nJoints];
                        for (int j = 0; j < nJoints; j++)
                        {
                            pose.JointLocalAngularVelocities[j] = new float3(
                                angularVelocitiesSpan[j * 3],
                                angularVelocitiesSpan[j * 3 + 1],
                                angularVelocitiesSpan[j * 3 + 2]
                            );
                        }

                        // --- Read contact flags ---
                        pose.LeftFootContact = reader.ReadUInt32() == 1u;
                        pose.RightFootContact = reader.ReadUInt32() == 1u;

                        poseSet.AddClip(pose);
                    }

                    for (int i = 0; i < nTags; i++)
                    {
                        string name = reader.ReadString();
                        int nRanges = (int)reader.ReadUInt32();
                        List<int> tagStarts = new List<int>(nRanges);
                        List<int> tagEnds = new List<int>(nRanges);
                        for (int r = 0; r < nRanges; r++)
                        {
                            tagStarts.Add((int)reader.ReadUInt32());
                            tagEnds.Add((int)reader.ReadUInt32());
                        }
                        poseSet.AddTagDeserialized(name, tagStarts, tagEnds);
                    }
                    poseSet.ConvertTagsToNativeArrays();
                }
            }
            return true;
        }
    }
}
