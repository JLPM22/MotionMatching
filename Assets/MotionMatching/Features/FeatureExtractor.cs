using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;
    using JointType = Skeleton.JointType;
    using AnimationClip = PoseSet.AnimationClip;

    /// <summary>
    /// Extracts features from a pose for Motion Matching from PoseSet
    /// </summary>
    public class FeatureExtractor
    {
        /// <summary>
        /// Extract the feature vectors from poseSet and return a new FeatureSet
        /// </summary>
        public FeatureSet Extract(PoseSet poseSet)
        {
            int nPoses = poseSet.Poses.Count;
            FeatureVector[] features = new FeatureVector[nPoses];
            Joint leftFoot = poseSet.Skeleton.Find(JointType.LeftFoot);
            Joint rightFoot = poseSet.Skeleton.Find(JointType.RightFoot);
            Joint hips = poseSet.Skeleton.Find(JointType.Hips);
            int i = 0;
            for (int c = 0; c < poseSet.Clips.Count; c++)
            {
                AnimationClip clip = poseSet.Clips[c];
                // HARDCODED: Trajectory we will use 20, 40 and 60 fps (60Hz) as the original paper
                for (int clipIt = clip.Start; clipIt < clip.End - 60; clipIt++)
                {
                    features[i] = ExtractFeature(poseSet, clipIt, clip, leftFoot, rightFoot, hips);
                    i += 1;
                }
            }
            return new FeatureSet(features);
        }

        /// <summary>
        /// Returns the feature vector of the pose at poseIndex, poseIndex cannot be at the end of a clip
        /// </summary>
        private FeatureVector ExtractFeature(PoseSet poseSet, int poseIndex, AnimationClip clip, Joint leftFoot, Joint rightFoot, Joint hips)
        {
            // HARDCODED: Trajectory we will use 20, 40 and 60 fps (60Hz) as the original paper
            Debug.Assert(poseIndex >= clip.Start, "clip does not contain poseIndex");
            Debug.Assert(poseIndex < clip.End - 60, "poseIndex cannot be in the last 60 frames of a clip");
            PoseVector pose = poseSet.Poses[poseIndex];
            PoseVector poseNext = poseSet.Poses[poseIndex + 1];
            // Compute local features based on the projection of the hips in the ZX plane (ground)
            // so hips and feet are local to a stable position with respect to the character
            // this solution only works for one type of skeleton
            GetWorldOriginCharacter(pose.RootWorld, pose.RootWorldRot, out float3 characterOrigin, out float3 characterForward);
            FeatureVector feature = new FeatureVector();
            feature.Valid = true;
            // Left Foot
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, leftFoot, characterOrigin, characterForward, clip, out feature.LeftFootLocalPosition, out feature.LeftFootLocalVelocity);
            // Right Foot
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, rightFoot, characterOrigin, characterForward, clip, out feature.RightFootLocalPosition, out feature.RightFootLocalVelocity);
            // Hips
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, hips, characterOrigin, characterForward, clip, out _, out feature.HipsLocalVelocity);
            // Trajectory
            feature.FutureTrajectoryLocalPosition = new float2[3];
            feature.FutureTrajectoryLocalDirection = new float2[3];
            GetTrajectoryFeatures(poseSet.Poses[poseIndex + 20], characterOrigin, characterForward, out feature.FutureTrajectoryLocalPosition[0], out feature.FutureTrajectoryLocalDirection[0]);
            GetTrajectoryFeatures(poseSet.Poses[poseIndex + 40], characterOrigin, characterForward, out feature.FutureTrajectoryLocalPosition[1], out feature.FutureTrajectoryLocalDirection[1]);
            GetTrajectoryFeatures(poseSet.Poses[poseIndex + 60], characterOrigin, characterForward, out feature.FutureTrajectoryLocalPosition[2], out feature.FutureTrajectoryLocalDirection[2]);
            return feature;
        }

        private void GetTrajectoryFeatures(PoseVector pose, float3 characterOrigin, float3 characterForward, out float2 futureLocalPosition, out float2 futureLocalDirection)
        {
            float3 futureLocalPosition3D = GetLocalPositionFromCharacter(pose.RootWorld, characterOrigin, characterForward);
            futureLocalPosition = new float2(futureLocalPosition3D.x, futureLocalPosition3D.z);
            float3 futureLocalDirection3D = GetLocalDirectionFromCharacter(math.mul(pose.RootWorldRot, new float3(0.0f, 0.0f, 1.0f)), characterForward);
            futureLocalDirection = math.normalize(new float2(futureLocalDirection3D.x, futureLocalDirection3D.z));
        }

        private void GetJointFeatures(PoseVector pose, PoseVector poseNext, Skeleton skeleton, Joint joint, float3 characterOrigin, float3 characterForward, AnimationClip clip,
                                      out float3 localPosition, out float3 localVelocity)
        {
            float3 worldPosition = ForwardKinematics(skeleton, pose, joint);
            float3 worldPositionNext = ForwardKinematics(skeleton, poseNext, joint);
            localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            localVelocity = (GetLocalPositionFromCharacter(worldPositionNext, characterOrigin, characterForward) - localPosition) / clip.FrameTime;
        }

        /// <summary>
        /// Returns the position of the joint in world space after applying FK using the pose
        /// </summary>
        private float3 ForwardKinematics(Skeleton skeleton, PoseVector pose, Joint joint)
        {
            float3 localOffset = joint.LocalOffset;
            Matrix4x4 localToWorld = Matrix4x4.identity;
            while (joint.ParentIndex != 0) // while not root
            {
                localToWorld = Matrix4x4.TRS(pose.JointLocalPositions[joint.ParentIndex], pose.JointLocalRotations[joint.ParentIndex], new float3(1.0f, 1.0f, 1.0f)) * localToWorld;
                joint = skeleton.GetParent(joint);
            }
            localToWorld = Matrix4x4.TRS(pose.RootWorld, pose.RootWorldRot, new float3(1.0f, 1.0f, 1.0f)) * localToWorld; // root
            return localToWorld.MultiplyPoint3x4(localOffset);
        }

        public static void GetWorldOriginCharacter(float3 worldHips, quaternion worldRotHips, out float3 center, out float3 forward)
        {
            center = worldHips;
            center.y = 0.0f; // Projected hips
            forward = math.mul(worldRotHips, new float3(0.0f, 0.0f, 1.0f));
            forward.y = 0.0f; // Projected forward (hips)
            forward = math.normalize(forward);
        }

        private float3 GetLocalPositionFromCharacter(float3 worldPos, float3 centerCharacter, float3 forwardCharacter)
        {
            float3 localPos = worldPos;
            localPos -= centerCharacter;
            localPos = math.mul(math.inverse(quaternion.LookRotation(forwardCharacter, new float3(0.0f, 1.0f, 0.0f))), localPos);
            return localPos;
        }

        public static float3 GetLocalDirectionFromCharacter(float3 worldDir, float3 forwardCharacter)
        {
            float3 localDir = worldDir;
            localDir = math.mul(math.inverse(quaternion.LookRotation(forwardCharacter, new float3(0.0f, 1.0f, 0.0f))), localDir);
            return localDir;
        }
    }
}