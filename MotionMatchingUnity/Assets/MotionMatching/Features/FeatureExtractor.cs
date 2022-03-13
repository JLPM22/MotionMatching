using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;
    using AnimationClip = PoseSet.AnimationClip;

    /// <summary>
    /// Extracts features from a pose for Motion Matching from PoseSet
    /// </summary>
    public class FeatureExtractor
    {
        /// <summary>
        /// Extract the feature vectors from poseSet and return a new FeatureSet
        /// </summary>
        public FeatureSet Extract(PoseSet poseSet, MotionMatchingData mmData)
        {
            int nPoses = poseSet.NumberPoses;
            FeatureVector[] features = new FeatureVector[nPoses];
            if (!poseSet.Skeleton.Find(HumanBodyBones.LeftFoot, out Joint leftFoot)) Debug.Assert(false, "The skeleton does not contain any joint of type HumanBodyBones.LeftFoot");
            if (!poseSet.Skeleton.Find(HumanBodyBones.RightFoot, out Joint rightFoot)) Debug.Assert(false, "The skeleton does not contain any joint of type HumanBodyBones.RightFoot");
            if (!poseSet.Skeleton.Find(HumanBodyBones.Hips, out Joint hips)) Debug.Assert(false, "The skeleton does not contain any joint of type HumanBodyBones.Hips");
            for (int poseIndex = 0; poseIndex < nPoses; ++poseIndex)
            {
                if (poseSet.IsPoseValidForPrediction(poseIndex))
                {
                    features[poseIndex] = ExtractFeature(poseSet, poseIndex, leftFoot, rightFoot, hips, mmData);
                }
            }
            return new FeatureSet(features);
        }

        /// <summary>
        /// Returns the feature vector of the pose at poseIndex, poseIndex cannot be at the end of a clip
        /// </summary>
        private FeatureVector ExtractFeature(PoseSet poseSet, int poseIndex, Joint leftFoot, Joint rightFoot, Joint hips, MotionMatchingData mmData)
        {
            // HARDCODED: only 3 points in trajectory
            poseSet.GetPose(poseIndex, out PoseVector pose);
            poseSet.GetPose(poseIndex + 1, out PoseVector poseNext);
            // Compute local features based on the projection of the hips in the ZX plane (ground)
            // so hips and feet are local to a stable position with respect to the character
            // this solution only works for one type of skeleton
            GetWorldOriginCharacter(pose.RootWorld, pose.RootWorldRot, mmData.HipsForwardLocalVector, out float3 characterOrigin, out float3 characterForward);
            FeatureVector feature = new FeatureVector();
            feature.IsValid = true;
            // Left Foot
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, leftFoot, characterOrigin, characterForward, poseSet.FrameTime, out feature.LeftFootLocalPosition, out feature.LeftFootLocalVelocity);
            // Right Foot
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, rightFoot, characterOrigin, characterForward, poseSet.FrameTime, out feature.RightFootLocalPosition, out feature.RightFootLocalVelocity);
            // Hips
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, hips, characterOrigin, characterForward, poseSet.FrameTime, out _, out feature.HipsLocalVelocity);
            // Trajectory
            float2 trajectoryLocalPos, trajectoryLocalDir;
            poseSet.GetPose(poseIndex + mmData.PredictionFrames, out PoseVector futurePose1);
            GetTrajectoryFeatures(futurePose1, characterOrigin, characterForward, mmData.HipsForwardLocalVector, out trajectoryLocalPos, out trajectoryLocalDir);
            feature.SetFutureTrajectoryLocalPosition(0, trajectoryLocalPos);
            feature.SetFutureTrajectoryLocalDirection(0, trajectoryLocalDir);
            poseSet.GetPose(poseIndex + mmData.PredictionFrames * 2, out PoseVector futurePose2);
            GetTrajectoryFeatures(futurePose2, characterOrigin, characterForward, mmData.HipsForwardLocalVector, out trajectoryLocalPos, out trajectoryLocalDir);
            feature.SetFutureTrajectoryLocalPosition(1, trajectoryLocalPos);
            feature.SetFutureTrajectoryLocalDirection(1, trajectoryLocalDir);
            poseSet.GetPose(poseIndex + mmData.PredictionFrames * 3, out PoseVector futurePose3);
            GetTrajectoryFeatures(futurePose3, characterOrigin, characterForward, mmData.HipsForwardLocalVector, out trajectoryLocalPos, out trajectoryLocalDir);
            feature.SetFutureTrajectoryLocalPosition(2, trajectoryLocalPos);
            feature.SetFutureTrajectoryLocalDirection(2, trajectoryLocalDir);
            return feature;
        }

        private void GetTrajectoryFeatures(PoseVector pose, float3 characterOrigin, float3 characterForward, float3 hipsForwardLocalVector, out float2 futureLocalPosition, out float2 futureLocalDirection)
        {
            float3 futureLocalPosition3D = GetLocalPositionFromCharacter(pose.RootWorld, characterOrigin, characterForward);
            futureLocalPosition = new float2(futureLocalPosition3D.x, futureLocalPosition3D.z);
            float3 futureLocalDirection3D = GetLocalDirectionFromCharacter(math.mul(pose.RootWorldRot, hipsForwardLocalVector), characterForward);
            futureLocalDirection = math.normalize(new float2(futureLocalDirection3D.x, futureLocalDirection3D.z));
        }

        private void GetJointFeatures(PoseVector pose, PoseVector poseNext, Skeleton skeleton, Joint joint, float3 characterOrigin, float3 characterForward, float frameTime,
                                      out float3 localPosition, out float3 localVelocity)
        {
            float3 worldPosition = ForwardKinematics(skeleton, pose, joint);
            float3 worldPositionNext = ForwardKinematics(skeleton, poseNext, joint);
            localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            localVelocity = (GetLocalPositionFromCharacter(worldPositionNext, characterOrigin, characterForward) - localPosition) / frameTime;
        }

        /// <summary>
        /// Returns the position of the joint in world space after applying FK using the pose
        /// </summary>
        private float3 ForwardKinematics(Skeleton skeleton, PoseVector pose, Joint joint)
        {
            float3 localOffset = pose.JointLocalPositions[joint.Index];
            Matrix4x4 localToWorld = Matrix4x4.identity;
            while (joint.ParentIndex != 0) // while not root
            {
                localToWorld = Matrix4x4.TRS(pose.JointLocalPositions[joint.ParentIndex], pose.JointLocalRotations[joint.ParentIndex], new float3(1.0f, 1.0f, 1.0f)) * localToWorld;
                joint = skeleton.GetParent(joint);
            }
            localToWorld = Matrix4x4.TRS(pose.RootWorld, pose.RootWorldRot, new float3(1.0f, 1.0f, 1.0f)) * localToWorld; // root
            return localToWorld.MultiplyPoint3x4(localOffset);
        }

        /// <summary>
        /// Returns the position and forward vector of the character in world space considering the hips position and rotation
        /// </summary>
        /// <param name="hipsForwardLocalVector">forward vector of the hips in world space when in bind pose</param>
        public static void GetWorldOriginCharacter(float3 worldHips, quaternion worldRotHips, float3 hipsForwardLocalVector, out float3 center, out float3 forward)
        {
            center = worldHips;
            center.y = 0.0f; // Projected hips
            forward = math.mul(worldRotHips, hipsForwardLocalVector);
            forward.y = 0.0f; // Projected forward (hips)
            forward = math.normalize(forward);
        }

        public static float3 GetLocalPositionFromCharacter(float3 worldPos, float3 centerCharacter, float3 forwardCharacter)
        {
            float3 localPos = worldPos;
            localPos -= centerCharacter;
            localPos = math.mul(math.inverse(quaternion.LookRotation(forwardCharacter, new float3(0.0f, 1.0f, 0.0f))), localPos);
            return localPos;
        }

        public static float3 GetLocalDirectionFromCharacter(float3 worldDir, float3 forwardCharacter)
        {
            float3 localDir = math.mul(math.inverse(quaternion.LookRotation(forwardCharacter, new float3(0.0f, 1.0f, 0.0f))), worldDir);
            return localDir;
        }
    }
}