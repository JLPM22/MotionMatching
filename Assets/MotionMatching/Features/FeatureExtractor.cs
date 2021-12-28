using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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
            // Compute local features based on the projection of the hips in the ZX plane? (ground)
            // so hips and feet are local to a stable position with respect to the character
            // this solution only works for a type of skeleton
            GetWorldOriginCharacter(pose, out Vector3 characterOrigin, out Vector3 characterForward);
            FeatureVector feature = new FeatureVector();
            feature.Valid = true;
            // Left Foot
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, leftFoot, characterOrigin, characterForward, clip, out feature.LeftFootLocalPosition, out feature.LeftFootLocalVelocity);
            // Right Foot
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, rightFoot, characterOrigin, characterForward, clip, out feature.RightFootLocalPosition, out feature.RightFootLocalVelocity);
            // Hips
            GetJointFeatures(pose, poseNext, poseSet.Skeleton, hips, characterOrigin, characterForward, clip, out _, out feature.HipsLocalVelocity);
            // Trajectory
            feature.FutureTrajectoryLocalPosition = new Vector2[3];
            feature.FutureTrajectoryLocalDirections = new Vector2[3];
            GetTrajectoryFeatures(poseSet.Poses[poseIndex + 20], characterOrigin, characterForward, out feature.FutureTrajectoryLocalPosition[0], out feature.FutureTrajectoryLocalDirections[0]);
            GetTrajectoryFeatures(poseSet.Poses[poseIndex + 40], characterOrigin, characterForward, out feature.FutureTrajectoryLocalPosition[1], out feature.FutureTrajectoryLocalDirections[1]);
            GetTrajectoryFeatures(poseSet.Poses[poseIndex + 60], characterOrigin, characterForward, out feature.FutureTrajectoryLocalPosition[2], out feature.FutureTrajectoryLocalDirections[2]);
            return feature;
        }

        private void GetTrajectoryFeatures(PoseVector pose, Vector3 characterOrigin, Vector3 characterForward, out Vector2 futureLocalPosition, out Vector2 futureLocalDirection)
        {
            Vector3 futureLocalPosition3D = GetLocalPositionFromCharacter(pose.RootMotion, characterOrigin, characterForward);
            futureLocalPosition = new Vector2(futureLocalPosition3D.x, futureLocalPosition3D.z);
            Vector3 futureLocalDirection3D = GetLocalDirectionFromCharacter(pose.JointLocalRotations[0] * Vector3.forward, characterForward);
            futureLocalDirection = (new Vector2(futureLocalDirection3D.x, futureLocalDirection3D.z)).normalized;
        }

        private void GetJointFeatures(PoseVector pose, PoseVector poseNext, Skeleton skeleton, Joint joint, Vector3 characterOrigin, Vector3 characterForward, AnimationClip clip,
                                      out Vector3 localPosition, out Vector3 localVelocity)
        {
            Vector3 worldPosition = ForwardKinematics(skeleton, pose, joint);
            Vector3 worldPositionNext = ForwardKinematics(skeleton, poseNext, joint);
            localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            localVelocity = (GetLocalPositionFromCharacter(worldPositionNext, characterOrigin, characterForward) - localPosition) / clip.FrameTime;
        }

        /// <summary>
        /// Returns the position of the joint in world space after applying FK using the pose
        /// </summary>
        private Vector3 ForwardKinematics(Skeleton skeleton, PoseVector pose, Joint joint)
        {
            Vector3 localOffset = joint.LocalOffset;
            Matrix4x4 localToWorld = Matrix4x4.identity;
            while (joint.ParentIndex != 0) // while not root
            {
                localToWorld = Matrix4x4.TRS(pose.JointLocalPositions[joint.ParentIndex], pose.JointLocalRotations[joint.ParentIndex], Vector3.one) * localToWorld;
                joint = skeleton.GetParent(joint);
            }
            localToWorld = Matrix4x4.TRS(pose.RootMotion, pose.JointLocalRotations[0], Vector3.one) * localToWorld; // root
            return localToWorld.MultiplyPoint3x4(localOffset);
        }

        public static void GetWorldOriginCharacter(PoseVector pose, out Vector3 center, out Vector3 forward)
        {
            Plane ground = new Plane(Vector3.up, Vector3.zero);
            center = ground.ClosestPointOnPlane(pose.RootMotion); // Projected hips
            forward = Vector3.ProjectOnPlane(pose.JointLocalRotations[0] * Vector3.forward, Vector3.up); // Projected forward (hips)
            forward = forward.normalized;
        }

        private Vector3 GetLocalPositionFromCharacter(Vector3 worldPos, Vector3 centerCharacter, Vector3 forwardCharacter)
        {
            Vector3 localPos = worldPos;
            localPos -= centerCharacter;
            localPos = Quaternion.Inverse(Quaternion.LookRotation(forwardCharacter, Vector3.up)) * localPos;
            return localPos;
        }

        private Vector3 GetLocalDirectionFromCharacter(Vector3 worldDir, Vector3 forwardCharacter)
        {
            Vector3 localDir = worldDir;
            localDir = Quaternion.Inverse(Quaternion.LookRotation(forwardCharacter, Vector3.up)) * localDir;
            return localDir;
        }
    }
}