using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System;

namespace MotionMatching
{
    [RequireComponent(typeof(Animator))]
    public class MotionMatchingSkinnedMeshRenderer : MonoBehaviour
    {
        public MotionMatchingController MotionMatching;

        private Animator Animator;
        // private quaternion[] SkinnedMeshPoseToMotionMatching; // From bind pose of the skinned mesh to the bind pose of motion matching (bind rotations are 0)
        private Dictionary<string, HumanBodyBones> JointNameToMecanim = new Dictionary<string, HumanBodyBones>();

        private void Awake()
        {
            Animator = GetComponent<Animator>();
            // Init JointNameToMecanim
            foreach (HumanBone bone in Animator.avatar.humanDescription.human)
            {
                JointNameToMecanim.Add(bone.boneName, (HumanBodyBones)Enum.Parse(typeof(HumanBodyBones), bone.humanName.Replace(" ", "")));
            }
        }

        // private void OnEnable()
        // {
        //     MotionMatching.OnSkeletonTransformUpdated += OnSkeletonTransformUpdated;
        // }

        // private void OnDisable()
        // {
        //     MotionMatching.OnSkeletonTransformUpdated -= OnSkeletonTransformUpdated;
        // }

        private void Start()
        {
            // Bones used by MotionMatching (from BVH)
            Transform[] mmBones = MotionMatching.GetSkeletonTransforms();
            Skeleton mmSkeleton = MotionMatching.GetSkeleton();
            // Dictionary used for fake bones to find parent (even if they aren't in the skeleton)
            Dictionary<string, Transform> boneDict = new Dictionary<string, Transform>();
            foreach (Transform bone in mmBones)
            {
                boneDict.Add(bone.name, bone);
            }

            // Get all SkinnedMeshRenderers
            SkinnedMeshRenderer[] skinnedMeshRenderers = GetComponentsInChildren<SkinnedMeshRenderer>();
            foreach (SkinnedMeshRenderer skinnedMeshRenderer in skinnedMeshRenderers)
            {
                // Fill this array with the bones used by MotionMatching
                // or create "fake" bones if the bone is not used by MotionMatching but present in the skinned mesh
                Transform[] bonesRenderer = new Transform[skinnedMeshRenderer.bones.Length];
                for (int i = 0; i < skinnedMeshRenderer.bones.Length; i++)
                {
                    if (JointNameToMecanim.TryGetValue(skinnedMeshRenderer.bones[i].name, out HumanBodyBones boneType) &&
                        mmSkeleton.Find(boneType, out Skeleton.Joint mmJoint))
                    {
                        // Bone used by MotionMatching
                        bonesRenderer[i] = mmBones[mmJoint.Index];
                    }
                    else if (boneDict.ContainsKey(skinnedMeshRenderer.bones[i].name))
                    {
                        // Bone not used by MotionMatching but already created
                        bonesRenderer[i] = boneDict[skinnedMeshRenderer.bones[i].name];
                    }
                    else
                    {
                        // Create "fake" bone
                        Transform parent = skinnedMeshRenderer.bones[i].parent;
                        Transform newBone = new GameObject(skinnedMeshRenderer.bones[i].name).transform;
                        if (JointNameToMecanim.TryGetValue(parent.name, out HumanBodyBones parentBoneType) &&
                            mmSkeleton.Find(parentBoneType, out Skeleton.Joint mmJointParent))
                        {
                            newBone.SetParent(mmBones[mmJointParent.Index], false);
                        }
                        else
                        {
                            newBone.SetParent(boneDict[parent.name], false);
                        }
                        newBone.localPosition = skinnedMeshRenderer.bones[i].localPosition;
                        newBone.localRotation = skinnedMeshRenderer.bones[i].localRotation;
                        newBone.localScale = skinnedMeshRenderer.bones[i].localScale;
                        boneDict.Add(newBone.name, newBone);
                        bonesRenderer[i] = newBone;
                    }
                }
                // Structures needed by SkinnedMeshRenderer to do the skinning
                skinnedMeshRenderer.bones = bonesRenderer;
                skinnedMeshRenderer.rootBone = bonesRenderer[0];
            }
        }

        // private void OnSkeletonTransformUpdated(Skeleton mmSkeleton, Transform[] mmSkeletonTransforms)
        // {
        //     if (SkinnedMeshPoseToMotionMatching == null)
        //     {
        //         InitBindPose(mmSkeleton, mmSkeletonTransforms);
        //     }
        //     // Retarget
        //     for (int i = 0; i < mmSkeletonTransforms.Length; i++)
        //     {
        //         mmSkeletonTransforms[i].localRotation = SkinnedMeshPoseToMotionMatching[i] * mmSkeletonTransforms[i].localRotation;
        //     }
        // }

        // private void InitBindPose(Skeleton mmSkeleton, Transform[] mmSkeletonTransforms)
        // {
        //     SkinnedMeshPoseToMotionMatching = new quaternion[mmSkeleton.Joints.Count];
        //     // (1). Set all LocalRotation to 0 to mimic the bind pose of motion matching
        //     for (int i = 0; i < mmSkeleton.Joints.Count; i++)
        //     {
        //         Skeleton.Joint mmJoint = mmSkeleton.Joints[i];
        //         Transform jointTransform = Animator.GetBoneTransform(mmJoint.Type);
        //         if (jointTransform != null)
        //         {
        //             if (mmJoint.Type == HumanBodyBones.Hips)
        //             {
        //                 Transform parent = jointTransform.parent;
        //                 while (parent != null)
        //                 {
        //                     parent.localRotation = quaternion.identity; // if exported from blender... sometimes they have Armature bone
        //                     parent = parent.parent;
        //                 }
        //             }
        //             jointTransform.localRotation = quaternion.identity;
        //         }
        //     }
        //     quaternion[] mmBeforeRot = new quaternion[mmSkeletonTransforms.Length];
        //     for (int i = 0; i < mmSkeletonTransforms.Length; i++)
        //     {
        //         mmBeforeRot[i] = mmSkeletonTransforms[i].localRotation;
        //         mmSkeletonTransforms[i].localRotation = quaternion.identity;
        //     }
        //     // (2). For each joint, take one child, and take the vector between the joint and the child.
        //     //      Search the angle between this vector in the skinned mesh and the same vector in the motion matching pose.
        //     //      This rotation converts the skinned mesh pose to the motion matching pose.
        //     //      Then, multiplying this rotation by the local rotation determined by motion matching will give the final pose.
        //     for (int i = 0; i < mmSkeleton.Joints.Count; i++) // the  way mmSkeleton is stored... parents will be always processed first than their children
        //     {
        //         Quaternion rot = Quaternion.identity;
        //         Skeleton.Joint mmJoint = mmSkeleton.Joints[i];
        //         if (mmSkeleton.Find(ParentToChild[(int)mmJoint.Type], out Skeleton.Joint mmJointChild) && mmJointChild.Type != HumanBodyBones.LastBone)
        //         {
        //             Transform jParent = Animator.GetBoneTransform(mmJoint.Type);
        //             Transform jChild = Animator.GetBoneTransform(mmJointChild.Type);
        //             if (jChild != null)
        //             {
        //                 float3 mmVector = mmSkeletonTransforms[mmJoint.Index].TransformDirection(mmJointChild.LocalOffset.normalized);
        //                 float3 vector = jParent.TransformDirection(jChild.localPosition.normalized);
        //                 rot = MathExtensions.FromToRotation(vector, mmVector);
        //             }
        //         }
        //         Animator.GetBoneTransform(mmJoint.Type).localRotation = rot;
        //         SkinnedMeshPoseToMotionMatching[i] = rot;
        //     }
        //     // Restore rotations
        //     for (int i = 0; i < mmSkeletonTransforms.Length; i++)
        //     {
        //         mmSkeletonTransforms[i].localRotation = mmBeforeRot[i];
        //     }
        // }

        // private HumanBodyBones[] ParentToChild = {
        //     HumanBodyBones.Spine, // Hips
        //     HumanBodyBones.LeftLowerLeg, // LeftUpperLeg
        //     HumanBodyBones.RightLowerLeg, // RightUpperLeg
        //     HumanBodyBones.LeftFoot, // LeftLowerLeg
        //     HumanBodyBones.RightFoot, // RightLowerLeg
        //     HumanBodyBones.LeftToes, // LeftFoot
        //     HumanBodyBones.RightToes, // RightFoot
        //     HumanBodyBones.Chest, // Spine
        //     HumanBodyBones.UpperChest, // Chest
        //     HumanBodyBones.Head, // Neck
        //     HumanBodyBones.LeftEye, // Head
        //     HumanBodyBones.LeftUpperArm, // LeftShoulder
        //     HumanBodyBones.RightUpperArm, // RightShoulder
        //     HumanBodyBones.LeftLowerArm, // LeftUpperArm
        //     HumanBodyBones.RightLowerArm, // RightUpperArm
        //     HumanBodyBones.LeftHand, // LeftLowerArm
        //     HumanBodyBones.RightHand, // RightLowerArm
        //     HumanBodyBones.LeftIndexProximal, // LeftHand
        //     HumanBodyBones.RightIndexProximal, // RightHand
        //     HumanBodyBones.LastBone, // LeftToes
        //     HumanBodyBones.LastBone, // RightToes
        //     HumanBodyBones.LastBone, // LeftEye
        //     HumanBodyBones.LastBone, // RightEye
        //     HumanBodyBones.LastBone, // Jaw
        //     HumanBodyBones.LeftThumbIntermediate, // LeftThumbProximal
        //     HumanBodyBones.LeftThumbDistal, // LeftThumbIntermediate
        //     HumanBodyBones.LastBone, // LeftThumbDistal
        //     HumanBodyBones.LeftIndexIntermediate, // LeftIndexProximal
        //     HumanBodyBones.LeftIndexDistal, // LeftIndexIntermediate
        //     HumanBodyBones.LastBone, // LeftIndexDistal
        //     HumanBodyBones.LeftMiddleIntermediate, // LeftMiddleProximal
        //     HumanBodyBones.LeftMiddleDistal, // LeftMiddleIntermediate
        //     HumanBodyBones.LastBone, // LeftMiddleDistal
        //     HumanBodyBones.LeftRingIntermediate, // LeftRingProximal
        //     HumanBodyBones.LeftRingDistal, // LeftRingIntermediate
        //     HumanBodyBones.LastBone, // LeftRingDistal
        //     HumanBodyBones.LeftLittleIntermediate, // LeftLittleProximal
        //     HumanBodyBones.LeftLittleDistal, // LeftLittleIntermediate
        //     HumanBodyBones.LastBone, // LeftLittleDistal
        //     HumanBodyBones.RightThumbIntermediate, // RightThumbProximal
        //     HumanBodyBones.RightThumbDistal, // RightThumbIntermediate
        //     HumanBodyBones.LastBone, // RightThumbDistal
        //     HumanBodyBones.RightIndexIntermediate, // RightIndexProximal
        //     HumanBodyBones.RightIndexDistal, // RightIndexIntermediate
        //     HumanBodyBones.LastBone, // RightIndexDistal
        //     HumanBodyBones.RightMiddleIntermediate, // RightMiddleProximal
        //     HumanBodyBones.RightMiddleDistal, // RightMiddleIntermediate
        //     HumanBodyBones.LastBone, // RightMiddleDistal
        //     HumanBodyBones.RightRingIntermediate, // RightRingProximal
        //     HumanBodyBones.RightRingDistal, // RightRingIntermediate
        //     HumanBodyBones.LastBone, // RightRingDistal
        //     HumanBodyBones.RightLittleIntermediate, // RightLittleProximal
        //     HumanBodyBones.RightLittleDistal, // RightLittleIntermediate
        //     HumanBodyBones.LastBone, // RightLittleDistal
        //     HumanBodyBones.Neck, // UpperChest
        //     HumanBodyBones.LastBone, // LastBone
        //  };
    }
}