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
        [Tooltip("Local vector (axis) pointing in the forward direction of the character")] public Vector3 ForwardLocalVector = new Vector3(0, 0, 1);
        [Tooltip("Local vector (axis) pointing in the up direction of the character")] public Vector3 UpLocalVector = new Vector3(0, 1, 0);

        private Animator Animator;

        // Retargeting
        // Initial orientations of the bones The code assumes the initial orientations are in T-Pose
        private Quaternion[] SourceTPose;
        private Quaternion[] TargetTPose;
        // Mapping from BodyJoints to the actual transforms
        private Transform[] SourceBones;
        private Transform[] TargetBones;
        // Mapping Hips Orientation
        private Quaternion HipsCorrection;
        
        public bool ShouldRetarget { get { return MotionMatching.MMData.BVHTPose != null; } }

        private void Awake()
        {
            Animator = GetComponent<Animator>();
        }

        private void OnEnable()
        {
            MotionMatching.OnSkeletonTransformUpdated += OnSkeletonTransformUpdated;
        }

        private void OnDisable()
        {
            MotionMatching.OnSkeletonTransformUpdated -= OnSkeletonTransformUpdated;
        }

        private void Start()
        {
            // BindSkinnedMeshRenderers();
            if (ShouldRetarget) InitRetargeting();
        }

        private void InitRetargeting()
        {
            MotionMatchingData mmData = MotionMatching.MMData;
            SourceTPose = new Quaternion[BodyJoints.Length];
            TargetTPose = new Quaternion[BodyJoints.Length];
            SourceBones = new Transform[BodyJoints.Length];
            TargetBones = new Transform[BodyJoints.Length];
            // Source TPose (BVH with TPose)
            BVHImporter bvhImporter = new BVHImporter();
            // Animation containing in the first frame a TPose
            BVHAnimation tposeAnimation = bvhImporter.Import(mmData.BVHTPose, mmData.UnitScale, true);
            // Store Rotations
            // Source
            Skeleton skeleton = tposeAnimation.Skeleton;
            for (int i = 0; i < BodyJoints.Length; i++)
            {
                if (mmData.GetJointName(BodyJoints[i], out string jointName) &&
                    skeleton.Find(jointName, out Skeleton.Joint joint))
                {
                    // Get the rotation for the first frame of the animation
                    SourceTPose[i] = tposeAnimation.GetWorldRotation(joint, 0);
                }
            }
            // Target
            Quaternion rot = Animator.transform.rotation;
            Animator.transform.rotation = Quaternion.identity;
            for (int i = 0; i < BodyJoints.Length; i++)
            {
                TargetTPose[i] = Animator.GetBoneTransform(BodyJoints[i]).rotation;
            }
            Animator.transform.rotation = rot;
            // Correct body orientation so they are both facing the same direction
            float3 targetWorldForward = math.mul(TargetTPose[0], ForwardLocalVector);
            float3 targetWorldUp = math.mul(TargetTPose[0], UpLocalVector);
            float3 sourceWorldForward = math.mul(SourceTPose[0], mmData.HipsForwardLocalVector);
            float3 sourceWorldUp = math.mul(SourceTPose[0], mmData.HipsUpLocalVector);
            quaternion targetLookAt = quaternion.LookRotation(targetWorldForward, targetWorldUp);
            quaternion sourceLookAt = quaternion.LookRotation(sourceWorldForward, sourceWorldUp);
            HipsCorrection = math.mul(sourceLookAt, math.inverse(targetLookAt));
            // Store Transforms
            Transform[] mmBones = MotionMatching.GetSkeletonTransforms();
            Dictionary<string, Transform> boneDict = new Dictionary<string, Transform>();
            foreach (Transform bone in mmBones)
            {
                boneDict.Add(bone.name, bone);
            }
            // Source
            for (int i = 0; i < BodyJoints.Length; i++)
            {
                if (mmData.GetJointName(BodyJoints[i], out string jointName) &&
                    boneDict.TryGetValue(jointName, out Transform bone))
                {
                    SourceBones[i] = bone;
                }
            }
            // Target
            for (int i = 0; i < BodyJoints.Length; i++)
            {
                TargetBones[i] = Animator.GetBoneTransform(BodyJoints[i]);
            }
        }

        private void OnSkeletonTransformUpdated()
        {
            if (!ShouldRetarget) return;
            // Motion
            transform.position = MotionMatching.transform.position;
            // Retargeting
            for (int i = 0; i < BodyJoints.Length; i++)
            {
                Quaternion sourceTPoseRotation = SourceTPose[i];
                Quaternion targetTPoseRotation = TargetTPose[i];
                Quaternion sourceRotation = SourceBones[i].rotation;
                /*
                    R_t = Rotation transforming from target local space to world space
                    R_s = Rotation transforming from source local space to world space
                    R_t = R_s * R_st (R_st is a matrix transforming from target local to source local space)
                    // It makes sense because R_st will be mapping from target to source, and R_s from source to world.
                    // The result is transforming from T to world, which is what R_t does.
                    RTPose_t = RTPose_s * R_st
                    R_st = (RTPose_s)^-1 * RTPose_t
                    R_t = R_s * (R_st)^-1 * RTPose_t
                */
                // targetTPoseRotation -> Local Target -> World (Target TPose)
                // HipsCorrection -> World (Target TPose) -> World (Source TPose)
                // sourceTPoseRotation^-1 -> World (SourceTPose) -> Local Source
                // sourceRotation -> Local Source -> World (Source)
                TargetBones[i].rotation = sourceRotation * Quaternion.Inverse(sourceTPoseRotation) * HipsCorrection * targetTPoseRotation;
            }
            // Hips Height
            TargetBones[0].position = MotionMatching.GetSkeletonTransforms()[1].position;
        }

        // Used for retargeting. First parent, then children
        private HumanBodyBones[] BodyJoints =
        {
            HumanBodyBones.Hips,

            HumanBodyBones.Spine,
            HumanBodyBones.Chest,
            HumanBodyBones.UpperChest,

            HumanBodyBones.Neck,
            HumanBodyBones.Head,

            HumanBodyBones.LeftShoulder,
            HumanBodyBones.LeftUpperArm,
            HumanBodyBones.LeftLowerArm,
            HumanBodyBones.LeftHand,

            HumanBodyBones.RightShoulder,
            HumanBodyBones.RightUpperArm,
            HumanBodyBones.RightLowerArm,
            HumanBodyBones.RightHand,

            HumanBodyBones.LeftUpperLeg,
            HumanBodyBones.LeftLowerLeg,
            HumanBodyBones.LeftFoot,
            HumanBodyBones.LeftToes,

            HumanBodyBones.RightUpperLeg,
            HumanBodyBones.RightLowerLeg,
            HumanBodyBones.RightFoot,
            HumanBodyBones.RightToes
        };

        private void OnValidate()
        {
            if (math.abs(math.length(ForwardLocalVector)) < 1E-3f)
            {
                Debug.LogWarning("ForwardLocalVector is too close to zero. Object: " + name);
            }
        }
    }
}