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
        [Tooltip("Local vector (axis) pointing in the forward direction of the character")] 
        public Vector3 ForwardLocalVector = new Vector3(0, 0, 1);
        [Tooltip("Local vector (axis) pointing in the up direction of the character")] 
        public Vector3 UpLocalVector = new Vector3(0, 1, 0);
        [Tooltip("Enable to avoid the toes joint (+ ToesSoleOffset) to penetrate the floor (assuming floor at y=0). The root joint will be adjusted to compensate the height difference.")] 
        public bool AvoidToesFloorPenetration;
        [Tooltip("Offset added to the toes joint to determine the sole position to avoid toes-floor penetration.")] 
        public Vector3 ToesSoleOffset;

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
        // Toes-Floor Penetration
        private float ToesPenetrationMovingCorrection;
        
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
            InitRetargeting();
        }

        private void InitRetargeting()
        {
            MotionMatchingData mmData = MotionMatching.MMData;
            SourceTPose = new Quaternion[BodyJoints.Length];
            TargetTPose = new Quaternion[BodyJoints.Length];
            SourceBones = new Transform[BodyJoints.Length];
            TargetBones = new Transform[BodyJoints.Length];
            // Animation containing in the first frame a T-Pose
            BVHAnimation tposeAnimation = mmData.AnimationDataTPose.GetAnimation();
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
            // Hips
            TargetBones[0].position = MotionMatching.GetSkeletonTransforms()[1].position;

            // Toes-Floor Penetration
            if (AvoidToesFloorPenetration)
            {
                const int leftToesIndex = 17;
                const int rightToesIndex = 21;
                float height = Mathf.Min(TargetBones[leftToesIndex].TransformPoint(ToesSoleOffset).y,
                                         TargetBones[rightToesIndex].TransformPoint(ToesSoleOffset).y);
                height = height < 0.0f ? -height : 0.0f;

                const float movingAverangeFactor = 0.99f;
                ToesPenetrationMovingCorrection = ToesPenetrationMovingCorrection * movingAverangeFactor + height * (1.0f - movingAverangeFactor);

                Vector3 hipsPos = TargetBones[0].position;
                hipsPos.y += ToesPenetrationMovingCorrection;
                TargetBones[0].position = hipsPos;
            }
        }

        // Used for retargeting. First parent, then children
        private HumanBodyBones[] BodyJoints =
        {
            HumanBodyBones.Hips, // 0

            HumanBodyBones.Spine, // 1
            HumanBodyBones.Chest, // 2
            HumanBodyBones.UpperChest, // 3

            HumanBodyBones.Neck, // 4
            HumanBodyBones.Head, // 5

            HumanBodyBones.LeftShoulder, // 6
            HumanBodyBones.LeftUpperArm, // 7
            HumanBodyBones.LeftLowerArm, // 8
            HumanBodyBones.LeftHand, // 9

            HumanBodyBones.RightShoulder, // 10
            HumanBodyBones.RightUpperArm, // 11
            HumanBodyBones.RightLowerArm, // 12
            HumanBodyBones.RightHand, // 13

            HumanBodyBones.LeftUpperLeg, // 14
            HumanBodyBones.LeftLowerLeg, // 15
            HumanBodyBones.LeftFoot, // 16
            HumanBodyBones.LeftToes, // 17

            HumanBodyBones.RightUpperLeg, // 18
            HumanBodyBones.RightLowerLeg, // 19
            HumanBodyBones.RightFoot, // 20
            HumanBodyBones.RightToes // 21
        };

        private void OnValidate()
        {
            if (math.abs(math.length(ForwardLocalVector)) < 1E-3f)
            {
                Debug.LogWarning("ForwardLocalVector is too close to zero. Object: " + name);
            }
        }

#if UNITY_EDITOR
        private void OnDrawGizmosSelected()
        {
            Animator animator = GetComponent<Animator>();

            if (animator == null) return;

            Vector3 leftSole = animator.GetBoneTransform(HumanBodyBones.LeftToes).TransformPoint(ToesSoleOffset);
            Vector3 rightSole = animator.GetBoneTransform(HumanBodyBones.RightToes).TransformPoint(ToesSoleOffset);
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(leftSole, 0.005f);
            Gizmos.DrawSphere(rightSole, 0.005f);
        }
#endif
    }
}