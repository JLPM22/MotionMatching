using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    public static class HumanBodyBonesExtensions
    {
        public static HumanBodyBones GetMirroredBone(HumanBodyBones bone)
        {
            Debug.Assert((int)bone < 24 || bone == HumanBodyBones.UpperChest || bone == HumanBodyBones.LastBone, "Fingers are not supported");
            switch (bone)
            {
                case HumanBodyBones.Hips:
                    return HumanBodyBones.Hips;
                case HumanBodyBones.LeftUpperLeg:
                    return HumanBodyBones.RightUpperLeg;
                case HumanBodyBones.RightUpperLeg:
                    return HumanBodyBones.LeftUpperLeg;
                case HumanBodyBones.LeftLowerLeg:
                    return HumanBodyBones.RightLowerLeg;
                case HumanBodyBones.RightLowerLeg:
                    return HumanBodyBones.LeftLowerLeg;
                case HumanBodyBones.LeftFoot:
                    return HumanBodyBones.RightFoot;
                case HumanBodyBones.RightFoot:
                    return HumanBodyBones.LeftFoot;
                case HumanBodyBones.Spine:
                    return HumanBodyBones.Spine;
                case HumanBodyBones.Chest:
                    return HumanBodyBones.Chest;
                case HumanBodyBones.Neck:
                    return HumanBodyBones.Neck;
                case HumanBodyBones.Head:
                    return HumanBodyBones.Head;
                case HumanBodyBones.LeftShoulder:
                    return HumanBodyBones.RightShoulder;
                case HumanBodyBones.RightShoulder:
                    return HumanBodyBones.LeftShoulder;
                case HumanBodyBones.LeftUpperArm:
                    return HumanBodyBones.RightUpperArm;
                case HumanBodyBones.RightUpperArm:
                    return HumanBodyBones.LeftUpperArm;
                case HumanBodyBones.LeftLowerArm:
                    return HumanBodyBones.RightLowerArm;
                case HumanBodyBones.RightLowerArm:
                    return HumanBodyBones.LeftLowerArm;
                case HumanBodyBones.LeftHand:
                    return HumanBodyBones.RightHand;
                case HumanBodyBones.RightHand:
                    return HumanBodyBones.LeftHand;
                case HumanBodyBones.LeftToes:
                    return HumanBodyBones.RightToes;
                case HumanBodyBones.RightToes:
                    return HumanBodyBones.LeftToes;
                case HumanBodyBones.LeftEye:
                    return HumanBodyBones.RightEye;
                case HumanBodyBones.RightEye:
                    return HumanBodyBones.LeftEye;
                case HumanBodyBones.Jaw:
                    return HumanBodyBones.Jaw;
                case HumanBodyBones.UpperChest:
                    return HumanBodyBones.UpperChest;
                default:
                    return HumanBodyBones.LastBone;
            }
        }

        /// <summary>
        /// Returns true if the bone belongs to the right arm (RightUpperArm or RightLowerArm or RightHand)
        /// </summary>
        public static bool IsRightArmBone(HumanBodyBones bone)
        {
            return bone == HumanBodyBones.RightUpperArm || bone == HumanBodyBones.RightLowerArm || bone == HumanBodyBones.RightHand;
        }
        /// <summary>
        /// Returns true if the bone belongs to the left arm (LeftUpperArm or LeftLowerArm or LeftHand)
        /// </summary>
        public static bool IsLeftArmBone(HumanBodyBones bone)
        {
            return bone == HumanBodyBones.LeftUpperArm || bone == HumanBodyBones.LeftLowerArm || bone == HumanBodyBones.LeftHand;
        }
    }
}