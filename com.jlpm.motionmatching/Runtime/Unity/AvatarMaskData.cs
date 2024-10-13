using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif


namespace MotionMatching
{
    [CreateAssetMenu(fileName = "NewAvatarMaskData", menuName = "MotionMatching/AvatarMaskData")]
    public class AvatarMaskData : ScriptableObject
    {
        [SerializeField]
        private bool[] JointEnabled = new bool[BodyJoints.Count];

        public bool IsEnabled(HumanBodyBones joint)
        {
            return JointEnabled[BodyJoints[joint]];
        }

        public void SetEnabled(HumanBodyBones joint, bool b)
        {
            JointEnabled[BodyJoints[joint]] = b;
        }

        public static Dictionary<HumanBodyBones, int> BodyJoints = new Dictionary<HumanBodyBones, int>
        {
            [HumanBodyBones.Spine] = 0,
            [HumanBodyBones.Chest] = 1,
            [HumanBodyBones.UpperChest] = 2,

            [HumanBodyBones.Neck] = 3,
            [HumanBodyBones.Head] = 4,

            [HumanBodyBones.LeftShoulder] = 5,
            [HumanBodyBones.LeftUpperArm] = 6,
            [HumanBodyBones.LeftLowerArm] = 7,
            [HumanBodyBones.LeftHand] = 8,

            [HumanBodyBones.RightShoulder] = 9,
            [HumanBodyBones.RightUpperArm] = 10,
            [HumanBodyBones.RightLowerArm] = 11,
            [HumanBodyBones.RightHand] = 12,

            [HumanBodyBones.LeftUpperLeg] = 13,
            [HumanBodyBones.LeftLowerLeg] = 14,
            [HumanBodyBones.LeftFoot] = 15,
            [HumanBodyBones.LeftToes] = 16,

            [HumanBodyBones.RightUpperLeg] = 17,
            [HumanBodyBones.RightLowerLeg] = 18,
            [HumanBodyBones.RightFoot] = 19,
            [HumanBodyBones.RightToes] = 20
        };
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(AvatarMaskData))]
    public class AvatarMaskDataEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            AvatarMaskData data = (AvatarMaskData)target;

            foreach (HumanBodyBones joint in AvatarMaskData.BodyJoints.Keys)
            {
                bool b = EditorGUILayout.Toggle(joint.ToString(), data.IsEnabled(joint));
                data.SetEnabled(joint, b);
            }

            // Save
            if (GUI.changed)
            {
                EditorUtility.SetDirty(target);
                EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
            }
        }
    }
#endif
}