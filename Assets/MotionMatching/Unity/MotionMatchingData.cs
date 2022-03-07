using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using System.IO;
#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
#endif

namespace MotionMatching
{
    using Joint = Skeleton.Joint;
    /// <summary>
    /// Defines all data used for Motion Matching in one avatar
    /// Contains animation clips, mapping between the skeleton and Mecanim, and other data
    /// </summary>
    [CreateAssetMenu(fileName = "MotionMatchingData", menuName = "MotionMatching/MotionMatchingData")]
    public class MotionMatchingData : ScriptableObject
    {
        // TODO: add a way to add multiple animation clips (BVHs)

        // TODO: DefaultHipsForward... detect/suggest automatically? try to fix automatically at BVHAnimation level? 
        // (if it is fixed some code can be deleted... all code related to DefaultHipsForward and in the UpdateTransform() when correcting the hips forward)

        public TextAsset BVH;
        public TextAsset BVHTPose; // BVH with a TPose in the first frame, used for retargeting
        public float UnitScale = 1.0f;
        public float3 HipsForwardLocalVector = new float3(0, 0, 1); // Local vector (axis) pointing in the forward direction of the hips
        public float MinimumPoseVelocity = 0.0001f; // Minimum velocity between poses (less than this value will be clamped to 0)
        public List<JointToMecanim> SkeletonToMecanim = new List<JointToMecanim>();

        private BVHAnimation Animation;
        private PoseSet PoseSet;

        public BVHAnimation GetOrImportAnimation()
        {
            if (Animation == null)
            {
                PROFILE.BEGIN_SAMPLE_PROFILING("BVH Import");
                BVHImporter importer = new BVHImporter();
                Animation = importer.Import(BVH, UnitScale);
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("BVH Import");

                // Add Mecanim mapping information
                Animation.UpdateMecanimInformation(this);
            }
            return Animation;
        }

        public PoseSet GetOrImportPoseSet()
        {
            if (PoseSet == null)
            {
                PROFILE.BEGIN_SAMPLE_PROFILING("Pose Import");
                PoseSerializer serializer = new PoseSerializer();
                if (!serializer.Deserialize(GetAssetPath(), name, out PoseSet))
                {
                    Debug.LogError("Failed to read pose set. Creating it in runtime instead.");
                    BVHAnimation animation = GetOrImportAnimation();
                    PoseExtractor poseExtractor = new PoseExtractor();
                    PoseSet = new PoseSet();
                    if (!poseExtractor.Extract(animation, PoseSet, this))
                    {
                        Debug.LogError("[FeatureDebug] Failed to extract pose from BVHAnimation");
                        return null;
                    }
                }
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Import");
            }
            return PoseSet;
        }

        public bool GetMecanimBone(string jointName, out HumanBodyBones bone)
        {
            for (int i = 0; i < SkeletonToMecanim.Count; i++)
            {
                if (SkeletonToMecanim[i].Name == jointName)
                {
                    bone = SkeletonToMecanim[i].MecanimBone;
                    return true;
                }
            }
            bone = HumanBodyBones.LastBone;
            return false;
        }

        public bool GetJointName(HumanBodyBones bone, out string jointName)
        {
            for (int i = 0; i < SkeletonToMecanim.Count; i++)
            {
                if (SkeletonToMecanim[i].MecanimBone == bone)
                {
                    jointName = SkeletonToMecanim[i].Name;
                    return true;
                }
            }
            jointName = "";
            return false;
        }

        public string GetAssetPath()
        {
            string assetPath = AssetDatabase.GetAssetPath(this);
            return Path.Combine(Application.dataPath, assetPath.Remove(assetPath.Length - ".asset".Length, 6).Remove(0, "Assets".Length + 1));
        }

        [System.Serializable]
        public struct JointToMecanim
        {
            public string Name;
            public HumanBodyBones MecanimBone;

            public JointToMecanim(string name, HumanBodyBones mecanimBone)
            {
                Name = name;
                MecanimBone = mecanimBone;
            }
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(MotionMatchingData))]
    public class MotionMatchingDataEditor : Editor
    {
        private bool SkeletonToMecanimFoldout;

        public override void OnInspectorGUI()
        {
            MotionMatchingData data = (MotionMatchingData)target;

            // BVH
            data.BVH = (TextAsset)EditorGUILayout.ObjectField("BVH", data.BVH, typeof(TextAsset), false);
            if (data.BVH == null) return;
            data.BVHTPose = (TextAsset)EditorGUILayout.ObjectField(new GUIContent("BVH with TPose", "BVH with a TPose in the first frame, used for retargeting"),
                                                                   data.BVHTPose, typeof(TextAsset), false);
            // UnitScale
            data.UnitScale = EditorGUILayout.FloatField("Unit Scale", data.UnitScale);
            // DefaultHipsForward
            data.HipsForwardLocalVector = EditorGUILayout.Vector3Field(new GUIContent("Hips Forward Local Vector", "Local vector (axis) pointing in the forward direction of the hips"),
                                                                       data.HipsForwardLocalVector);
            if (math.abs(math.length(data.HipsForwardLocalVector) - 1.0f) > 1E-6f)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.HelpBox("Hips Forward Local Vector should be normalized", MessageType.Warning);
                if (GUILayout.Button("Fix")) data.HipsForwardLocalVector = math.normalize(data.HipsForwardLocalVector);
                EditorGUILayout.EndHorizontal();
            }
            // MinimumVelocity
            data.MinimumPoseVelocity = EditorGUILayout.FloatField(new GUIContent("Minimum Pose Velocity", "Minimum velocity between poses (less than this value will be clamped to 0)"),
                                                                  data.MinimumPoseVelocity);
            if (data.MinimumPoseVelocity < 0) data.MinimumPoseVelocity = 0;

            if (GUILayout.Button("Read Skeleton from BVH"))
            {
                BVHImporter importer = new BVHImporter();
                BVHAnimation animation = importer.Import(data.BVH, data.UnitScale);
                // Check if SkeletonToMecanim should be reset
                bool shouldResetSkeletonToMecanim = true || data.SkeletonToMecanim.Count != animation.Skeleton.Joints.Count;
                if (!shouldResetSkeletonToMecanim)
                {
                    foreach (MotionMatchingData.JointToMecanim jtm in data.SkeletonToMecanim)
                    {
                        if (!animation.Skeleton.Find(jtm.Name, out _))
                        {
                            shouldResetSkeletonToMecanim = true;
                            break;
                        }
                    }
                }
                if (shouldResetSkeletonToMecanim)
                {
                    data.SkeletonToMecanim.Clear();
                    foreach (Joint joint in animation.Skeleton.Joints)
                    {
                        HumanBodyBones bone;
                        try
                        {
                            bone = (HumanBodyBones)Enum.Parse(typeof(HumanBodyBones), joint.Name);
                        }
                        catch (Exception)
                        {
                            bone = HumanBodyBones.LastBone;
                        }
                        data.SkeletonToMecanim.Add(new MotionMatchingData.JointToMecanim(joint.Name, bone));
                    }
                }
            }

            // Display SkeletonToMecanim
            SkeletonToMecanimFoldout = EditorGUILayout.BeginFoldoutHeaderGroup(SkeletonToMecanimFoldout, "Skeleton to Mecanim");
            if (SkeletonToMecanimFoldout)
            {
                for (int i = 0; i < data.SkeletonToMecanim.Count; i++)
                {
                    MotionMatchingData.JointToMecanim jtm = data.SkeletonToMecanim[i];
                    EditorGUILayout.BeginHorizontal();
                    GUI.contentColor = jtm.MecanimBone == HumanBodyBones.LastBone ? new Color(1.0f, 0.6f, 0.6f) : Color.white;
                    HumanBodyBones newHumanBodyBone = (HumanBodyBones)EditorGUILayout.EnumPopup(jtm.Name, jtm.MecanimBone);
                    GUI.contentColor = Color.white;
                    jtm.MecanimBone = newHumanBodyBone;
                    data.SkeletonToMecanim[i] = jtm;
                    EditorGUILayout.EndHorizontal();
                }
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            // Generate Databases
            if (GUILayout.Button("Generate Databases"))
            {
                BVHAnimation animation = data.GetOrImportAnimation();

                PROFILE.BEGIN_SAMPLE_PROFILING("Pose Extract", true);
                PoseExtractor poseExtractor = new PoseExtractor();
                PoseSet poseSet = new PoseSet();
                if (!poseExtractor.Extract(animation, poseSet, data))
                {
                    Debug.LogError("[FeatureDebug] Failed to extract pose from BVHAnimation");
                }
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Extract", true);

                PROFILE.BEGIN_SAMPLE_PROFILING("Pose Serialize", true);
                PoseSerializer poseSerializer = new PoseSerializer();
                poseSerializer.Serialize(poseSet, data.GetAssetPath(), data.name);
                AssetDatabase.Refresh();
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Serialize", true);
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