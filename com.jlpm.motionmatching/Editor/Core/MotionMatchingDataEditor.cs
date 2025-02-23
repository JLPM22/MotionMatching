using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using System.Collections.Generic;
using Unity.Mathematics;
using System;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;

    [CustomEditor(typeof(MotionMatchingData))]
    public class MotionMatchingDataEditor : Editor
    {
        private bool SkeletonToMecanimFoldout;
        private bool FeatureSelectorFoldout;

        public void GenerateDatabases(MotionMatchingData mmData)
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Pose Extract");
            mmData.ImportPoseSet();
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Extract");

            PROFILE.BEGIN_SAMPLE_PROFILING("Pose Serialize");
            PoseSerializer poseSerializer = new();
            poseSerializer.Serialize(mmData.PoseSet, mmData.GetAssetPath(), this.name);
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Serialize");

            mmData.ComputeJointsLocalForward();

            PROFILE.BEGIN_SAMPLE_PROFILING("Feature Extract");
            mmData.ImportFeatureSet();
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Extract");

            PROFILE.BEGIN_SAMPLE_PROFILING("Feature Serialize");
            FeatureSerializer featureSerializer = new();
            featureSerializer.Serialize(mmData.FeatureSet, mmData, mmData.GetAssetPath(), this.name);
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Serialize");

            mmData.Dispose();

            AssetDatabase.Refresh();
        }

        public override void OnInspectorGUI()
        {
            MotionMatchingData data = (MotionMatchingData)target;

            bool generateButtonError = false;

            // BVH
            EditorGUILayout.LabelField("Animations", EditorStyles.boldLabel);
            EditorGUI.indentLevel++;
            for (int i = 0; i < (data.AnimationDatas == null ? 0 : data.AnimationDatas.Count); i++)
            {
                data.AnimationDatas[i] = (AnimationData)EditorGUILayout.ObjectField(data.AnimationDatas[i], typeof(AnimationData), false);
            }
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Add Animation"))
            {
                data.AnimationDatas ??= new List<AnimationData>();
                data.AnimationDatas.Add(null);
            }
            if (GUILayout.Button("Remove Animation"))
            {
                data.AnimationDatas.RemoveAt(data.AnimationDatas.Count - 1);
            }
            EditorGUILayout.EndHorizontal();
            EditorGUI.indentLevel--;
            if (data.AnimationDatas == null) return;

            // BVH TPose
            EditorGUILayout.Separator();
            data.AnimationDataTPose = (AnimationData)EditorGUILayout.ObjectField(new GUIContent("Animation with T-Pose", "Animation with a T-Pose in the first frame, used for retargeting"),
                                                                                 data.AnimationDataTPose, typeof(AnimationData), false);

            // Hips Local Vectors --------
            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Hips Local Vectors", EditorStyles.boldLabel);
            if (GUILayout.Button("Auto-Set Hips Vectors", GUILayout.Width(150)))
            {
                HipsLocalVectorsHelperEditorWindow.ShowWindow(data);
            }
            EditorGUILayout.EndHorizontal();
            EditorGUI.indentLevel++;
            // DefaultHipsForward
            data.HipsForwardLocalVector = EditorGUILayout.Vector3Field(new GUIContent("Forward Vector", "Local vector (axis) pointing in the forward direction of the hips"),
                                                                       data.HipsForwardLocalVector);
            if (math.abs(math.length(data.HipsForwardLocalVector) - 1.0f) > 1E-6f)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.HelpBox("Hips Forward Local Vector should be normalized", MessageType.Error);
                if (GUILayout.Button("Fix")) data.HipsForwardLocalVector = math.normalize(data.HipsForwardLocalVector);
                EditorGUILayout.EndHorizontal();
                generateButtonError = true;
            }
            // HipsUpLocalVector
            data.HipsUpLocalVector = EditorGUILayout.Vector3Field(new GUIContent("Up Vector", "Local vector (axis) pointing in the up direction of the hips"),
                                                           data.HipsUpLocalVector);
            if (math.abs(math.length(data.HipsUpLocalVector) - 1.0f) > 1E-6f)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.HelpBox("Hips Up Local Vector should be normalized", MessageType.Error);
                if (GUILayout.Button("Fix")) data.HipsUpLocalVector = math.normalize(data.HipsUpLocalVector);
                EditorGUILayout.EndHorizontal();
                generateButtonError = true;
            }
            EditorGUI.indentLevel--;

            // SmoothSimulationBone
            //data.SmoothSimulationBone = EditorGUILayout.Toggle(new GUIContent("Smooth Simulation Bone", "Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter"),
            //                                                   data.SmoothSimulationBone);

            // ContactVelocityThreshold
            EditorGUILayout.Separator();
            data.ContactVelocityThreshold = EditorGUILayout.FloatField(new GUIContent("Contact Velocity Threshold", "Minimum velocity of the foot to be considered in movement and not in contact with the ground"),
                                                                        data.ContactVelocityThreshold);

            // SkeletonToMecanim
            EditorGUILayout.Separator();
            if (data.AnimationDataTPose == null)
            {
                EditorGUILayout.HelpBox("Animation with T-Pose not set", MessageType.Warning);
                return;
            }
            if (GUILayout.Button("Read Skeleton from BVH"))
            {
                BVHAnimation animation = data.AnimationDataTPose.GetAnimation();
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
                EditorGUI.indentLevel++;
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
                EditorGUI.indentLevel--;
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            // Display Feature Selector
            FeatureSelectorFoldout = EditorGUILayout.BeginFoldoutHeaderGroup(FeatureSelectorFoldout, "Feature Selector");
            if (FeatureSelectorFoldout)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.LabelField("Trajectory Features", EditorStyles.boldLabel);
                for (int i = 0; i < data.TrajectoryFeatures.Count; i++)
                {
                    MotionMatchingData.TrajectoryFeature trajectoryFeature = data.TrajectoryFeatures[i];
                    // Header
                    EditorGUILayout.BeginVertical(GUI.skin.box);
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField((i + 1).ToString());
                    GUILayout.FlexibleSpace();
                    if (GUILayout.Button("x"))
                    {
                        data.TrajectoryFeatures.RemoveAt(i--);
                    }
                    EditorGUILayout.EndHorizontal();
                    // Name
                    trajectoryFeature.Name = EditorGUILayout.TextField("Name", trajectoryFeature.Name);
                    // Feature Type
                    trajectoryFeature.FeatureType = (MotionMatchingData.TrajectoryFeature.Type)EditorGUILayout.EnumPopup("Type", trajectoryFeature.FeatureType);
                    // Frames
                    EditorGUILayout.LabelField("Frames Prediction");
                    EditorGUILayout.BeginHorizontal();
                    for (int j = 0; j < trajectoryFeature.FramesPrediction.Length; j++)
                    {
                        trajectoryFeature.FramesPrediction[j] = EditorGUILayout.IntField(trajectoryFeature.FramesPrediction[j]);
                    }
                    if (GUILayout.Button("Add"))
                    {
                        int[] newFrames = new int[trajectoryFeature.FramesPrediction.Length + 1];
                        for (int j = 0; j < trajectoryFeature.FramesPrediction.Length; j++) newFrames[j] = trajectoryFeature.FramesPrediction[j];
                        trajectoryFeature.FramesPrediction = newFrames;
                    }
                    if (trajectoryFeature.FramesPrediction.Length > 0 && GUILayout.Button("Remove"))
                    {
                        int[] newFrames = new int[trajectoryFeature.FramesPrediction.Length - 1];
                        for (int j = 0; j < trajectoryFeature.FramesPrediction.Length - 1; j++) newFrames[j] = trajectoryFeature.FramesPrediction[j];
                        trajectoryFeature.FramesPrediction = newFrames;
                    }
                    EditorGUILayout.EndHorizontal();
                    if (trajectoryFeature.FeatureType == MotionMatchingData.TrajectoryFeature.Type.Position ||
                        trajectoryFeature.FeatureType == MotionMatchingData.TrajectoryFeature.Type.Direction)
                    {
                        // Bone
                        trajectoryFeature.SimulationBone = EditorGUILayout.Toggle("Simulation Bone", trajectoryFeature.SimulationBone);
                        if (!trajectoryFeature.SimulationBone)
                        {
                            trajectoryFeature.Bone = (HumanBodyBones)EditorGUILayout.EnumPopup("Bone", trajectoryFeature.Bone);
                            GUI.enabled = !trajectoryFeature.ZeroY || !trajectoryFeature.ZeroZ;
                            trajectoryFeature.ZeroX = EditorGUILayout.Toggle("Zero X", trajectoryFeature.ZeroX);
                            GUI.enabled = !trajectoryFeature.ZeroX || !trajectoryFeature.ZeroZ;
                            trajectoryFeature.ZeroY = EditorGUILayout.Toggle("Zero Y", trajectoryFeature.ZeroY);
                            GUI.enabled = !trajectoryFeature.ZeroX || !trajectoryFeature.ZeroY;
                            trajectoryFeature.ZeroZ = EditorGUILayout.Toggle("Zero Z", trajectoryFeature.ZeroZ);
                            GUI.enabled = true;
                        }
                        else
                        {
                            trajectoryFeature.ZeroX = false;
                            trajectoryFeature.ZeroY = true; // project simulation bone to the ground
                            trajectoryFeature.ZeroZ = false;
                        }
                        if (trajectoryFeature.FeatureType == MotionMatchingData.TrajectoryFeature.Type.Position)
                        {
                            // TODO: incorporate visualization position here
                        }
                    }
                    else
                    {
                        if (trajectoryFeature.FeatureType == MotionMatchingData.TrajectoryFeature.Type.Custom1D)
                        {
                            trajectoryFeature.FeatureExtractor = EditorGUILayout.ObjectField(new GUIContent("Feature1DExtractor", "ScriptableObject inheriting from the 'Feature1DExtractor' class"), trajectoryFeature.FeatureExtractor, typeof(Feature1DExtractor), false) as ScriptableObject;
                            if (trajectoryFeature.FeatureExtractor == null)
                            {
                                EditorGUILayout.HelpBox("Please enter an instance of a ScriptableObject inheriting from the 'Feature1DExtractor' class",
                                                        MessageType.Error);
                                generateButtonError = true;
                            }
                        }
                        else if (trajectoryFeature.FeatureType == MotionMatchingData.TrajectoryFeature.Type.Custom2D)
                        {
                            trajectoryFeature.FeatureExtractor = EditorGUILayout.ObjectField(new GUIContent("Feature2DExtractor", "ScriptableObject inheriting from the 'Feature2DExtractor' class"), trajectoryFeature.FeatureExtractor, typeof(Feature2DExtractor), false) as ScriptableObject;
                            if (trajectoryFeature.FeatureExtractor == null)
                            {
                                EditorGUILayout.HelpBox("Please enter an instance of a ScriptableObject inheriting from the 'Feature2DExtractor' class",
                                                        MessageType.Error);
                                generateButtonError = true;
                            }
                        }
                        else if (trajectoryFeature.FeatureType == MotionMatchingData.TrajectoryFeature.Type.Custom3D)
                        {
                            trajectoryFeature.FeatureExtractor = EditorGUILayout.ObjectField(new GUIContent("Feature3DExtractor", "ScriptableObject inheriting from the 'Feature3DExtractor' class"), trajectoryFeature.FeatureExtractor, typeof(Feature3DExtractor), false) as ScriptableObject;
                            if (trajectoryFeature.FeatureExtractor == null)
                            {
                                EditorGUILayout.HelpBox("Please enter an instance of a ScriptableObject inheriting from the 'Feature3DExtractor' class",
                                                        MessageType.Error);
                                generateButtonError = true;
                            }
                        }
                        else if (trajectoryFeature.FeatureType == MotionMatchingData.TrajectoryFeature.Type.Custom4D)
                        {
                            trajectoryFeature.FeatureExtractor = EditorGUILayout.ObjectField(new GUIContent("Feature4DExtractor", "ScriptableObject inheriting from the 'Feature4DExtractor' class"), trajectoryFeature.FeatureExtractor, typeof(Feature4DExtractor), false) as ScriptableObject;
                            if (trajectoryFeature.FeatureExtractor == null)
                            {
                                EditorGUILayout.HelpBox("Please enter an instance of a ScriptableObject inheriting from the 'Feature4DExtractor' class",
                                                        MessageType.Error);
                                generateButtonError = true;
                            }
                        }
                    }
                    EditorGUILayout.EndVertical();
                }
                if (GUILayout.Button("Add Trajectory Feature"))
                {
                    data.TrajectoryFeatures.Add(new MotionMatchingData.TrajectoryFeature());
                }
                EditorGUILayout.LabelField("Pose Features", EditorStyles.boldLabel);
                for (int i = 0; i < data.PoseFeatures.Count; i++)
                {
                    MotionMatchingData.PoseFeature poseFeature = data.PoseFeatures[i];
                    // Header
                    EditorGUILayout.BeginVertical(GUI.skin.box);
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.LabelField((i + 1).ToString());
                    GUILayout.FlexibleSpace();
                    if (GUILayout.Button("x"))
                    {
                        data.PoseFeatures.RemoveAt(i--);
                    }
                    EditorGUILayout.EndHorizontal();
                    //  Properties
                    poseFeature.Name = EditorGUILayout.TextField("Name", poseFeature.Name);
                    poseFeature.FeatureType = (MotionMatchingData.PoseFeature.Type)EditorGUILayout.EnumPopup("Type", poseFeature.FeatureType);
                    poseFeature.Bone = (HumanBodyBones)EditorGUILayout.EnumPopup(poseFeature.Bone);
                    EditorGUILayout.EndVertical();
                }
                if (GUILayout.Button("Add Pose Feature"))
                {
                    data.PoseFeatures.Add(new MotionMatchingData.PoseFeature());
                }
                EditorGUI.indentLevel--;
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            // Generate Databases
            EditorGUILayout.Separator();
            if (generateButtonError)
            {
                EditorGUILayout.HelpBox("Oops! Looks like there are errors in the feature definition. Please resolve them before generating the databases.", MessageType.Error);
                GUI.enabled = false;
            }
            if (GUILayout.Button("Generate Databases", GUILayout.Height(30)))
            {
                GenerateDatabases(data);
            }
            GUI.enabled = true;

            // Error Check
            if (data.JointsLocalForwardError)
            {
                EditorGUILayout.HelpBox("Internal error detected. Please regenerate databases.", MessageType.Error);
            }

            // Save
            if (GUI.changed)
            {
                EditorUtility.SetDirty(target);
                EditorSceneManager.MarkSceneDirty(EditorSceneManager.GetActiveScene());
            }
        }
    }
}
