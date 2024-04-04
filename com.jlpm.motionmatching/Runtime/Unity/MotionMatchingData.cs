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
        // TODO: DefaultHipsForward... detect/suggest automatically? try to fix automatically at BVHAnimation level? 
        // (if it is fixed some code can be deleted... all code related to DefaultHipsForward and in the UpdateTransform() when correcting the hips forward)
        
        public List<AnimationData> AnimationDatas;
        public AnimationData AnimationDataTPose; // Animation with a TPose in the first frame, used for retargeting
        public float3 HipsForwardLocalVector = new float3(0, 0, 1); // Local vector (axis) pointing in the forward direction of the hips
        public float3 HipsUpLocalVector = new float3(0, 1, 0); // Local vector (axis) pointing in the up direction of the hips
        // TODO: Implement Savitzky-Golay filter or similar low-pass filter in Unity (before I was using Python implementation)
        //public bool SmoothSimulationBone; // Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter
        public float ContactVelocityThreshold = 0.15f; // Minimum velocity of the foot to be considered in movement and not in contact with the ground
        public List<JointToMecanim> SkeletonToMecanim = new List<JointToMecanim>();
        public List<TrajectoryFeature> TrajectoryFeatures = new List<TrajectoryFeature>();
        public List<PoseFeature> PoseFeatures = new List<PoseFeature>();

        private PoseSet PoseSet;
        private FeatureSet FeatureSet;

        // Information extracted form T-Pose
        [SerializeField] private float3[] JointsLocalForward; // Local forward vector of each joint 
        public bool JointsLocalForwardError { get { return JointsLocalForward == null; } }

        private void ImportAnimations()
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("BVH Import");
            for (int i = 0; i < AnimationDatas.Count; i++)
            {
                AnimationDatas[i].GetAnimation(); // Imports the animation
                // Add Mecanim mapping information
                AnimationDatas[i].UpdateMecanimInformation(this);
            }
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("BVH Import");
        }

        public PoseSet GetOrImportPoseSet()
        {
            if (PoseSet == null)
            {
                PROFILE.BEGIN_SAMPLE_PROFILING("Pose Import");
                PoseSerializer serializer = new PoseSerializer();
                if (!serializer.Deserialize(GetAssetPath(), name, this, out PoseSet))
                {
                    Debug.LogError("Failed to read pose set. Creating it in runtime instead.");
                    ImportPoseSet();
#if UNITY_EDITOR
                    PROFILE.BEGIN_SAMPLE_PROFILING("Pose Serialize");
                    PoseSerializer poseSerializer = new PoseSerializer();
                    poseSerializer.Serialize(PoseSet, GetAssetPath(), this.name);
                    PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Serialize");
#endif
                }
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Import");
            }
            return PoseSet;
        }

        private void ImportPoseSet()
        {
            ImportAnimations();
            PoseSet = new PoseSet(this);
            PoseSet.SetSkeletonFromBVH(AnimationDatas[0].GetAnimation().Skeleton);
            for (int i = 0; i < AnimationDatas.Count; i++)
            {
                // Extract poses
                BVHAnimation animation = AnimationDatas[i].GetAnimation();
                PoseExtractor poseExtractor = new PoseExtractor();
                if (!poseExtractor.Extract(animation, PoseSet, this, out int animationClip))
                {
                    Debug.LogError("[FeatureDebug] Failed to extract pose from BVHAnimation. BVH Index: " + i);
                }
                // Add tags
                foreach (AnimationData.Tag tag in AnimationDatas[i].Tags)
                {
                    PoseSet.AddTag(animationClip, tag);
                }
            }
            PoseSet.ConvertTagsToNativeArrays();
        }

        public FeatureSet GetOrImportFeatureSet()
        {
            if (FeatureSet == null)
            {
                PROFILE.BEGIN_SAMPLE_PROFILING("Feature Import");
                FeatureSerializer serializer = new FeatureSerializer();
                if (!serializer.Deserialize(GetAssetPath(), name, this, out FeatureSet))
                {
                    Debug.LogError("Failed to read feature set. Creating it in runtime instead.");
                    ImportFeatureSet();
#if UNITY_EDITOR
                    PROFILE.BEGIN_SAMPLE_PROFILING("Feature Serialize");
                    FeatureSerializer featureSerializer = new FeatureSerializer();
                    featureSerializer.Serialize(FeatureSet, this, GetAssetPath(), this.name);
                    PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Serialize");
#endif
                }
                PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Import");
            }
            return FeatureSet;
        }

        private void ImportFeatureSet()
        {
            FeatureSet = new FeatureSet(this, PoseSet.NumberPoses);
            FeatureSet.Extract(PoseSet, this);
            FeatureSet.NormalizeFeatures();
        }

        private void ComputeJointsLocalForward()
        {
            // Import T-Pose
            BVHAnimation tposeAnimation = AnimationDataTPose.GetAnimation();
            JointsLocalForward = new float3[tposeAnimation.Skeleton.Joints.Count + 1]; // +1 for the simulation bone
            // Find forward character vector by projecting hips forward vector onto the ground
            Quaternion[] localRotations = tposeAnimation.Frames[0].LocalRotations;
            float3 hipsWorldForwardProjected = math.mul(localRotations[0], HipsForwardLocalVector);
            hipsWorldForwardProjected.y = 0;
            hipsWorldForwardProjected = math.normalize(hipsWorldForwardProjected);
            // Find right character vector by rotating Y-Axis 90 degrees (Unity is Left-Handed and Y-Axis is Up)
            float3 hipsWorldRightProjected = math.mul(quaternion.AxisAngle(math.up(), math.radians(90.0f)), hipsWorldForwardProjected);
            // Compute JointsLocalForward based on the T-Pose
            JointsLocalForward[0] = math.forward();
            for (int i = 1; i < JointsLocalForward.Length; i++)
            {
                quaternion worldRot = quaternion.identity;
                int joint = i - 1;
                while (joint != 0) // while not root
                {
                    worldRot = math.mul(localRotations[joint], worldRot);
                    joint = tposeAnimation.Skeleton.Joints[joint].ParentIndex;
                }
                worldRot = math.mul(localRotations[0], worldRot); // root
                joint = i - 1;
                // Change to Local
                if (!GetMecanimBone(tposeAnimation.Skeleton.Joints[joint].Name, out HumanBodyBones bone))
                {
                    Debug.LogError("[FeatureDebug] Failed to find Mecanim bone for joint " + tposeAnimation.Skeleton.Joints[joint].Name);
                }
                float3 worldForward = hipsWorldForwardProjected;
                if (HumanBodyBonesExtensions.IsLeftArmBone(bone))
                {
                    worldForward = -hipsWorldRightProjected;
                }
                else if (HumanBodyBonesExtensions.IsRightArmBone(bone))
                {
                    worldForward = hipsWorldRightProjected;
                }
                JointsLocalForward[i] = math.mul(math.inverse(worldRot), worldForward);
            }
        }

        /// <summary>
        /// Returns the local forward vector of the iven joint index (after adding simulation bone)
        /// Vector computed from the T-Pose BVH and HipsForwardLocalVector
        /// </summary>
        public float3 GetLocalForward(int jointIndex)
        {
            Debug.Assert(!JointsLocalForwardError, "JointsLocalForward is not initialized");
            return JointsLocalForward[jointIndex];
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
            string path = Path.Combine(Application.streamingAssetsPath, "MMDatabases", name);
#if UNITY_EDITOR
            if (!Directory.Exists(path))
            {
                Directory.CreateDirectory(path);
            }
#endif
            return path;
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

        [System.Serializable]
        public class TrajectoryFeature
        {
            public enum Type
            {
                Position,
                Direction,
                Custom1D,
                Custom2D,
                Custom3D,
                Custom4D
            }
            public string Name;
            public Type FeatureType;
            public int[] FramesPrediction = new int[0]; // Number of frames in the future for each point of the trajectory
            public bool SimulationBone; // Use the simulation bone (articial root added during pose extraction) instead of a bone
            public HumanBodyBones Bone; // Bone used to compute the trajectory in the feature set
            public bool ZeroX, ZeroY, ZeroZ; // Zero the X, Y and/or Z component of the trajectory feature
            public ScriptableObject FeatureExtractor; // Custom feature extractor for user-defined types

            public int GetSize()
            {
                if (FeatureType == Type.Position || FeatureType == Type.Direction)
                {
                    return 3 - (ZeroX ? 1 : 0) - (ZeroY ? 1 : 0) - (ZeroZ ? 1 : 0);
                }
                else if (FeatureType == Type.Custom1D)
                {
                    return 1;
                }
                else if (FeatureType == Type.Custom2D)
                {
                    return 2;
                }
                else if (FeatureType == Type.Custom3D)
                {
                    return 3;
                }
                else if (FeatureType == Type.Custom4D)
                {
                    return 4;
                }
                Debug.Assert(false, "Size not defined for FeatureType: " + FeatureType.ToString());
                return -1;
            }
        }

        [System.Serializable]
        public class PoseFeature
        {
            public enum Type
            {
                Position,
                Velocity
            }
            public string Name;
            public Type FeatureType;
            public HumanBodyBones Bone;
        }

#if UNITY_EDITOR
        public void GenerateDatabases()
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Pose Extract");
            ImportPoseSet();
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Extract");

            PROFILE.BEGIN_SAMPLE_PROFILING("Pose Serialize");
            PoseSerializer poseSerializer = new PoseSerializer();
            poseSerializer.Serialize(PoseSet, GetAssetPath(), this.name);
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Serialize");

            ComputeJointsLocalForward();

            PROFILE.BEGIN_SAMPLE_PROFILING("Feature Extract");
            ImportFeatureSet();
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Extract");

            PROFILE.BEGIN_SAMPLE_PROFILING("Feature Serialize");
            FeatureSerializer featureSerializer = new FeatureSerializer();
            featureSerializer.Serialize(FeatureSet, this, GetAssetPath(), this.name);
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Serialize");

            // Dispose
            if (PoseSet != null)
            {
                PoseSet.Dispose();
                PoseSet = null;
            }
            if (FeatureSet != null)
            {
                FeatureSet.Dispose();
                FeatureSet = null;
            }

            AssetDatabase.Refresh();
        }
#endif

        public void Dispose()
        {
            if (PoseSet != null)
            {
                PoseSet.Dispose();
                PoseSet = null;
            }
            if (FeatureSet != null)
            {
                FeatureSet.Dispose();
                FeatureSet = null;
            }
        }
    }

#if UNITY_EDITOR
    [CustomEditor(typeof(MotionMatchingData))]
    public class MotionMatchingDataEditor : Editor
    {
        private bool SkeletonToMecanimFoldout;
        private bool FeatureSelectorFoldout;

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
            data.AnimationDataTPose = (AnimationData)EditorGUILayout.ObjectField(new GUIContent("Animation with T-Pose", "Animation with a T-Pose in the first frame, used for retargeting"),
                                                                                 data.AnimationDataTPose, typeof(AnimationData), false);
            // DefaultHipsForward
            data.HipsForwardLocalVector = EditorGUILayout.Vector3Field(new GUIContent("Hips Forward Local Vector", "Local vector (axis) pointing in the forward direction of the hips"),
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
            data.HipsUpLocalVector = EditorGUILayout.Vector3Field(new GUIContent("Hips Up Local Vector", "Local vector (axis) pointing in the up direction of the hips"),
                                                           data.HipsUpLocalVector);
            if (math.abs(math.length(data.HipsUpLocalVector) - 1.0f) > 1E-6f)
            {
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.HelpBox("Hips Up Local Vector should be normalized", MessageType.Error);
                if (GUILayout.Button("Fix")) data.HipsUpLocalVector = math.normalize(data.HipsUpLocalVector);
                EditorGUILayout.EndHorizontal();
                generateButtonError = true;
            }

            // SmoothSimulationBone
            //data.SmoothSimulationBone = EditorGUILayout.Toggle(new GUIContent("Smooth Simulation Bone", "Smooth the simulation bone (articial root added during pose extraction) using Savitzky-Golay filter"),
            //                                                   data.SmoothSimulationBone);

            // ContactVelocityThreshold
            data.ContactVelocityThreshold = EditorGUILayout.FloatField(new GUIContent("Contact Velocity Threshold", "Minimum velocity of the foot to be considered in movement and not in contact with the ground"),
                                                                        data.ContactVelocityThreshold);

            // SkeletonToMecanim
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
                data.GenerateDatabases();
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
#endif
}