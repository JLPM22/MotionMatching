using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Unity.Collections;

namespace MotionMatching
{
    public class MMSearchDebugEditorWindow : EditorWindow
    {
        private bool GroupCurrent;
        private bool GroupQuery;
        private bool GroupDiff;

        [MenuItem("MotionMatching/SearchDebugWindow")]
        private static void ShowWindow()
        {
            var window = GetWindow<MMSearchDebugEditorWindow>();
            window.titleContent = new GUIContent("SearchDebugWindow");
            window.Show();
        }

        private void OnGUI()
        {
            if (!Application.isPlaying)
            {
                EditorGUILayout.HelpBox("This window is only available at runtime.", MessageType.Info);
                return;
            }

            var mmController = Selection.activeGameObject?.GetComponent<MotionMatchingController>();
            if (mmController == null)
            {
                EditorGUILayout.HelpBox("Please select a MotionMatchingController.", MessageType.Info);
                return;
            }
            MotionMatchingData mmData = mmController.MMData;
            FeatureSet featureSet = mmController.GetFeatureSet();

            GUI.enabled = false;
            EditorGUILayout.IntField("Last Frame", mmController.GetLastFrame());
            GUI.enabled = true;
            int currentFrame = EditorGUILayout.IntField("Current Frame", mmController.GetCurrentFrame());
            if (currentFrame != mmController.GetCurrentFrame()) mmController.SetCurrentFrame(currentFrame);

            NativeArray<float> currentFeature = new NativeArray<float>(featureSet.FeatureSize, Allocator.Temp);
            featureSet.GetFeature(currentFeature, currentFrame);

            GroupCurrent = EditorGUILayout.BeginFoldoutHeaderGroup(GroupCurrent, "Current");
            if (GroupCurrent)
            {
                DisplayFeatureVector(currentFeature, "Current Feature", mmData);
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            NativeArray<float> queryFeature = mmController.GetQueryFeature();
            GroupQuery = EditorGUILayout.BeginFoldoutHeaderGroup(GroupQuery, "Query");
            if (GroupQuery)
            {
                DisplayFeatureVector(queryFeature, "Last MM Search Query Feature", mmData);
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            GroupDiff = EditorGUILayout.BeginFoldoutHeaderGroup(GroupDiff, "Diff");
            if (GroupDiff)
            {
                NativeArray<float> featureWeights = mmController.UpdateAndGetFeatureWeights();
                float sqrDistance = 0.0f;
                for (int i = 0; i < featureSet.FeatureSize; ++i)
                {
                    float diff = currentFeature[i] - queryFeature[i];
                    sqrDistance += diff * diff * featureWeights[i];
                    currentFeature[i] = diff * diff * featureWeights[i];
                }
                DisplayFeatureVector(currentFeature, "Difference", mmData);
                EditorGUILayout.LabelField("Sqr Distance", sqrDistance.ToString(), EditorStyles.boldLabel);
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            currentFeature.Dispose();
        }

        private void DisplayFeatureVector(NativeArray<float> vector, string name, MotionMatchingData mmData)
        {
            var style = new GUIStyle(GUI.skin.textField) { alignment = TextAnchor.MiddleLeft };
            GUILayout.ExpandWidth(false);
            EditorGUILayout.BeginVertical(GUI.skin.box);
            EditorGUILayout.LabelField(name);
            int offset = 0;
            for (int t = 0; t < mmData.TrajectoryFeatures.Count; t++)
            {
                var feature = mmData.TrajectoryFeatures[t];
                int featureSize = feature.GetSize();
                EditorGUILayout.LabelField(feature.Name);
                for (int p = 0; p < feature.FramesPrediction.Length; p++)
                {
                    EditorGUILayout.BeginHorizontal();
                    for (int i = 0; i < featureSize; i++)
                    {
                        EditorGUILayout.LabelField(vector[offset + i].ToString("F3"), style, GUILayout.ExpandWidth(false), GUILayout.MaxWidth(60));
                    }
                    EditorGUILayout.EndHorizontal();
                    offset += featureSize;
                }
            }
            for (int p = 0; p < mmData.PoseFeatures.Count; p++)
            {
                var feature = mmData.PoseFeatures[p];
                EditorGUILayout.LabelField(feature.Name);
                EditorGUILayout.BeginHorizontal();
                EditorGUILayout.LabelField(vector[offset + 0].ToString("F3"), style, GUILayout.ExpandWidth(false), GUILayout.MaxWidth(60));
                EditorGUILayout.LabelField(vector[offset + 1].ToString("F3"), style, GUILayout.ExpandWidth(false), GUILayout.MaxWidth(60));
                EditorGUILayout.LabelField(vector[offset + 2].ToString("F3"), style, GUILayout.ExpandWidth(false), GUILayout.MaxWidth(60));
                EditorGUILayout.EndHorizontal();
                offset += 3;
            }
            EditorGUILayout.EndVertical();
        }

        void OnInspectorUpdate()
        {
            // Call Repaint on OnInspectorUpdate as it repaints the windows
            // less times as if it was OnGUI/Update
            Repaint();
        }
    }
}