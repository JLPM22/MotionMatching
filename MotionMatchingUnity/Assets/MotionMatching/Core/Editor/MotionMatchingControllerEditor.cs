using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace MotionMatching
{
    [CustomEditor(typeof(MotionMatchingController))]
    public class MotionMatchingControllerEditor : Editor
    {
        private bool ToggleFeatureWeights;
        private bool ToggleProfiling;
        private bool ToggleDebug;

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            MotionMatchingController controller = (MotionMatchingController)target;

            // Feature Weights
            if (controller.FeatureWeights.Length != (controller.MMData.TrajectoryFeatures.Count + controller.MMData.PoseFeatures.Count))
            {
                float[] newWeights = new float[controller.MMData.TrajectoryFeatures.Count + controller.MMData.PoseFeatures.Count];
                for (int i = 0; i < newWeights.Length; ++i) newWeights[i] = 1.0f;
                for (int i = 0; i < Mathf.Min(controller.FeatureWeights.Length, newWeights.Length); i++) newWeights[i] = controller.FeatureWeights[i];
                controller.FeatureWeights = newWeights;
            }
            ToggleFeatureWeights = EditorGUILayout.BeginFoldoutHeaderGroup(ToggleFeatureWeights, "Feature Weights");
            if (ToggleFeatureWeights)
            {
                EditorGUI.indentLevel++;
                EditorGUILayout.BeginVertical(GUI.skin.box);
                for (int i = 0; i < controller.MMData.TrajectoryFeatures.Count; ++i)
                {
                    string name = controller.MMData.TrajectoryFeatures[i].Name;
                    controller.FeatureWeights[i] = EditorGUILayout.FloatField(name, controller.FeatureWeights[i]);
                }
                EditorGUILayout.EndVertical();
                EditorGUILayout.BeginVertical(GUI.skin.box);
                for (int i = 0; i < controller.MMData.PoseFeatures.Count; ++i)
                {
                    string name = controller.MMData.PoseFeatures[i].Name;
                    int index = controller.MMData.TrajectoryFeatures.Count + i;
                    controller.FeatureWeights[index] = EditorGUILayout.FloatField(name, controller.FeatureWeights[index]);
                }
                EditorGUILayout.EndVertical();
                EditorGUI.indentLevel--;
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            // Debug & Profiling
            if (!Application.isPlaying) return;

            ToggleDebug = EditorGUILayout.BeginFoldoutHeaderGroup(ToggleDebug, "Debug");
            if (ToggleDebug)
            {
                EditorGUILayout.LabelField("Pose Index: " + controller.GetCurrentFrame());
            }
            EditorGUILayout.EndFoldoutHeaderGroup();

            ToggleProfiling = EditorGUILayout.BeginFoldoutHeaderGroup(ToggleProfiling, "Profiling");
            if (ToggleProfiling)
            {
                if (PROFILE.IS_PROFILING_ENABLED())
                {
                    PROFILE.DATA totalMotionMatchingData = PROFILE.GET_DATA("Motion Matching Total");
                    if (totalMotionMatchingData != null)
                    {
                        EditorGUILayout.Space();
                        EditorGUILayout.LabelField("Total:");
                        EditorGUILayout.LabelField("Min: " + totalMotionMatchingData.MinMs.ToString("F2") + "ms" + " (" + totalMotionMatchingData.MinTicks.ToString("F0") + " ticks)");
                        EditorGUILayout.LabelField("Max: " + totalMotionMatchingData.MaxMs.ToString("F2") + "ms" + " (" + totalMotionMatchingData.MaxTicks.ToString("F0") + " ticks)");
                        EditorGUILayout.LabelField("Avg: " + totalMotionMatchingData.GetAverageMs().ToString("F2") + "ms" + " (" + totalMotionMatchingData.GetAverageTicks().ToString("F0") + " ticks)");
                    }

                    PROFILE.DATA searchMotionMatchingData = PROFILE.GET_DATA("Motion Matching Search");
                    if (searchMotionMatchingData != null)
                    {
                        EditorGUILayout.Space();
                        EditorGUILayout.LabelField("Search:");
                        EditorGUILayout.LabelField("Min: " + searchMotionMatchingData.MinMs.ToString("F2") + "ms" + " (" + searchMotionMatchingData.MinTicks.ToString("F0") + " ticks)");
                        EditorGUILayout.LabelField("Max: " + searchMotionMatchingData.MaxMs.ToString("F2") + "ms" + " (" + searchMotionMatchingData.MaxTicks.ToString("F0") + " ticks)");
                        EditorGUILayout.LabelField("Avg: " + searchMotionMatchingData.GetAverageMs().ToString("F2") + "ms" + " (" + searchMotionMatchingData.GetAverageTicks().ToString("F0") + " ticks)");
                    }
                }
            }
            EditorGUILayout.EndFoldoutHeaderGroup();
        }
    }
}