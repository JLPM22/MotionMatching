using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

namespace MotionMatching
{
    [CustomEditor(typeof(MotionMatchingController))]
    public class MotionMatchingControllerEditor : Editor
    {
        private bool ToggleProfiling;
        private bool ToggleDebug;

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            MotionMatchingController controller = (MotionMatchingController)target;

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

                    EditorGUILayout.Space();
                    EditorGUILayout.LabelField("Memory is approximated", EditorStyles.boldLabel);

                    EditorGUILayout.Space();
                    long MiB = PROFILE.GET_MEMORY("Pose Extract") / (1024 * 1024);
                    EditorGUILayout.LabelField("Pose Memory: " + MiB + " MiB");

                    EditorGUILayout.Space();
                    MiB = PROFILE.GET_MEMORY("Feature Extract") / (1024 * 1024);
                    EditorGUILayout.LabelField("Feature Memory: " + MiB + " MiB");
                }
            }
            EditorGUILayout.EndFoldoutHeaderGroup();
        }
    }
}