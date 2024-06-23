using UnityEditor;
using UnityEngine;

namespace MotionMatching
{
    public class MotionMatchingDocumentation : EditorWindow
    {
        [MenuItem("MotionMatching/Documentation", priority = 100000)]
        public static void ShowWindow()
        {
            Application.OpenURL("https://jlpm22.github.io/motionmatching-docs/");
        }
    }
}