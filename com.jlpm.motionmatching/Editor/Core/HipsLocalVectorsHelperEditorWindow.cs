using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;

namespace MotionMatching
{
    public class HipsLocalVectorsHelperEditorWindow : EditorWindow
    {
        public static HipsLocalVectorsHelperEditorWindow Window;
        private SceneGUI ReturnButton;
        private MotionMatchingData MMData;
        private Transform[] Skeleton;
        private AnimationData AnimationData;

        public static void ShowWindow(MotionMatchingData mmData)
        {
            Window = GetWindow<HipsLocalVectorsHelperEditorWindow>();
            Window.titleContent = new GUIContent("Auto-Set Hips Local Vectors");
            // Open new scene
            EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();
            string currentScene = EditorSceneManager.GetActiveScene().path;
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);

            Window.ReturnButton = new SceneGUI(currentScene);
            SceneView.duringSceneGui += Window.UpdateGUI;

            Window.MMData = mmData;
            Window.AnimationData = mmData.AnimationDataTPose;

            Window.ImportBVH();
        }

        private void OnGUI()
        {
            EditorGUILayout.LabelField("Rotate the skeleton to align its forward direction with world Z (Blue) and its up direction with world Y (Green).  Click 'Set Hips Local Vectors' to apply.", EditorStyles.wordWrappedLabel);
            EditorGUILayout.Space();
            if (GUILayout.Button("Set Hips Local Vectors"))
            {
                MMData.HipsForwardLocalVector = Skeleton[0].InverseTransformDirection(Vector3.forward);
                MMData.HipsUpLocalVector = Skeleton[0].InverseTransformDirection(Vector3.up);

                Debug.Log("Hips Local Forward Vector is set to " + MMData.HipsForwardLocalVector);
                Debug.Log("Hips Local Up Vector is set to " + MMData.HipsUpLocalVector);

                Close();
            }
        }

        private void ImportBVH()
        {
            RemoveSkeleton();
            BVHAnimation animation = AnimationData.GetAnimation();
            // Create skeleton
            Skeleton = new Transform[animation.Skeleton.Joints.Count];
            for (int j = 0; j < Skeleton.Length; j++)
            {
                // Joints
                Skeleton.Joint joint = animation.Skeleton.Joints[j];
                Transform t = (new GameObject()).transform;
                t.name = joint.Name;
                if (j > 0)
                {
                    t.SetParent(Skeleton[joint.ParentIndex], false);
                }
                t.SetLocalPositionAndRotation(joint.LocalOffset, animation.Frames[0].LocalRotations[j]);
                Skeleton[j] = t;
                // Visual
                Transform visual = (new GameObject()).transform;
                visual.name = "Visual";
                visual.SetParent(t, false);
                visual.localScale = new Vector3(0.1f, 0.1f, 0.1f);
                visual.localPosition = Vector3.zero;
                visual.localRotation = Quaternion.identity;
                // Sphere
                Transform sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere).transform;
                sphere.name = "Sphere";
                sphere.SetParent(visual, false);
                sphere.localScale = Vector3.one;
                sphere.localPosition = Vector3.zero;
                sphere.localRotation = Quaternion.identity;
                if (j > 0)
                {
                    // Capsule
                    Transform capsule = GameObject.CreatePrimitive(PrimitiveType.Capsule).transform;
                    capsule.name = "Capsule";
                    capsule.SetParent(Skeleton[joint.ParentIndex].Find("Visual"), false);
                    float distance = Vector3.Distance(t.position, t.parent.position) * (1.0f / visual.localScale.y) * 0.5f;
                    capsule.localScale = new Vector3(0.5f, distance, 0.5f);
                    Vector3 up = (t.position - t.parent.position).normalized;
                    capsule.localPosition = t.parent.InverseTransformDirection(up) * distance;
                    capsule.localRotation = Quaternion.Inverse(t.parent.rotation) * Quaternion.LookRotation(new Vector3(-up.y, up.x, 0.0f), up);
                }
            }
        }

        private void RemoveSkeleton()
        {
            if (Skeleton != null)
            {
                for (int i = 0; i < Skeleton.Length; i++)
                {
                    if (Skeleton[i] != null)
                    {
                        DestroyImmediate(Skeleton[i].gameObject);
                        Skeleton[i] = null;
                    }
                }
                Skeleton = null;
            }
        }

        private void UpdateGUI(SceneView sceneView)
        {
            ReturnButton.RenderSceneGUI(sceneView);
        }

        private void OnDestroy()
        {
            ReturnButton.ReturnScene();
            SceneView.duringSceneGui -= UpdateGUI;
        }
    }
}