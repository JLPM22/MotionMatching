using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using UnityEditor;
using UnityEngine.UIElements;
using UnityEditor.UIElements;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using UnityEditor.PackageManager.UI;
using System.Runtime.CompilerServices;

namespace MotionMatching
{
    /* New Scene thanks to: https://gist.github.com/ulrikdamm/338392c3b0900de225ec6dd10864cab4 */
    public class AnimationViewerEditorWindow : EditorWindow
    {
        public static AnimationViewerEditorWindow Window;
        private static SceneGUI ReturnButton;

        private BVHAnimation Animation;
        private Transform[] Skeleton;
        private int CurrentFrame = 0;
        private TextAsset BVHAsset;
        private float BVHScale = 1.0f;
        private int TargetFramerate;
        private double LastUpdateTime;

        private ObjectField BVHAssetField;
        private FloatField BVHScaleField;
        private IntegerField CurrentFrameField;
        private IntegerField TargetFramerateField;

        private VisualElement Timeline;

        [MenuItem("MotionMatching/Animation Viewer")]
        public static void ShowWindow()
        {
            Window = GetWindow<AnimationViewerEditorWindow>();
            Window.titleContent = new GUIContent("Animation Viewer");
            // Open new scene
            EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo();
            string currentScene = EditorSceneManager.GetActiveScene().path;
            Scene scene = EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            GameObject.CreatePrimitive(PrimitiveType.Plane);

            ReturnButton = new SceneGUI
            {
                PreviousScene = currentScene
            };
        }

        public void CreateGUI()
        {
            VisualElement root = rootVisualElement;

            CreateBVHAssetField(root);
            CreateScaleField(root);
            CreateFrameField(root);
            CreateTimeline(root);
        }

        private void CreateBVHAssetField(VisualElement root)
        {
            BVHAssetField = new ObjectField
            {
                label = "BVH Asset",
                objectType = typeof(TextAsset)
            };
            BVHAssetField.RegisterValueChangedCallback(x =>
            {
                BVHAsset = null;
                if (x.newValue != null && x.newValue != x.previousValue)
                {
                    BVHAsset = (TextAsset)x.newValue;
                    ImportBVH();
                }
            });
            root.Add(BVHAssetField);
        }

        private void CreateScaleField(VisualElement root)
        {
            BVHScaleField = new FloatField
            {
                label = "Scale",
                value = BVHScale,
                isDelayed = true
            };
            BVHScaleField.RegisterValueChangedCallback(x =>
            {
                if (x.newValue != x.previousValue)
                {
                    BVHScale = x.newValue;
                    ImportBVH();
                }
            });
            root.Add(BVHScaleField);
        }

        private void CreateFrameField(VisualElement root)
        {
            CurrentFrameField = new IntegerField
            {
                label = "Frame",
                value = CurrentFrame
            };
            CurrentFrameField.RegisterValueChangedCallback(x =>
            {
                UpdateCurrentFrame(x.newValue);
            });
            root.Add(CurrentFrameField);

            TargetFramerateField = new IntegerField
            {
                label = "Framerate",
                value = TargetFramerate
            };
            TargetFramerateField.RegisterValueChangedCallback(x =>
            {
                UpdateTargetFramerate(x.newValue);
            });
            root.Add(TargetFramerateField);
        }

        private void CreateTimeline(VisualElement root)
        {
            if (root.Contains(Timeline))
            {
                root.Remove(Timeline);
            }
            // Create frames rule with numbers
            int maxFrames = Animation == null ? 2 : Animation.Frames.Length;
            Timeline = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    flexShrink = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    alignItems = Align.FlexStart,
                    justifyContent = Justify.Center,
                    backgroundColor = new Color(0.6f, 0.6f, 0.6f, 1.0f)
                }
            };
            root.Add(Timeline);
            VisualElement frameRule = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    flexShrink = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    alignItems = Align.Center,
                    justifyContent = Justify.Center,
                    backgroundColor = new Color(0.4f, 0.4f, 0.4f, 1.0f),
                    color = Color.black,
                    height=20
                }
            };
            Timeline.Add(frameRule);
            
            const int maxFramesPerRule = 20;
            int framesStep = Mathf.Max(1, maxFrames / maxFramesPerRule);
            for (int i = 0; i < maxFrames; i += framesStep)
            {
                VisualElement frameLabelBox = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        flexGrow = 1,
                        flexShrink = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        color = Color.black
                    }
                };
                Label frameLabel = new Label(i.ToString());
                frameLabelBox.Add(frameLabel);
                frameRule.Add(frameLabelBox);
            }
        }

        private void ImportBVH()
        {
            RemoveSkeleton();
            BVHImporter bvhImporter = new BVHImporter();
            Animation = bvhImporter.Import(BVHAsset, BVHScale);
            UpdateTargetFramerate(Mathf.CeilToInt(1.0f / Animation.FrameTime));
            UpdateCurrentFrame(0);
            CreateTimeline(rootVisualElement);
            // Create skeleton
            Skeleton = new Transform[Animation.Skeleton.Joints.Count];
            for (int j = 0; j < Skeleton.Length; j++)
            {
                // Joints
                Skeleton.Joint joint = Animation.Skeleton.Joints[j];
                Transform t = (new GameObject()).transform;
                t.name = joint.Name;
                if (j > 0)
                {
                    t.SetParent(Skeleton[joint.ParentIndex], false);
                }
                t.localPosition = joint.LocalOffset;
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

        private void UpdateTargetFramerate(int newValue)
        {
            TargetFramerate = newValue;
            TargetFramerateField.value = TargetFramerate;
        }

        private void UpdateCurrentFrame(int newValue)
        {
            CurrentFrame = newValue;
            CurrentFrameField.value = CurrentFrame;
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

        private void Update()
        {
            if (Animation != null && LastUpdateTime + (1.0 / TargetFramerate) < EditorApplication.timeSinceStartup)
            {
                // Update skeleton
                BVHAnimation.Frame frame = Animation.Frames[CurrentFrame];
                Skeleton[0].position = frame.RootMotion;
                for (int i = 0; i < Skeleton.Length; i++)
                {
                    Skeleton[i].localRotation = frame.LocalRotations[i];
                }
                // Update frame index
                UpdateCurrentFrame((CurrentFrame + 1) % Animation.Frames.Length);
                LastUpdateTime = EditorApplication.timeSinceStartup;
            }
        }

        private void OnDestroy()
        {
            ReturnButton.ReturnScene();
        }
    }

    public sealed class SceneGUI
    {
        public string PreviousScene;

        public SceneGUI()
        {
            SceneView.duringSceneGui += RenderSceneGUI;
        }

        public void RenderSceneGUI(SceneView sceneview)
        {
            var style = new GUIStyle();
            style.margin = new RectOffset(10, 10, 10, 10);

            Handles.BeginGUI();
            GUILayout.BeginArea(new Rect(20, 20, 180, 300), style);
            var rect = EditorGUILayout.BeginVertical();
            GUI.Box(rect, GUIContent.none);

            if (GUILayout.Button("Return", new GUILayoutOption[0]))
            {
                ReturnScene();
                if (AnimationViewerEditorWindow.Window != null) AnimationViewerEditorWindow.Window.Close();
            }

            EditorGUILayout.EndVertical();
            GUILayout.EndArea();
            Handles.EndGUI();
        }

        public void ReturnScene()
        {
            SceneView.duringSceneGui -= RenderSceneGUI;
            EditorSceneManager.OpenScene(PreviousScene);
        }
    }
}