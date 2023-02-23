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
using System;

namespace MotionMatching
{
    /* More info on how to create a new scene: https://gist.github.com/ulrikdamm/338392c3b0900de225ec6dd10864cab4 */
    public class AnimationViewerEditorWindow : EditorWindow
    {
        public static AnimationViewerEditorWindow Window;
        private SceneGUI ReturnButton;

        private BVHAnimation Animation;
        private Transform[] Skeleton;
        private int CurrentFrame = 0;
        private TextAsset BVHAsset;
        private float BVHScale = 1.0f;
        private int TargetFramerate;
        private double LastUpdateTime;

        private IntegerField CurrentFrameField;
        private IntegerField TargetFramerateField;

        private VisualElement Timeline;
        private VisualElement CurrentFrameIndicator;

        private List<Tag> Tags;

        private int NumberFrames { get { return Animation.Frames.Length; } }

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
            
            Window.ReturnButton = new SceneGUI(currentScene);
            SceneView.duringSceneGui += Window.UpdateGUI;
        }

        public void CreateGUI()
        {
            VisualElement root = rootVisualElement;
            root.RegisterCallback<PointerMoveEvent>(OnPointerMoveRoot, TrickleDown.TrickleDown);
            root.RegisterCallback<PointerUpEvent>(OnPointerUpRoot, TrickleDown.TrickleDown);
            CreateBVHAssetField(root);
        }

        private void CreateBVHAssetField(VisualElement root)
        {
            ObjectField bvhAssetField = new ObjectField
            {
                label = "BVH Asset",
                objectType = typeof(TextAsset)
            };
            bvhAssetField.RegisterValueChangedCallback(x =>
            {
                BVHAsset = null;
                if (x.newValue != null && x.newValue != x.previousValue)
                {
                    BVHAsset = (TextAsset)x.newValue;
                    CreateBVHFields();
                    ImportBVH();
                    ImportTags();
                    CreateTimeline(rootVisualElement);
                    UpdateCurrentFrame(0);
                    UpdateAnimation(forward: false);
                }
            });
            root.Add(bvhAssetField);
        }

        private void CreateScaleField(VisualElement root)
        {
            FloatField bvhScaleField = new FloatField
            {
                label = "Scale",
                value = BVHScale,
                isDelayed = true
            };
            bvhScaleField.RegisterValueChangedCallback(x =>
            {
                if (x.newValue != x.previousValue)
                {
                    BVHScale = x.newValue;
                    ImportBVH();
                }
            });
            root.Add(bvhScaleField);
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

        const float FrameRuleHeight = 20;
        const float PlayButtonWidth = 40;
        private void CreateTimeline(VisualElement root)
        {
            if (root.Contains(Timeline))
            {
                root.Remove(Timeline);
            }
            // Create frames rule with numbers
            Timeline = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Column,
                    flexGrow = 1,
                    flexShrink = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = new Color(0.6f, 0.6f, 0.6f, 1.0f)
                }
            };
            root.Add(Timeline);
            VisualElement frameRule = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    alignItems = Align.FlexStart,
                    alignSelf = Align.Stretch,
                    justifyContent = Justify.FlexStart,
                    height=FrameRuleHeight,
                }
            };
            Timeline.Add(frameRule);
            Button playButton = new Button
            {
                text = "Play",
                style =
                {
                    height = FrameRuleHeight,
                    width = PlayButtonWidth,
                }
            };
            playButton.clicked += () =>
            {
                if (playButton.text == "Play")
                {
                    playButton.text = "Stop";
                    EditorApplication.update += UpdateAnimationForward;
                }
                else
                {
                    playButton.text = "Play";
                    EditorApplication.update -= UpdateAnimationForward;
                }
            };
            frameRule.Add(playButton);
            VisualElement frameRuleLabels = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    flexShrink = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = new Color(0.4f, 0.4f, 0.4f, 1.0f),
                    height = FrameRuleHeight,
                    color = Color.black,
                }
            };
            frameRuleLabels.RegisterCallback<PointerDownEvent>(OnPointerDownFrameRuler, TrickleDown.TrickleDown);
            frameRule.Add(frameRuleLabels);
            // Add reference labels
            const int maxFramesPerRule = 20;
            int framesStep = Mathf.Max(1, NumberFrames / maxFramesPerRule);
            for (int i = 0; i < NumberFrames; i += framesStep)
            {
                VisualElement frameLabelBox = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        flexGrow = 1,
                        flexShrink = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        alignItems = Align.Center,
                        justifyContent = Justify.FlexStart,
                        height = FrameRuleHeight,
                        color = Color.black
                    }
                };
                Label frameLabel = new Label(i.ToString());
                frameLabelBox.Add(frameLabel);
                frameRuleLabels.Add(frameLabelBox);
            }
            // Add current frame indicator
            CurrentFrameIndicator = new VisualElement
            {
                style =
                {
                    alignItems = Align.FlexStart,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = new Color(0.25f, 0.42f, 0.68f, 1.0f),
                    height = FrameRuleHeight,
                    width = 5,
                    position = Position.Absolute,
                }
            };
            frameRuleLabels.Add(CurrentFrameIndicator);
            UpdateCurrentFrameIndicator();
            // Tags
            CreateTagsTimeline(Timeline);
        }

        private void CreateTagsTimeline(VisualElement root)
        {
            // Main Container
            VisualElement tagsContainer = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Column,
                    alignItems = Align.FlexStart,
                    justifyContent = Justify.FlexStart,
                    alignSelf = Align.Stretch,
                    backgroundColor = new Color(0.6f, 0.6f, 0.6f, 1.0f),
                }
            };
            root.Add(tagsContainer);
            // Tags
            for (int i = 0; i < Tags.Count; ++i)
            {
                int tagIndex = i;
                Tag tag = Tags[i];
                VisualElement tagContainer = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        alignSelf = Align.Stretch,
                        alignItems = Align.Center,
                        justifyContent = Justify.FlexStart,
                        backgroundColor = new Color(0.4f, 0.4f, 0.4f, 1.0f),
                    }
                };
                tagsContainer.Add(tagContainer);
                // Tag name
                TextField textField = new TextField
                {
                    value = tag.Name,
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        width = PlayButtonWidth,
                    }
                };
                textField.RegisterValueChangedCallback(x =>
                {
                    tag.Name = x.newValue;
                    Tags[tagIndex] = tag;
                });
                tagContainer.Add(textField);
                // Tag range
                MinMaxSlider range = new MinMaxSlider
                {
                    value = new Vector2(tag.Start, tag.End),
                    lowLimit = 0,
                    highLimit = NumberFrames - 1,
                    style =
                    {
                        flexGrow = 1,
                        flexShrink = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                    }
                };
                range.RegisterValueChangedCallback(x =>
                {
                    tag.Start = (int)x.newValue.x;
                    tag.End = (int)x.newValue.y;
                    Tags[tagIndex] = tag;
                });
                tagContainer.Add(range);
            }
            // New Tag Button
            Button newTagButton = new Button
            {
                text = "New Tag",
                style =
                {
                    height = FrameRuleHeight,
                    width = 200,
                }
            };
            newTagButton.clicked += () =>
            {
                Tag newTag = new Tag();
                Tags.Add(newTag);
                root.Remove(tagsContainer);
                root.Remove(newTagButton);
                CreateTagsTimeline(root);
            };
            root.Add(newTagButton);
        }

        private bool PointerDownOnFrameRuler = false;
        private void OnPointerDownFrameRuler(PointerDownEvent e)
        {
            if (e.button == 0) // left mouse button pressed
            {
                PointerDownOnFrameRuler = true;
                UpdateFrameFromPointer(e.position);
            }
        }
        private void OnPointerUpRoot(PointerUpEvent e)
        {
            if (e.button == 0 && PointerDownOnFrameRuler) // left mouse button released
            {
                PointerDownOnFrameRuler = false;
            }
        }
        private void OnPointerMoveRoot(PointerMoveEvent e)
        {
            if (PointerDownOnFrameRuler) // left mouse button moved
            {
                UpdateFrameFromPointer(e.position);
            }
        }
        private void UpdateFrameFromPointer(Vector2 pointer)
        {
            float containerWidth = CurrentFrameIndicator.parent.resolvedStyle.width;
            float x = CurrentFrameIndicator.parent.WorldToLocal(pointer).x;
            int frame = Mathf.CeilToInt((x / containerWidth) * NumberFrames);
            UpdateCurrentFrame(frame);
            UpdateAnimation(forward: false);
        }

        private void UpdateCurrentFrameIndicator()
        {
            float containerWidth = CurrentFrameIndicator.parent.resolvedStyle.width;
            CurrentFrameIndicator.style.left = CurrentFrame * (containerWidth / NumberFrames);
        }

        private void CreateBVHFields()
        {
            CreateScaleField(rootVisualElement);
            CreateFrameField(rootVisualElement);
        }

        private void ImportBVH()
        {
            RemoveSkeleton();
            BVHImporter bvhImporter = new BVHImporter();
            Animation = bvhImporter.Import(BVHAsset, BVHScale);
            UpdateTargetFramerate(Mathf.CeilToInt(1.0f / Animation.FrameTime));
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

        private void ImportTags()
        {
            Tags = new List<Tag>();
        }

        private void UpdateTargetFramerate(int newValue)
        {
            TargetFramerate = newValue;
            TargetFramerateField.value = TargetFramerate;
        }

        private void UpdateCurrentFrame(int newValue)
        {
            newValue = Mathf.Clamp(newValue, 0, Animation.Frames.Length - 1);
            CurrentFrame = newValue;
            CurrentFrameField.value = CurrentFrame;
            UpdateCurrentFrameIndicator();
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

        private void UpdateAnimationForward() => UpdateAnimation();
        private void UpdateAnimation(bool forward=true)
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
                if (forward) UpdateCurrentFrame((CurrentFrame + 1) % Animation.Frames.Length);
                LastUpdateTime = EditorApplication.timeSinceStartup;
            }
        }

        private void UpdateGUI(SceneView sceneView)
        {
            ReturnButton.RenderSceneGUI(sceneView);
        }

        private void OnDestroy()
        {
            ReturnButton.ReturnScene();
            EditorApplication.update -= UpdateAnimationForward;
            SceneView.duringSceneGui -= UpdateGUI;
        }
    }

    [System.Serializable]
    public struct SceneGUI
    {
        public string PreviousScene;

        public SceneGUI(string previousScene)
        {
            PreviousScene = previousScene;
        }

        public void RenderSceneGUI(SceneView sceneView)
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
            EditorSceneManager.OpenScene(PreviousScene);
        }
    }
}