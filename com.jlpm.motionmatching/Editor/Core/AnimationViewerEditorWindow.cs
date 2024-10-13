using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using UnityEngine.UIElements;
using UnityEditor.UIElements;
using UnityEditor.SceneManagement;
using UnityEngine.SceneManagement;
using System;
using System.Linq;

namespace MotionMatching
{
    using Tag = AnimationData.Tag;

    /* More info on how to create a new scene: https://gist.github.com/ulrikdamm/338392c3b0900de225ec6dd10864cab4 */
    public class AnimationViewerEditorWindow : EditorWindow
    {
        public static AnimationViewerEditorWindow Window;
        private SceneGUI ReturnButton;

        private Transform[] Skeleton;
        private int CurrentFrame = 0;
        private AnimationData AnimationData;
        private int TargetFramerate;
        private double LastUpdateTime;

        private IntegerField CurrentFrameField;
        private IntegerField TargetFramerateField;

        private VisualElement TimelineContainer;
        private VisualElement CurrentFrameIndicator;
        private VisualElement FrameRuleLabels;
        private Button PlayButton;

        private EditorApplication.CallbackFunction UpdatePoseFunction;
        private Action OnUpdatePoseStopped;

        // Ranges
        private int SelectedTag;
        private int SelectedStartRange;
        private int SelectedEndRange;
        private List<VisualElement> RangesContainer;
        private List<List<VisualElement>> TagRangesLines;
        private List<List<VisualElement>> TagRangesStart;
        private List<List<VisualElement>> TagRangesEnd;

        // QueryTag
        private QueryTag CurrentQueryTag;
        private TextField QueryTagTextField;
        private VisualElement QueryTagRangesContainer;
        private List<VisualElement> QueryTagRangesLine;
        private List<VisualElement> QueryTagRangesStart;
        private List<VisualElement> QueryTagRangesEnd;

        // Colors
        private static readonly Color HighlightColor = new Color(0.25f, 0.42f, 0.68f, 1.0f);
        private static readonly Color TagColor = new Color(0.68f, 0.42f, 0.25f, 1.0f);
        private static readonly Color QueryColor = new Color(0.42f, 0.68f, 0.25f, 1.0f);
        private static readonly Color LightGray = new Color(0.4f, 0.4f, 0.4f, 1.0f);
        private static readonly Color DarkGray = new Color(0.3f, 0.3f, 0.3f, 1.0f);

        private int NumberFrames { get { return AnimationData.GetAnimation().Frames.Length; } }

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
            // Init auxiliary variables
            SelectedTag = -1;
            SelectedStartRange = -1;
            SelectedEndRange = -1;

            VisualElement root = rootVisualElement;
            root.RegisterCallback<PointerMoveEvent>(OnPointerMoveRoot, TrickleDown.TrickleDown);
            root.RegisterCallback<PointerUpEvent>(OnPointerUpRoot, TrickleDown.TrickleDown);
            root.parent.RegisterCallback<MouseLeaveEvent>(OnMouseLeaveRoot, TrickleDown.TrickleDown);
            CreateBVHAssetField(root);
        }

        private void OnGUI()
        {
            if (AnimationData == null) return;

            // Check if the spacebar key is pressed
            if (Event.current.type == EventType.KeyDown && Event.current.keyCode == KeyCode.Space)
            {
                PlayButtonClick();
            }
        }

        private void CreateBVHAssetField(VisualElement root)
        {
            ObjectField animationDataField = new ObjectField
            {
                label = "Animation Data",
                objectType = typeof(AnimationData)
            };
            animationDataField.RegisterValueChangedCallback(x =>
            {
                AnimationData = null;
                if (x.newValue != null && x.newValue != x.previousValue)
                {
                    AnimationData = (AnimationData)x.newValue;
                    CreateBVHFields();
                    ImportBVH();
                    CreateTimeline(rootVisualElement);
                    UpdateCurrentFrame(0);
                    UpdatePose(forward: false);
                }
            });
            root.Add(animationDataField);
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
        const float PlayButtonWidth = 70;
        const float TagButtonWidth = 10;
        const float MarginWidth = 5;
        const float FrameIndicatorWidth = 5;
        private void CreateTimeline(VisualElement root)
        {
            if (root.Contains(TimelineContainer))
            {
                root.Remove(TimelineContainer);
            }
            TimelineContainer = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    flexShrink = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    alignItems = Align.FlexStart,
                    justifyContent = Justify.FlexStart,
                }
            };
            root.Add(TimelineContainer);
            // Create two regions, one with the timeline and another with a little bit of padding to the right
            VisualElement rightPaddingTimeline = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Column,
                    backgroundColor = DarkGray,
                    width = 5
                }
            };
            // Create frames rule with numbers
            VisualElement timeline = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Column,
                    flexGrow = 1,
                    flexShrink = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = DarkGray,
                }
            };
            TimelineContainer.Add(timeline);
            TimelineContainer.Add(rightPaddingTimeline);
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
            timeline.Add(frameRule);
            PlayButton = new Button
            {
                text = "Play",
                style =
                {
                    height = FrameRuleHeight,
                    width = PlayButtonWidth,
                    marginLeft = MarginWidth,
                    marginRight = MarginWidth,
                }
            };
            PlayButton.clicked += PlayButtonClick;
            frameRule.Add(PlayButton);
            FrameRuleLabels = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    flexShrink = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = DarkGray,
                    height = FrameRuleHeight,
                    color = Color.black,
                }
            };
            FrameRuleLabels.RegisterCallback<PointerDownEvent>(OnPointerDownFrameRuler, TrickleDown.TrickleDown);
            frameRule.Add(FrameRuleLabels);
            // Add current frame indicator
            CurrentFrameIndicator = new VisualElement
            {
                style =
                {
                    alignItems = Align.FlexStart,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = HighlightColor,
                    height = FrameRuleHeight,
                    width = FrameIndicatorWidth,
                    position = Position.Absolute,
                }
            };
            FrameRuleLabels.Add(CurrentFrameIndicator);
            FrameRuleLabels.RegisterCallback<GeometryChangedEvent>((_) => UpdateCurrentFrameIndicator());
            FrameRuleLabels.RegisterCallback<GeometryChangedEvent>((_) => UpdateFrameLabels());
            // Tags
            CreateTagsTimeline(timeline);
        }

        private void PlayButtonClick()
        {
            if (PlayButton.text == "Play")
            {
                if (OnUpdatePoseStopped != null)
                {
                    OnUpdatePoseStopped();
                    OnUpdatePoseStopped = null;
                }
                PlayButton.text = "Stop";
                UpdatePoseFunction = () => UpdatePose();
                EditorApplication.update += UpdatePoseFunction;
                OnUpdatePoseStopped = () =>
                {
                    PlayButton.text = "Play";
                    EditorApplication.update -= UpdatePoseFunction;
                    UpdatePoseFunction = null;
                };
            }
            else
            {
                OnUpdatePoseStopped();
                OnUpdatePoseStopped = null;
            }
        }

        private void CreateTagsTimeline(VisualElement root)
        {
            // Main Container
            VisualElement tagsContainer = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Column,
                    alignSelf = Align.Stretch,
                    alignItems = Align.FlexStart,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = new Color(0.6f, 0.6f, 0.6f, 1.0f),
                }
            };
            root.Add(tagsContainer);
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
            // Query Tag Container
            VisualElement queryContainer = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    alignSelf = Align.Stretch,
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = LightGray,
                }
            };
            // Tags
            TagRangesLines ??= new List<List<VisualElement>>();
            TagRangesStart ??= new List<List<VisualElement>>();
            TagRangesEnd ??= new List<List<VisualElement>>();
            RangesContainer ??= new List<VisualElement>();
            TagRangesLines.Clear();
            TagRangesStart.Clear();
            TagRangesEnd.Clear();
            RangesContainer.Clear();
            for (int tagIndex = 0; tagIndex < AnimationData.Tags.Count; ++tagIndex)
            {
                Tag tag = AnimationData.Tags[tagIndex];
                TagRangesLines.Add(new List<VisualElement>());
                TagRangesStart.Add(new List<VisualElement>());
                TagRangesEnd.Add(new List<VisualElement>());
                VisualElement tagContainer = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        alignSelf = Align.Stretch,
                        alignItems = Align.Center,
                        justifyContent = Justify.FlexStart,
                        backgroundColor = LightGray,
                    }
                };
                tagsContainer.Add(tagContainer);
                VisualElement leftContainer = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        alignSelf = Align.Stretch,
                        alignItems = Align.Center,
                        justifyContent = Justify.FlexStart,
                        backgroundColor = DarkGray,
                        width = PlayButtonWidth + 2 * MarginWidth,
                        overflow = Overflow.Hidden,
                    }
                };
                tagContainer.Add(leftContainer);
                // Tag name
                TextField textField = new TextField
                {
                    value = tag.Name,
                    isDelayed = true,
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        width = PlayButtonWidth - 2 * TagButtonWidth - 3 * MarginWidth,
                        marginLeft = MarginWidth,
                        marginRight = MarginWidth,
                    }
                };
                int tagIndexCopy = tagIndex;
                textField.RegisterValueChangedCallback(x =>
                {
                    Tag tag = AnimationData.Tags[tagIndexCopy];
                    tag.Name = GetAvailableTagName(x.newValue);
                    textField.SetValueWithoutNotify(tag.Name);
                    AnimationData.Tags[tagIndexCopy] = tag;
                    AnimationData.SaveEditor();
                    QueryTagExpressionChanged(QueryTagTextField.value);
                });
                leftContainer.Add(textField);
                // Tag remove button
                Button tagRemoveButton = new Button()
                {
                    text = "x",
                    style =
                    {
                        height = FrameRuleHeight,
                        width = TagButtonWidth,
                        marginLeft = 0,
                        marginRight = MarginWidth,
                    }
                };
                int tagIndexCopy2 = tagIndex;
                tagRemoveButton.clicked += () =>
                {
                    if (OnUpdatePoseStopped != null)
                    {
                        OnUpdatePoseStopped();
                        OnUpdatePoseStopped = null;
                    }
                    QueryTagExpressionChanged(QueryTagTextField.value);
                    AnimationData.RemoveTag(tagIndexCopy2);
                    root.Remove(tagsContainer);
                    root.Remove(newTagButton);
                    root.Remove(queryContainer);
                    CreateTagsTimeline(root);
                };
                leftContainer.Add(tagRemoveButton);
                // Tag play button
                Button tagPlayButton = new Button()
                {
                    text = "p",
                    style =
                    {
                        height = FrameRuleHeight,
                        width = TagButtonWidth,
                        marginLeft = 0,
                        marginRight = MarginWidth,
                    }
                };
                int tagIndexCopy3 = tagIndex;
                tagPlayButton.clicked += () =>
                {
                    if (tagPlayButton.text == "p")
                    {
                        if (OnUpdatePoseStopped != null)
                        {
                            OnUpdatePoseStopped();
                            OnUpdatePoseStopped = null;
                        }
                        tagPlayButton.text = "s";
                        UpdatePoseFunction = () => UpdatePose(true, tagIndexCopy3);
                        EditorApplication.update += UpdatePoseFunction;
                        OnUpdatePoseStopped = () =>
                        {
                            tagPlayButton.text = "p";
                            EditorApplication.update -= UpdatePoseFunction;
                            UpdatePoseFunction = null;
                        };
                    }
                    else
                    {
                        OnUpdatePoseStopped();
                        OnUpdatePoseStopped = null;
                    }
                };
                leftContainer.Add(tagPlayButton);
                // Tag ranges container
                VisualElement rangesContainer = new VisualElement
                {
                    style =
                    {
                        flexDirection = FlexDirection.Row,
                        flexGrow = 1,
                        alignSelf = Align.Stretch,
                        alignItems = Align.Center,
                    }
                };
                tagContainer.Add(rangesContainer);
                RangesContainer.Add(rangesContainer);
                rangesContainer.RegisterCallback<PointerDownEvent>((e) => OnPointerDownRangesContainer(e, tagIndexCopy, rangesContainer));
                // Ranges
                CreateRangesVisual(tagIndex);
                // Update Ranges Container
                rangesContainer.RegisterCallback<GeometryChangedEvent>((_) => UpdateRangesContainer(tagIndexCopy));
            }
            newTagButton.clicked += () =>
            {
                AnimationData.AddTag(GetAvailableTagName(""));
                root.Remove(tagsContainer);
                root.Remove(newTagButton);
                root.Remove(queryContainer);
                CreateTagsTimeline(root);
                QueryTagExpressionChanged(QueryTagTextField.value);
            };
            root.Add(newTagButton);
            root.Add(queryContainer);
            // Query tag
            VisualElement leftQueryContainer = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    alignSelf = Align.Stretch,
                    alignItems = Align.Center,
                    justifyContent = Justify.FlexStart,
                    backgroundColor = DarkGray,
                    width = PlayButtonWidth + 2 * MarginWidth,
                    overflow = Overflow.Hidden,
                }
            };
            queryContainer.Add(leftQueryContainer);
            // Query name
            QueryTagTextField = new TextField
            {
                value = QueryTagTextField != null ? QueryTagTextField.value : "((tag1 & tag2) | tag3) - tag4",
                style =
                {
                    alignItems = Align.FlexStart,
                    justifyContent = Justify.FlexStart,
                    width = PlayButtonWidth - TagButtonWidth - MarginWidth,
                    marginLeft = MarginWidth,
                    marginRight = MarginWidth,
                }
            };
            QueryTagTextField.RegisterValueChangedCallback(x => QueryTagExpressionChanged(x.newValue));
            leftQueryContainer.Add(QueryTagTextField);
            // Tag play button
            Button queryTagPlayButton = new Button()
            {
                text = "p",
                style =
                {
                    height = FrameRuleHeight,
                    width = TagButtonWidth,
                    marginLeft = 0,
                    marginRight = MarginWidth,
                }
            };
            queryTagPlayButton.clicked += () =>
            {
                if (queryTagPlayButton.text == "p")
                {
                    if (OnUpdatePoseStopped != null)
                    {
                        OnUpdatePoseStopped();
                        OnUpdatePoseStopped = null;
                    }
                    queryTagPlayButton.text = "s";
                    UpdatePoseFunction = () =>
                    {
                        if (CurrentQueryTag != null) UpdatePose(true, queryTag: true);
                        else
                        {
                            OnUpdatePoseStopped();
                            OnUpdatePoseStopped = null;
                        }
                    };
                    EditorApplication.update += UpdatePoseFunction;
                    OnUpdatePoseStopped = () =>
                    {
                        queryTagPlayButton.text = "p";
                        EditorApplication.update -= UpdatePoseFunction;
                        UpdatePoseFunction = null;
                    };
                }
                else
                {
                    OnUpdatePoseStopped();
                    OnUpdatePoseStopped = null;
                }
            };
            leftQueryContainer.Add(queryTagPlayButton);
            QueryTagRangesContainer = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    alignSelf = Align.Stretch,
                    alignItems = Align.Center,
                }
            };
            queryContainer.Add(QueryTagRangesContainer);
            // Update Ranges Container
            QueryTagRangesContainer.RegisterCallback<GeometryChangedEvent>((_) => UpdateQueryTagRangesContainer());
        }

        const int RangeHandleHeight = 12;
        const int RangeHandleWidth = 6;
        private void CreateRangesVisual(int tagIndex)
        {
            VisualElement rangesContainer = RangesContainer[tagIndex];
            rangesContainer.Clear();

            QueryTagRangesLine ??= new List<VisualElement>();
            QueryTagRangesStart ??= new List<VisualElement>();
            QueryTagRangesEnd ??= new List<VisualElement>();

            QueryTagRangesLine.Clear();
            QueryTagRangesStart.Clear();
            QueryTagRangesEnd.Clear();

            TagRangesLines[tagIndex].Clear();
            TagRangesStart[tagIndex].Clear();
            TagRangesEnd[tagIndex].Clear();

            // Tag auxiliary line
            VisualElement rangeAuxLine = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    backgroundColor = new Color(0.5f, 0.5f, 0.5f, 1.0f),
                    height = 2,
                }
            };
            rangesContainer.Add(rangeAuxLine);

            Tag tag = AnimationData.Tags[tagIndex];
            int tagSize = tag.Start == null ? 0 : tag.Start.Length;
            for (int rangeIndex = 0; rangeIndex < tagSize; ++rangeIndex)
            {
                // Range line
                VisualElement rangeLine = new VisualElement
                {
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        flexGrow = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        backgroundColor = TagColor,
                        height = 4,
                        position = Position.Absolute,
                    }
                };
                rangesContainer.Add(rangeLine);
                TagRangesLines[tagIndex].Add(rangeLine);
                // Range start button
                VisualElement rangeStart = new VisualElement
                {
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        flexGrow = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        backgroundColor = TagColor,
                        height = RangeHandleHeight,
                        position = Position.Absolute,
                    }
                };
                rangesContainer.Add(rangeStart);
                TagRangesStart[tagIndex].Add(rangeStart);
                int tagIndexCopy = tagIndex;
                int rangeIndexCopy = rangeIndex;
                rangeStart.RegisterCallback<PointerDownEvent>((e) => OnPointerDownStartRange(e, tagIndexCopy, rangeIndexCopy));
                // Range end button
                VisualElement rangeEnd = new VisualElement
                {
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        flexGrow = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        backgroundColor = TagColor,
                        height = RangeHandleHeight,
                        position = Position.Absolute,
                    }
                };
                rangesContainer.Add(rangeEnd);
                TagRangesEnd[tagIndex].Add(rangeEnd);
                rangeEnd.RegisterCallback<PointerDownEvent>((e) => OnPointerDownEndRange(e, tagIndexCopy, rangeIndexCopy));
            }
        }

        private void CreateQueryRangesVisual()
        {
            QueryTagRangesContainer.Clear();

            // Auxiliary line
            VisualElement rangeAuxLine = new VisualElement
            {
                style =
                {
                    flexDirection = FlexDirection.Row,
                    flexGrow = 1,
                    flexBasis = new StyleLength(StyleKeyword.Auto),
                    backgroundColor = new Color(0.5f, 0.5f, 0.5f, 1.0f),
                    height = 2,
                }
            };
            QueryTagRangesContainer.Add(rangeAuxLine);

            QueryTagRangesLine.Clear();
            QueryTagRangesStart.Clear();
            QueryTagRangesEnd.Clear();

            if (CurrentQueryTag == null) return;

            for (int rangeIndex = 0; rangeIndex < CurrentQueryTag.GetStartRanges().Length; ++rangeIndex)
            {
                // Range line
                VisualElement rangeLine = new VisualElement
                {
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        flexGrow = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        backgroundColor = QueryColor,
                        height = 4,
                        position = Position.Absolute,
                    }
                };
                QueryTagRangesContainer.Add(rangeLine);
                QueryTagRangesLine.Add(rangeLine);
                // Range start button
                VisualElement rangeStart = new VisualElement
                {
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        flexGrow = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        backgroundColor = QueryColor,
                        height = RangeHandleHeight,
                        position = Position.Absolute,
                    }
                };
                QueryTagRangesContainer.Add(rangeStart);
                QueryTagRangesStart.Add(rangeStart);
                // Range end button
                VisualElement rangeEnd = new VisualElement
                {
                    style =
                    {
                        alignItems = Align.FlexStart,
                        justifyContent = Justify.FlexStart,
                        flexGrow = 1,
                        flexBasis = new StyleLength(StyleKeyword.Auto),
                        backgroundColor = QueryColor,
                        height = RangeHandleHeight,
                        position = Position.Absolute,
                    }
                };
                QueryTagRangesContainer.Add(rangeEnd);
                QueryTagRangesEnd.Add(rangeEnd);
            }

            UpdateQueryTagRangesContainer();
        }

        private void UpdateRangesContainer(int tagIndex)
        {
            for (int rangeIndex = 0; rangeIndex < TagRangesLines[tagIndex].Count; ++rangeIndex)
            {
                Tag tag = AnimationData.Tags[tagIndex];
                int startFrame = tag.Start[rangeIndex];
                int endFrame = tag.End[rangeIndex];
                VisualElement rangeLine = TagRangesLines[tagIndex][rangeIndex];
                float rangesContainerWidth = rangeLine.parent.resolvedStyle.width;
                float left = rangesContainerWidth * ((float)startFrame / NumberFrames);
                float right = rangesContainerWidth - left;
                float leftEnd = rangesContainerWidth * ((float)endFrame / NumberFrames);
                float rightEnd = rangesContainerWidth - leftEnd;
                rangeLine.style.left = left;
                rangeLine.style.right = rightEnd;
                VisualElement rangeStart = TagRangesStart[tagIndex][rangeIndex];
                rangeStart.style.left = left;
                rangeStart.style.right = right - RangeHandleWidth + Mathf.Max(0, RangeHandleWidth - (leftEnd - left) - 1);
                VisualElement rangeEnd = TagRangesEnd[tagIndex][rangeIndex];
                rangeEnd.style.left = leftEnd - RangeHandleWidth + Mathf.Max(0, RangeHandleWidth - (leftEnd - left) - 1);
                rangeEnd.style.right = rightEnd;
            }

            if (CurrentQueryTag != null) CurrentQueryTag.ComputeRanges(null, AnimationData.Tags, force: true);

            CreateQueryRangesVisual();
        }

        private void UpdateQueryTagRangesContainer()
        {
            if (QueryTagRangesLine == null) return;

            for (int rangeIndex = 0; rangeIndex < QueryTagRangesLine.Count; ++rangeIndex)
            {
                int startFrame = CurrentQueryTag.GetStartRanges()[rangeIndex];
                int endFrame = CurrentQueryTag.GetEndRanges()[rangeIndex];
                VisualElement rangeLine = QueryTagRangesLine[rangeIndex];
                float rangesContainerWidth = rangeLine.parent.resolvedStyle.width;
                float left = rangesContainerWidth * ((float)startFrame / NumberFrames);
                float right = rangesContainerWidth - left;
                float leftEnd = rangesContainerWidth * ((float)endFrame / NumberFrames);
                float rightEnd = rangesContainerWidth - leftEnd;
                rangeLine.style.left = left;
                rangeLine.style.right = rightEnd;
                VisualElement rangeStart = QueryTagRangesStart[rangeIndex];
                rangeStart.style.left = left;
                rangeStart.style.right = right - RangeHandleWidth + Mathf.Max(0, RangeHandleWidth - (leftEnd - left) - 1);
                VisualElement rangeEnd = QueryTagRangesEnd[rangeIndex];
                rangeEnd.style.left = leftEnd - RangeHandleWidth + Mathf.Max(0, RangeHandleWidth - (leftEnd - left) - 1);
                rangeEnd.style.right = rightEnd;
            }
        }

        private void QueryTagExpressionChanged(string newExpression)
        {
            if (CurrentQueryTag != null) CurrentQueryTag.Dispose();
            CurrentQueryTag = null;
            bool res = QueryTag.Parse(newExpression, out QueryTag queryTag);
            if (res)
            {
                // Check all Tags exist
                foreach (string tagName in queryTag.GetTags())
                {
                    bool found = false;
                    foreach (Tag tag in AnimationData.Tags)
                    {
                        if (tag.Name == tagName)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                    {
                        CreateQueryRangesVisual();
                        return;
                    }
                }
                // Process query tag
                queryTag.ComputeRanges(null, AnimationData.Tags, force: true);
                CurrentQueryTag = queryTag;
            }
            CreateQueryRangesVisual();
        }

        private void OnPointerDownRangesContainer(PointerDownEvent e, int tagIndex, VisualElement element)
        {
            Tag tag = AnimationData.Tags[tagIndex];
            int GetRangeIndex(int tagIndex, int startFrame, int endFrame)
            {
                startFrame = Mathf.Max(0, startFrame);
                endFrame = Mathf.Min(endFrame, NumberFrames - 1);
                if (tag.Start != null)
                {
                    for (int rangeIndex = 0; rangeIndex < tag.Start.Length; ++rangeIndex)
                    {
                        // Check if the current range intersects with the startFrame and endFrame
                        bool isIntersect = !(tag.Start[rangeIndex] > endFrame || tag.End[rangeIndex] < startFrame);

                        if (isIntersect) return rangeIndex;
                    }
                }
                return -1;
            }

            int frameRadius = Mathf.FloorToInt(NumberFrames * 0.005f); // 0.5 %

            if (e.clickCount >= 2)
            {
                int frame = GetFrameFromPointer(e.position, RangesContainer[tagIndex]);
                int rangeIndex = GetRangeIndex(tagIndex, frame - frameRadius * 2, frame + frameRadius);
                if (rangeIndex == -1) CreateRange(tagIndex, frame);
            }
            else if (e.button == (int)MouseButton.RightMouse)
            {
                var dropdownMenu = new GenericDropdownMenu();

                int frame = GetFrameFromPointer(e.position, RangesContainer[tagIndex]);
                int rangeIndex = GetRangeIndex(tagIndex, frame - frameRadius * 2, frame + frameRadius);

                if (rangeIndex == -1)
                {
                    // Add range
                    dropdownMenu.AddItem("Add range", false, () => CreateRange(tagIndex, frame));
                }
                else
                {
                    // Show Start and End
                    dropdownMenu.AddDisabledItem("[" + tag.Start[rangeIndex] + ", " + tag.End[rangeIndex] + "]", false);
                    dropdownMenu.AddSeparator("");
                    // Remove range
                    dropdownMenu.AddItem("Remove range", false, () => RemoveRange(tagIndex, rangeIndex));
                }

                // Show the context menu
                dropdownMenu.DropDown(new Rect(e.position, Vector2.zero), element);
            }
        }

        private void OnPointerDownStartRange(PointerDownEvent e, int tagIndex, int rangeStartIndex)
        {
            if (e.button == 0) // left mouse button pressed
            {
                SelectedTag = tagIndex;
                SelectedStartRange = rangeStartIndex;
            }
        }

        private void OnPointerDownEndRange(PointerDownEvent e, int tagIndex, int rangeEndIndex)
        {
            if (e.button == 0) // left mouse button pressed
            {
                SelectedTag = tagIndex;
                SelectedEndRange = rangeEndIndex;
            }
        }

        private bool PointerDownOnFrameRuler = false;
        private void OnPointerDownFrameRuler(PointerDownEvent e)
        {
            if (e.button == 0) // left mouse button pressed
            {
                if (OnUpdatePoseStopped != null)
                {
                    OnUpdatePoseStopped();
                    OnUpdatePoseStopped = null;
                }

                PointerDownOnFrameRuler = true;
                UpdateFrameFromPointer(e.position);
            }
        }
        private void OnPointerUpRoot(PointerUpEvent e)
        {
            if (e.button == 0) // left mouse button released
            {
                if (PointerDownOnFrameRuler)
                {
                    PointerDownOnFrameRuler = false;
                }
                if (SelectedTag != -1)
                {
                    SelectedTag = -1;
                }
                if (SelectedStartRange != -1)
                {
                    SelectedStartRange = -1;
                }
                if (SelectedEndRange != -1)
                {
                    SelectedEndRange = -1;
                }
            }
        }
        private void OnMouseLeaveRoot(MouseLeaveEvent e)
        {
            if (e.target != rootVisualElement.parent) return;

            if (PointerDownOnFrameRuler)
            {
                PointerDownOnFrameRuler = false;
            }
            if (SelectedTag != -1)
            {
                SelectedTag = -1;
            }
            if (SelectedStartRange != -1)
            {
                SelectedStartRange = -1;
            }
            if (SelectedEndRange != -1)
            {
                SelectedEndRange = -1;
            }
        }
        private void OnPointerMoveRoot(PointerMoveEvent e)
        {
            if (PointerDownOnFrameRuler) // left mouse button moved
            {
                UpdateFrameFromPointer(e.position);
            }
            if (SelectedTag != -1 && SelectedStartRange != -1)
            {
                UpdateFrameFromPointer(e.position);
                UpdateStartTagFromPointer(e.position);
                AnimationData.SaveEditor();
            }
            if (SelectedTag != -1 && SelectedEndRange != -1)
            {
                UpdateFrameFromPointer(e.position);
                UpdateEndTagFromPointer(e.position);
                AnimationData.SaveEditor();
            }
        }
        private void UpdateFrameFromPointer(Vector2 pointer)
        {
            int frame = GetFrameFromPointer(pointer, CurrentFrameIndicator.parent);
            UpdateCurrentFrame(frame);
            UpdatePose(forward: false);
        }

        private void CreateRange(int tagIndex, int frame)
        {
            int radius = Mathf.FloorToInt(NumberFrames * 0.02f);

            // Assume at least one frame is available (frame)
            // Find maximum radius right and maximum radius left
            int leftRadius = Mathf.Max(0, frame - radius);
            int rightRadius = Mathf.Min(NumberFrames - 1, frame + radius);

            Tag tag = AnimationData.Tags[tagIndex];
            if (tag.Start == null)
            {
                // If no ranges yet just initialize the first one
                tag.Start = new int[1] { leftRadius };
                tag.End = new int[1] { rightRadius };
            }
            else
            {
                // Otherwise add a new range and find it's sorted position in the ranges array
                Array.Resize<int>(ref tag.Start, tag.Start.Length + 1);
                Array.Resize<int>(ref tag.End, tag.End.Length + 1);
                int rangeIndex = tag.Start.Length - 2;
                for (; rangeIndex >= 0; --rangeIndex)
                {
                    if (tag.Start[rangeIndex] > frame)
                    {
                        tag.Start[rangeIndex + 1] = tag.Start[rangeIndex];
                        tag.End[rangeIndex + 1] = tag.End[rangeIndex];
                    }
                    else
                    {
                        rangeIndex += 1;
                        break;
                    }
                }
                rangeIndex = rangeIndex < 0 ? 0 : rangeIndex;
                // Expand tag according to left and right radius and without overlapping with neighbor ranges
                if (rangeIndex > 0)
                {
                    leftRadius = Mathf.Max(leftRadius, tag.End[rangeIndex - 1] + 1);
                }
                if (rangeIndex < tag.Start.Length - 1)
                {
                    rightRadius = Mathf.Min(rightRadius, tag.Start[rangeIndex + 1] - 1);
                }
                tag.Start[rangeIndex] = leftRadius;
                tag.End[rangeIndex] = rightRadius;
            }
            AnimationData.Tags[tagIndex] = tag;
            AnimationData.SaveEditor();

            CreateRangesVisual(tagIndex);
            UpdateRangesContainer(tagIndex);
        }

        private void RemoveRange(int tagIndex, int rangeIndex)
        {
            Tag tag = AnimationData.Tags[tagIndex];
            if (tag.Start != null)
            {
                // Create new arrays with a size reduced by 1
                int[] newStart = new int[tag.Start.Length - 1];
                int[] newEnd = new int[tag.End.Length - 1];

                // Copy the range data over to the new arrays, skipping the range to be removed
                for (int i = 0, j = 0; i < tag.Start.Length; ++i)
                {
                    if (i != rangeIndex)
                    {
                        newStart[j] = tag.Start[i];
                        newEnd[j] = tag.End[i];
                        ++j;
                    }
                }

                // Replace the old range arrays with the new ones
                tag.Start = newStart;
                tag.End = newEnd;

                AnimationData.Tags[tagIndex] = tag;
                AnimationData.SaveEditor();

                CreateRangesVisual(tagIndex);
                UpdateRangesContainer(tagIndex);
            }
        }

        private void UpdateStartTagFromPointer(Vector2 pointer)
        {
            Tag tag = AnimationData.Tags[SelectedTag];

            int max = tag.End[SelectedStartRange] - 1;
            int min = 0;
            for (int rangeIndex = 0; rangeIndex < tag.End.Length; ++rangeIndex)
            {
                if (rangeIndex == SelectedStartRange) continue;

                if (tag.End[rangeIndex] < tag.Start[SelectedStartRange])
                {
                    min = Math.Max(min, tag.End[rangeIndex] + 1);
                }
            }

            VisualElement startRange = TagRangesStart[SelectedTag][SelectedStartRange];
            int frame = GetFrameFromPointer(pointer, startRange.parent);
            tag.Start[SelectedStartRange] = Mathf.Min(Mathf.Max(frame, min), max);

            AnimationData.Tags[SelectedTag] = tag;

            UpdateRangesContainer(SelectedTag);
        }

        private void UpdateEndTagFromPointer(Vector2 pointer)
        {
            Tag tag = AnimationData.Tags[SelectedTag];

            int max = NumberFrames - 1;
            int min = tag.Start[SelectedEndRange] + 1;
            for (int rangeIndex = 0; rangeIndex < tag.Start.Length; ++rangeIndex)
            {
                if (rangeIndex == SelectedEndRange) continue;

                if (tag.Start[rangeIndex] > tag.End[SelectedEndRange])
                {
                    max = Math.Min(max, tag.Start[rangeIndex] - 1);
                }
            }

            VisualElement endRange = TagRangesEnd[SelectedTag][SelectedEndRange];
            int frame = GetFrameFromPointer(pointer, endRange.parent);
            tag.End[SelectedEndRange] = Mathf.Min(Mathf.Max(frame, min), max);

            AnimationData.Tags[SelectedTag] = tag;

            UpdateRangesContainer(SelectedTag);
        }

        private void UpdateCurrentFrameIndicator()
        {
            float containerWidth = CurrentFrameIndicator.parent.resolvedStyle.width;
            CurrentFrameIndicator.style.left = containerWidth * ((float)CurrentFrame / NumberFrames) - FrameIndicatorWidth / 2;
        }

        private static readonly int[] CandidatesFramesStep = { 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000, 200000, 500000, 1000000 };
        private void UpdateFrameLabels()
        {
            // Remove all children but the FrameIndicator
            for (int i = FrameRuleLabels.childCount - 1; i >= 1; i--) FrameRuleLabels.RemoveAt(i);

            int maxLabels = Mathf.Max(1, (int)(FrameRuleLabels.resolvedStyle.width / 50.0f));

            int framesStep = 1;
            int framesStepIndex = 0;
            while (NumberFrames / framesStep > maxLabels)
            {
                framesStep = CandidatesFramesStep[framesStepIndex++];
            }
            for (int i = 0; i < NumberFrames - framesStep; i += framesStep)
            {
                VisualElement frameLabelBox = new VisualElement
                {
                    style =
                    {
                        alignItems = Align.Center,
                        justifyContent = Justify.Center,
                        height = FrameRuleHeight,
                        color = Color.black,
                        position = Position.Absolute,
                    }
                };
                TextElement frameLabel = new TextElement
                {
                    text = i.ToString(),
                    style =
                    {
                        unityTextAlign = TextAnchor.MiddleCenter,
                    }
                };
                frameLabelBox.Add(frameLabel);
                FrameRuleLabels.Add(frameLabelBox);

                // Use a local variable to capture the loop variable
                int index = i;

                frameLabelBox.RegisterCallback<GeometryChangedEvent>((evt) =>
                {
                    float newLeft = FrameRuleLabels.resolvedStyle.width * ((float)index / NumberFrames) - frameLabel.resolvedStyle.width / 2;
                    if (Mathf.Abs(frameLabelBox.style.left.value.value - newLeft) > 0.01f)
                    {
                        frameLabelBox.style.left = newLeft;
                    }
                });
            }
        }

        private void UpdateTagRangeFrameIndicator()
        {
            for (int tagIndex = 0; tagIndex < AnimationData.Tags.Count; ++tagIndex)
            {
                Tag tag = AnimationData.Tags[tagIndex];
                if (tag.Start != null)
                {
                    for (int rangeIndex = 0; rangeIndex < tag.Start.Length; ++rangeIndex)
                    {
                        TagRangesLines[tagIndex][rangeIndex].style.backgroundColor = TagColor;
                        TagRangesStart[tagIndex][rangeIndex].style.backgroundColor = TagColor;
                        TagRangesEnd[tagIndex][rangeIndex].style.backgroundColor = TagColor;
                        if (CurrentFrame >= tag.Start[rangeIndex] && CurrentFrame <= tag.End[rangeIndex])
                        {
                            TagRangesLines[tagIndex][rangeIndex].style.backgroundColor = HighlightColor;
                            TagRangesStart[tagIndex][rangeIndex].style.backgroundColor = HighlightColor;
                            TagRangesEnd[tagIndex][rangeIndex].style.backgroundColor = HighlightColor;
                        }
                    }
                }
            }
            if (CurrentQueryTag != null)
            {
                for (int rangeIndex = 0; rangeIndex < CurrentQueryTag.GetStartRanges().Length; ++rangeIndex)
                {
                    QueryTagRangesLine[rangeIndex].style.backgroundColor = QueryColor;
                    QueryTagRangesStart[rangeIndex].style.backgroundColor = QueryColor;
                    QueryTagRangesEnd[rangeIndex].style.backgroundColor = QueryColor;
                    if (CurrentFrame >= CurrentQueryTag.GetStartRanges()[rangeIndex] && CurrentFrame <= CurrentQueryTag.GetEndRanges()[rangeIndex])
                    {
                        QueryTagRangesLine[rangeIndex].style.backgroundColor = HighlightColor;
                        QueryTagRangesStart[rangeIndex].style.backgroundColor = HighlightColor;
                        QueryTagRangesEnd[rangeIndex].style.backgroundColor = HighlightColor;
                    }
                }
            }
        }

        private string GetAvailableTagName(string tagName)
        {
            if (tagName == "")
            {
                tagName = "NewTag";
            }

            string initialName = tagName;

            bool available = false;

            while (!available)
            {
                available = true;
                foreach (var tag in AnimationData.Tags)
                {
                    if (tag.Name == tagName)
                    {
                        int num = 1;
                        string[] splitName = tag.Name.Split(".");
                        if (splitName.Length > 1 && int.TryParse(splitName[splitName.Length - 1], out int candidateNum))
                        {
                            num = candidateNum + 1;
                        }
                        tagName = initialName + "." + num.ToString();
                        available = false;
                        break;
                    }
                }
            }

            return tagName;
        }

        private int GetFrameFromPointer(Vector2 pointer, VisualElement container)
        {
            float x = container.WorldToLocal(pointer).x;
            int frame = Mathf.RoundToInt((x / container.resolvedStyle.width) * NumberFrames);
            return frame;
        }

        private void CreateBVHFields()
        {
            CreateFrameField(rootVisualElement);
        }

        private void ImportBVH()
        {
            RemoveSkeleton();
            BVHAnimation animation = AnimationData.GetAnimation();
            UpdateTargetFramerate(Mathf.RoundToInt(1.0f / animation.FrameTime));
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
            newValue = Mathf.Clamp(newValue, 0, AnimationData.GetAnimation().Frames.Length - 1);
            CurrentFrame = newValue;
            CurrentFrameField.value = CurrentFrame;
            UpdateCurrentFrameIndicator();
            UpdateTagRangeFrameIndicator();
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

        private void UpdatePose(bool forward = true, int tagIndex = -1, bool queryTag = false)
        {
            BVHAnimation animation = AnimationData.GetAnimation();
            if (animation != null && LastUpdateTime + (1.0 / TargetFramerate) < EditorApplication.timeSinceStartup)
            {
                // Update skeleton
                BVHAnimation.Frame frame = animation.Frames[CurrentFrame];
                Skeleton[0].position = frame.RootMotion;
                for (int i = 0; i < Skeleton.Length; i++)
                {
                    Skeleton[i].localRotation = frame.LocalRotations[i];
                }
                // Update frame index
                if (forward)
                {
                    if (tagIndex == -1 && !queryTag)
                    {
                        UpdateCurrentFrame((CurrentFrame + 1) % animation.Frames.Length);
                    }
                    else if (!queryTag)
                    {
                        Tag tag = AnimationData.Tags[tagIndex];
                        bool found = false;
                        for (int rangeIndex = 0; rangeIndex < tag.Start.Length && !found; ++rangeIndex)
                        {
                            if (CurrentFrame <= tag.End[rangeIndex])
                            {
                                if (CurrentFrame < tag.Start[rangeIndex])
                                {
                                    UpdateCurrentFrame(tag.Start[rangeIndex]);
                                }
                                else
                                {
                                    UpdateCurrentFrame(CurrentFrame + 1);
                                }
                                found = true;
                            }
                        }
                        if (!found)
                        {
                            UpdateCurrentFrame(tag.Start[0]);
                        }
                    }
                    else
                    {
                        bool found = false;
                        for (int rangeIndex = 0; rangeIndex < CurrentQueryTag.GetStartRanges().Length && !found; ++rangeIndex)
                        {
                            if (CurrentFrame <= CurrentQueryTag.GetEndRanges()[rangeIndex])
                            {
                                if (CurrentFrame < CurrentQueryTag.GetStartRanges()[rangeIndex])
                                {
                                    UpdateCurrentFrame(CurrentQueryTag.GetStartRanges()[rangeIndex]);
                                }
                                else
                                {
                                    UpdateCurrentFrame(CurrentFrame + 1);
                                }
                                found = true;
                            }
                        }
                        if (!found)
                        {
                            UpdateCurrentFrame(CurrentQueryTag.GetStartRanges()[0]);
                        }
                    }
                }
                LastUpdateTime = EditorApplication.timeSinceStartup;
            }
        }

        private void UpdateGUI(SceneView sceneView)
        {
            ReturnButton.RenderSceneGUI(sceneView);
        }

        private void OnDestroy()
        {
            if (CurrentQueryTag != null) CurrentQueryTag.Dispose();
            ReturnButton.ReturnScene();
            EditorApplication.update -= UpdatePoseFunction;
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