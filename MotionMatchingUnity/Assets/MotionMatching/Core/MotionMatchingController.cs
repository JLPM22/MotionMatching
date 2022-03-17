using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;
    using PoseFeature = MotionMatchingData.PoseFeature;

    // Simulation bone is the transform
    public class MotionMatchingController : MonoBehaviour
    {
        public event Action OnSkeletonTransformUpdated;

        public MotionMatchingCharacterController CharacterController;
        public MotionMatchingData MMData;
        public bool LockFPS = true;
        public int SearchFrames = 10; // Motion Matching every SearchFrames frames
        public bool Inertialize = true; // Should inertialize transitions after a big change of the pose
        [Range(0.0f, 1.0f)] public float InertializeHalfLife = 0.1f; // Time needed to move half of the distance between the source to the target pose
        [Tooltip("How important is the trajectory (future positions + future directions)")][Range(0.0f, 1.0f)] public float Responsiveness = 1.0f;
        [Tooltip("How important is the current pose")][Range(0.0f, 1.0f)] public float Quality = 1.0f;
        [HideInInspector] public float[] FeatureWeights;
        [Header("Debug")]
        public float SpheresRadius = 0.1f;
        public bool DebugSkeleton = true;
        public bool DebugCurrent = true;
        public bool DebugPose = true;
        public bool DebugTrajectory = true;

        public float3 Velocity { get; private set; }

        private PoseSet PoseSet;
        private FeatureSet FeatureSet;
        private Transform[] SkeletonTransforms;
        private int CurrentFrame;
        private int SearchFrameCount;
        private NativeArray<float> QueryFeature;
        private NativeArray<int> SearchResult;
        private NativeArray<float> FeaturesWeightsNativeArray;
        private Inertialization Inertialization;

        private void Awake()
        {
            // PoseSet
            PoseSet = MMData.GetOrImportPoseSet();

            // FeatureSet
            FeatureSet = MMData.GetOrImportFeatureSet();

            // Skeleton
            SkeletonTransforms = new Transform[PoseSet.Skeleton.Joints.Count];
            foreach (Skeleton.Joint joint in PoseSet.Skeleton.Joints)
            {
                Transform t = (new GameObject()).transform;
                t.name = joint.Name;
                if (joint.Index == 0) t.SetParent(transform, false);
                else t.SetParent(SkeletonTransforms[joint.ParentIndex], false);
                t.localPosition = joint.LocalOffset;
                SkeletonTransforms[joint.Index] = t;
            }

            // Inertialization
            Inertialization = new Inertialization(PoseSet.Skeleton);

            // FPS
            if (LockFPS)
            {
                Application.targetFrameRate = (int)(1.0f / PoseSet.FrameTime);
                Debug.Log("[Motion Matching] Updated Target FPS: " + Application.targetFrameRate);
            }
            else
            {
                Application.targetFrameRate = -1;
            }

            // Other initialization
            SearchResult = new NativeArray<int>(1, Allocator.Persistent);
            int numberFeatures = (MMData.TrajectoryFeatures.Count + MMData.PoseFeatures.Count);
            if (FeatureWeights == null || FeatureWeights.Length != numberFeatures)
            {
                float[] newWeights = new float[numberFeatures];
                for (int i = 0; i < newWeights.Length; ++i) newWeights[i] = 1.0f;
                for (int i = 0; i < Mathf.Min(FeatureWeights.Length, newWeights.Length); i++) newWeights[i] = FeatureWeights[i];
                FeatureWeights = newWeights;
            }
            FeaturesWeightsNativeArray = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);
            QueryFeature = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);
            // Search first Frame valid (to start with a valid pose)
            for (int i = 0; i < FeatureSet.NumberFeatureVectors; i++)
            {
                if (FeatureSet.IsValidFeature(i))
                {
                    CurrentFrame = i;
                    break;
                }
            }
            // Init Pose
            transform.position = CharacterController.GetWorldInitPosition();
            transform.rotation = quaternion.LookRotation(CharacterController.GetWorldInitDirection(), Vector3.up);
        }

        private void OnEnable()
        {
            SearchFrameCount = 0;
            CharacterController.OnUpdated += OnCharacterControllerUpdated;
            CharacterController.OnInputChangedQuickly += OnInputChangedQuickly;
        }

        private void OnDisable()
        {
            CharacterController.OnUpdated -= OnCharacterControllerUpdated;
            CharacterController.OnInputChangedQuickly -= OnInputChangedQuickly;
        }

        private void OnCharacterControllerUpdated(float deltaTime)
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Total");
            if (SearchFrameCount == 0)
            {
                // Motion Matching
                PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Search");
                int bestFrame = SearchMotionMatching();
                PROFILE.END_SAMPLE_PROFILING("Motion Matching Search");
                if (bestFrame != CurrentFrame)
                {
                    // Inertialize
                    if (Inertialize)
                    {
                        Inertialization.PoseTransition(PoseSet, CurrentFrame, bestFrame);
                    }
                    CurrentFrame = bestFrame;
                }
                SearchFrameCount = SearchFrames;
            }
            else
            {
                // Advance
                SearchFrameCount -= 1;
            }
            // Always advance one (bestFrame from motion matching is the best match to the current frame, but we want to move to the next frame)
            CurrentFrame += 1;

            UpdateTransformAndSkeleton(CurrentFrame);
            PROFILE.END_SAMPLE_PROFILING("Motion Matching Total");
        }

        private void OnInputChangedQuickly()
        {
            SearchFrameCount = 0; // Force search
        }

        private int SearchMotionMatching()
        {
            // Weights
            int offset = 0;
            for (int i = 0; i < MMData.TrajectoryFeatures.Count; i++)
            {
                TrajectoryFeature feature = MMData.TrajectoryFeatures[i];
                float weight = FeatureWeights[i] * Responsiveness;
                for (int p = 0; p < feature.FramesPrediction.Length; ++p)
                {
                    for (int f = 0; f < (feature.Project ? 2 : 3); f++)
                    {
                        FeaturesWeightsNativeArray[offset + f] = weight;
                    }
                    offset += (feature.Project ? 2 : 3);
                }
            }
            for (int i = 0; i < MMData.PoseFeatures.Count; i++)
            {
                PoseFeature feature = MMData.PoseFeatures[i];
                float weight = FeatureWeights[i + MMData.TrajectoryFeatures.Count] * Quality;
                FeaturesWeightsNativeArray[offset + 0] = weight;
                FeaturesWeightsNativeArray[offset + 1] = weight;
                FeaturesWeightsNativeArray[offset + 2] = weight;
                offset += 3;
            }

            // Init Query Vector
            FeatureSet.GetFeature(QueryFeature, CurrentFrame);
            FillTrajectory(QueryFeature);

            // Get next feature vector (when doing motion matching search, they need less error than this)
            float currentDistance = float.MaxValue;
            bool currentValid = false;
            if (FeatureSet.IsValidFeature(CurrentFrame))
            {
                currentValid = true;
                currentDistance = 0.0f;
                // the pose is the same... the distance is only the trajectory
                for (int j = 0; j < FeatureSet.PoseOffset; j++)
                {
                    float diff = FeatureSet.GetFeatures()[CurrentFrame * FeatureSet.FeatureSize + j] - QueryFeature[j];
                    currentDistance += diff * diff * FeaturesWeightsNativeArray[j];
                }
            }

            // Search
            var job = new LinearMotionMatchingSearchBurst
            {
                Valid = FeatureSet.GetValid(),
                Features = FeatureSet.GetFeatures(),
                QueryFeature = QueryFeature,
                FeatureWeights = FeaturesWeightsNativeArray,
                FeatureSize = FeatureSet.FeatureSize,
                PoseOffset = FeatureSet.PoseOffset,
                CurrentDistance = currentDistance,
                BestIndex = SearchResult
            };
            job.Schedule().Complete();

            // Check if use current or best
            int best = SearchResult[0];
            if (currentValid && best == -1) best = CurrentFrame;

            return best;
        }

        private void FillTrajectory(NativeArray<float> vector)
        {
            int offset = 0;
            for (int i = 0; i < MMData.TrajectoryFeatures.Count; i++)
            {
                TrajectoryFeature feature = MMData.TrajectoryFeatures[i];
                for (int p = 0; p < feature.FramesPrediction.Length; ++p)
                {
                    float3 value = CharacterController.GetWorldSpacePrediction(feature, p);
                    switch (feature.FeatureType)
                    {
                        case TrajectoryFeature.Type.Position:
                            value = GetPositionLocalCharacter(value);
                            break;
                        case TrajectoryFeature.Type.Direction:
                            value = GetDirectionLocalCharacter(value);
                            break;
                        default:
                            Debug.Assert(false, "Unknown feature type: " + feature.FeatureType);
                            break;
                    }
                    if (feature.Project)
                    {
                        vector[offset + 0] = value.x;
                        vector[offset + 1] = value.z;
                        offset += 2;
                    }
                    else
                    {
                        vector[offset + 0] = value.x;
                        vector[offset + 1] = value.y;
                        vector[offset + 2] = value.z;
                        offset += 3;
                    }
                }
            }
            // Normalize (only trajectory... because current FeatureVector is already normalized)
            FeatureSet.NormalizeTrajectory(vector);
        }

        private void UpdateTransformAndSkeleton(int frameIndex)
        {
            PoseSet.GetPose(frameIndex, out PoseVector pose);
            // Update Inertialize if enabled
            if (Inertialize)
            {
                Inertialization.Update(pose, InertializeHalfLife, Time.deltaTime);
            }
            // Simulation Bone
            float3 previousPosition = transform.position;
            transform.position += transform.TransformDirection(pose.RootDisplacement);
            transform.rotation = transform.rotation * pose.RootRotDisplacement;
            Velocity = ((float3)transform.position - previousPosition) / Time.deltaTime;
            // Joints
            if (Inertialize)
            {
                for (int i = 0; i < Inertialization.InertializedRotations.Length; i++)
                {
                    SkeletonTransforms[i].localRotation = Inertialization.InertializedRotations[i];
                }
            }
            else
            {
                for (int i = 0; i < pose.JointLocalRotations.Length; i++)
                {
                    SkeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
                }
            }
            // Correct Root Orientation to match the Simulation Bone
            float3 characterForward = transform.forward;
            characterForward = math.normalize(new float3(characterForward.x, 0, characterForward.z));
            float3 hipsForward = math.mul(SkeletonTransforms[0].rotation, MMData.HipsForwardLocalVector);
            hipsForward = math.normalize(new float3(hipsForward.x, 0, hipsForward.z));
            SkeletonTransforms[0].rotation = math.mul(MathExtensions.FromToRotation(hipsForward, characterForward, new float3(0, 1, 0)), SkeletonTransforms[0].rotation);
            // Root Y Position
            SkeletonTransforms[0].localPosition = new float3(0, Inertialize ? Inertialization.InertializedHipsY : pose.RootWorld.y, 0);
            // Post processing the transforms
            if (OnSkeletonTransformUpdated != null) OnSkeletonTransformUpdated.Invoke();
        }

        private float3 GetPositionLocalCharacter(float3 worldPosition)
        {
            return transform.InverseTransformPoint(worldPosition);
        }

        private float3 GetDirectionLocalCharacter(float3 worldDir)
        {
            return transform.InverseTransformDirection(worldDir);
        }

        /// <summary>
        /// Returns the current frame index in the pose set
        /// </summary>
        public int GetCurrentFrame()
        {
            return CurrentFrame;
        }

        /// <summary>
        /// Returns the skeleton used by Motion Matching
        /// </summary>
        public Skeleton GetSkeleton()
        {
            return PoseSet.Skeleton;
        }

        /// <summary>
        /// Returns the transforms used by Motion Matching to simulate the skeleton
        /// </summary>
        public Transform[] GetSkeletonTransforms()
        {
            return SkeletonTransforms;
        }

        private void OnDestroy()
        {
            if (FeatureSet != null) FeatureSet.Dispose();
            if (QueryFeature != null && QueryFeature.IsCreated) QueryFeature.Dispose();
            if (SearchResult != null && SearchResult.IsCreated) SearchResult.Dispose();
            if (FeaturesWeightsNativeArray != null && FeaturesWeightsNativeArray.IsCreated) FeaturesWeightsNativeArray.Dispose();
        }

        private void OnApplicationQuit()
        {
            OnDestroy();
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            // Skeleton
            if (SkeletonTransforms == null || PoseSet == null) return;

            if (DebugSkeleton)
            {
                Gizmos.color = Color.red;
                for (int i = 1; i < SkeletonTransforms.Length; i++)
                {
                    Transform t = SkeletonTransforms[i];
                    Gizmos.DrawLine(t.parent.position, t.position);
                }
            }

            // Character
            if (PoseSet == null) return;

            int currentFrame = CurrentFrame;
            PoseSet.GetPose(currentFrame, out PoseVector pose);
            float3 characterOrigin = transform.position;
            float3 characterForward = transform.forward;
            if (DebugCurrent)
            {
                Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
                Gizmos.DrawSphere(characterOrigin, SpheresRadius);
                GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward);
            }

            // Feature Set
            if (FeatureSet == null) return;

            FeatureDebug.DrawFeatureGizmos(FeatureSet, MMData, SpheresRadius, currentFrame, characterOrigin, characterForward,
                                           SkeletonTransforms, PoseSet.Skeleton, DebugPose, DebugTrajectory);
        }
#endif
    }
}