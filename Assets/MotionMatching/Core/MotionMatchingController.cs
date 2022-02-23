using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;

namespace MotionMatching
{
    // Simulation bone is the transform
    public class MotionMatchingController : MonoBehaviour
    {
        public event Action<Skeleton, Transform[]> OnSkeletonTransformUpdated;

        public SpringCharacterController CharacterController;
        public MotionMatchingData MMData;
        public float SpheresRadius = 0.1f;
        public bool LockFPS = true;
        public float3 DefaultHipsForward = new float3(0, 0, 1);
        public int SearchFrames = 10; // Motion Matching every SearchFrames frames
        public bool Normalize = false;
        public FeatureSet.NormalizeType NormalizeType = FeatureSet.NormalizeType.Magnitude;
        [Range(0.0f, 1.0f)] public float Responsiveness = 1.0f;
        [Range(0.0f, 1.0f)] public float Quality = 1.0f;
        [Header("Debug")]
        public bool DebugSkeleton = true;
        public bool DebugCurrent = true;
        public bool DebugJoints = true;
        public bool DebugTrajectory = true;


        private BVHAnimation Animation;
        private PoseSet PoseSet;
        private FeatureSet FeatureSet;
        private Transform[] SkeletonTransforms;
        private int CurrentFrame;
        private int SearchFrameCount;
        private FeatureVector QueryFeature;
        private NativeArray<int> SearchResult;

        private void Awake()
        {
            // Assertions
            Debug.Assert(math.length(DefaultHipsForward) < 1.01f && math.length(DefaultHipsForward) > 0.99f, "DefaultHipsForward must be normalized");

            // BVH
            PROFILE.BEGIN_SAMPLE_PROFILING("BVH Import");
            BVHImporter importer = new BVHImporter();
            Animation = importer.Import(MMData.BVH, MMData.UnitScale);
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("BVH Import");

            // Add Mecanim mapping information
            Animation.UpdateMecanimInformation(MMData);

            PROFILE.BEGIN_SAMPLE_PROFILING("Pose Extract", true);
            PoseExtractor poseExtractor = new PoseExtractor();
            PoseSet = new PoseSet();
            if (!poseExtractor.Extract(Animation, PoseSet, DefaultHipsForward))
            {
                Debug.LogError("[FeatureDebug] Failed to extract pose from BVHAnimation");
            }
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Pose Extract", true);

            PROFILE.BEGIN_SAMPLE_PROFILING("Feature Extract", true);
            FeatureExtractor featureExtractor = new FeatureExtractor();
            FeatureSet = featureExtractor.Extract(PoseSet, DefaultHipsForward);
            if (Normalize) FeatureSet.NormalizeFeatures(NormalizeType);
            PROFILE.END_AND_PRINT_SAMPLE_PROFILING("Feature Extract", true);

            // Skeleton
            SkeletonTransforms = new Transform[Animation.Skeleton.Joints.Count];
            foreach (Skeleton.Joint joint in Animation.Skeleton.Joints)
            {
                Transform t = (new GameObject()).transform;
                t.name = joint.Name;
                if (joint.Index == 0) t.SetParent(transform, false);
                else t.SetParent(SkeletonTransforms[joint.ParentIndex], false);
                t.localPosition = joint.LocalOffset;
                SkeletonTransforms[joint.Index] = t;
            }

            // FPS
            if (LockFPS)
            {
                Application.targetFrameRate = (int)(1.0f / Animation.FrameTime);
                Debug.Log("[Motion Matching] Updated Target FPS: " + Application.targetFrameRate);
            }
            else
            {
                Application.targetFrameRate = -1;
            }

            // Other initialization
            SearchResult = new NativeArray<int>(1, Allocator.Persistent);
        }

        private void OnEnable()
        {
            SearchFrameCount = 0;
            CharacterController.OnUpdate += OnCharacterControllerUpdated;
            CharacterController.OnInputChangedQuickly += OnInputChangedQuickly;
        }

        private void OnDisable()
        {
            CharacterController.OnUpdate -= OnCharacterControllerUpdated;
            CharacterController.OnInputChangedQuickly -= OnInputChangedQuickly;
        }

        private void Start()
        {
            QueryFeature = new FeatureVector();
        }

        private void OnCharacterControllerUpdated(float deltaTime)
        {
            PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Total");
            if (SearchFrameCount == 0)
            {
                // Motion Matching
                PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Search");
                int nextFrame = SearchMotionMatching();
                PROFILE.END_SAMPLE_PROFILING("Motion Matching Search");
                if (nextFrame != CurrentFrame)
                {
                    CurrentFrame = nextFrame;
                }
                else
                {
                    CurrentFrame += 1;
                }
                SearchFrameCount = SearchFrames;
            }
            else
            {
                // Advance
                CurrentFrame += 1;
                SearchFrameCount -= 1;
            }
            UpdateTransformAndSkeleton(CurrentFrame);
            PROFILE.END_SAMPLE_PROFILING("Motion Matching Total");
        }

        private void OnInputChangedQuickly()
        {
            SearchFrameCount = 0; // Force search
        }

        private int SearchMotionMatching()
        {
            // Init Query Vector
            for (int i = 0; i < CharacterController.NumberPrediction; ++i)
            {
                float2 worldPredictedPos = CharacterController.GetWorldPredictedPosition(i);
                QueryFeature.SetFutureTrajectoryLocalPosition(i, GetPositionLocalCharacter(worldPredictedPos));
                float2 worldPredictedDir = CharacterController.GetWorldPredictedDirection(i);
                QueryFeature.SetFutureTrajectoryLocalDirection(i, GetDirectionLocalCharacter(worldPredictedDir));
            }
            FeatureVector current = FeatureSet.GetFeature(CurrentFrame);
            QueryFeature.LeftFootLocalPosition = current.LeftFootLocalPosition;
            QueryFeature.LeftFootLocalVelocity = current.LeftFootLocalVelocity;
            QueryFeature.RightFootLocalPosition = current.RightFootLocalPosition;
            QueryFeature.RightFootLocalVelocity = current.RightFootLocalVelocity;
            QueryFeature.HipsLocalVelocity = current.HipsLocalVelocity;
            // Normalize
            if (Normalize) QueryFeature = FeatureSet.NormalizeFeatureVector(QueryFeature);
            // Search
            var job = new LinearMotionMatchingSearchBurst
            {
                Features = FeatureSet.GetFeatures(),
                QueryFeature = QueryFeature,
                Responsiveness = Responsiveness,
                Quality = Quality,
                BestIndex = SearchResult
            };
            job.Schedule().Complete();
            return SearchResult[0];
        }

        private void UpdateTransformAndSkeleton(int frameIndex)
        {
            PoseVector pose = PoseSet.Poses[frameIndex];
            // Simulation Bone
            transform.position += transform.TransformDirection(pose.RootVelocity);
            transform.rotation = transform.rotation * pose.RootRotVelocity;
            // Joints
            for (int i = 0; i < pose.JointLocalRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            }
            // Correct Root Orientation to match the Simulation Bone
            float3 characterForward = transform.forward;
            characterForward = math.normalize(new float3(characterForward.x, 0, characterForward.z));
            float3 hipsForward = math.mul(SkeletonTransforms[0].rotation, DefaultHipsForward);
            hipsForward = math.normalize(new float3(hipsForward.x, 0, hipsForward.z));
            SkeletonTransforms[0].rotation = math.mul(MathExtensions.FromToRotation(hipsForward, characterForward), SkeletonTransforms[0].rotation);
            // Root Y Position
            SkeletonTransforms[0].localPosition = new float3(0, pose.RootWorld.y, 0);

            if (OnSkeletonTransformUpdated != null) OnSkeletonTransformUpdated.Invoke(Animation.Skeleton, SkeletonTransforms);
        }

        private float2 GetPositionLocalCharacter(float2 worldPosition)
        {
            float3 localPosition = transform.InverseTransformPoint(new float3(worldPosition.x, 0.0f, worldPosition.y));
            return new float2(localPosition.x, localPosition.z);
        }

        private float2 GetDirectionLocalCharacter(float2 worldDir)
        {
            float3 localDir = transform.InverseTransformDirection(new float3(worldDir.x, 0.0f, worldDir.y));
            return new float2(localDir.x, localDir.z);
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
            return Animation.Skeleton;
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
            FeatureSet.Dispose();
            if (SearchResult != null && SearchResult.IsCreated) SearchResult.Dispose();
        }

        private void OnApplicationQuit()
        {
            FeatureSet.Dispose();
            if (SearchResult != null && SearchResult.IsCreated) SearchResult.Dispose();
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            // Skeleton
            if (SkeletonTransforms == null || Animation == null || Animation.EndSites == null) return;

            if (DebugSkeleton)
            {
                Gizmos.color = Color.red;
                for (int i = 1; i < SkeletonTransforms.Length; i++)
                {
                    Transform t = SkeletonTransforms[i];
                    Gizmos.DrawLine(t.parent.position, t.position);
                }
                foreach (BVHAnimation.EndSite endSite in Animation.EndSites)
                {
                    Transform t = SkeletonTransforms[endSite.ParentIndex];
                    Gizmos.DrawLine(t.position, t.TransformPoint(endSite.Offset));
                }
            }

            // Character
            if (PoseSet == null) return;

            int currentFrame = CurrentFrame;
            if (currentFrame < 0) currentFrame = Animation.Frames.Length - 1;
            PoseVector pose = PoseSet.Poses[currentFrame];
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

            FeatureVector fv = FeatureSet.GetFeature(currentFrame);
            if (fv.Valid)
            {
                quaternion characterRot = quaternion.LookRotation(characterForward, new float3(0, 1, 0));
                if (DebugJoints)
                {
                    // Left Foot
                    Gizmos.color = Color.cyan;
                    float3 leftFootWorld = characterOrigin + math.mul(characterRot, fv.LeftFootLocalPosition);
                    Gizmos.DrawWireSphere(leftFootWorld, SpheresRadius);
                    float3 leftFootVelWorld = math.mul(characterRot, fv.LeftFootLocalVelocity);
                    if (math.length(leftFootVelWorld) > 0.001f)
                    {
                        GizmosExtensions.DrawArrow(leftFootWorld, leftFootWorld + leftFootVelWorld * 0.1f, 0.25f * math.length(leftFootVelWorld) * 0.1f);
                    }
                    // Right Foot
                    Gizmos.color = Color.yellow;
                    float3 rightFootWorld = characterOrigin + math.mul(characterRot, fv.RightFootLocalPosition);
                    Gizmos.DrawWireSphere(rightFootWorld, SpheresRadius);
                    float3 rightFootVelWorld = math.mul(characterRot, fv.RightFootLocalVelocity);
                    if (math.length(rightFootVelWorld) > 0.001f)
                    {
                        GizmosExtensions.DrawArrow(rightFootWorld, rightFootWorld + rightFootVelWorld * 0.1f, 0.25f * math.length(rightFootVelWorld) * 0.1f);
                    }
                    // Hips
                    Gizmos.color = Color.green;
                    float3 hipsVelWorld = math.mul(characterRot, fv.HipsLocalVelocity);
                    if (math.length(hipsVelWorld) > 0.001f)
                    {
                        GizmosExtensions.DrawArrow(SkeletonTransforms[0].position, SkeletonTransforms[0].position + (Vector3)(hipsVelWorld * 0.1f), 0.25f * math.length(hipsVelWorld) * 0.1f);
                    }
                }
                if (DebugTrajectory)
                {
                    // Trajectory
                    for (int i = 0; i < FeatureVector.GetFutureTrajectoryLength(); ++i)
                    {
                        Gizmos.color = Color.blue * (1.0f - (float)i / (FeatureVector.GetFutureTrajectoryLength() * 1.25f));
                        float2 futurePos = fv.GetFutureTrajectoryLocalPosition(i);
                        float3 futureWorld = characterOrigin + math.mul(characterRot, (new float3(futurePos.x, 0.0f, futurePos.y)));
                        Gizmos.DrawSphere(futureWorld, SpheresRadius);
                        float2 futureDir = fv.GetFutureTrajectoryLocalDirection(i);
                        float3 futureDirWorld = math.mul(characterRot, (new float3(futureDir.x, 0.0f, futureDir.y)));
                        GizmosExtensions.DrawArrow(futureWorld, futureWorld + futureDirWorld);
                    }
                }
            }
        }
#endif
    }
}