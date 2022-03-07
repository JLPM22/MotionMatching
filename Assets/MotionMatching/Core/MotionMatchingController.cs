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
        public event Action OnSkeletonTransformUpdated;

        public MotionMatchingCharacterController CharacterController;
        public MotionMatchingData MMData;
        public bool LockFPS = true;
        public int SearchFrames = 10; // Motion Matching every SearchFrames frames
        public bool Inertialize = true; // Should inertialize transitions after a big change of the pose
        [Range(0.0f, 1.0f)] public float InertializeHalfLife = 0.1f; // Time needed to move half of the distance between the source to the target pose
        [Tooltip("How important is the trajectory (future positions + future directions)")][Range(0.0f, 1.0f)] public float Responsiveness = 1.0f;
        [Tooltip("How important is the current pose")][Range(0.0f, 1.0f)] public float Quality = 1.0f;
        // TODO: generalize this and do custom editor
        public float[] FeatureWeights = { 1, 1, 1, 1, 1, 1, 1 };
        [Header("Debug")]
        public float SpheresRadius = 0.1f;
        public bool DebugSkeleton = true;
        public bool DebugCurrent = true;
        public bool DebugJoints = true;
        public bool DebugTrajectory = true;

        public float3 Velocity { get; private set; }

        private PoseSet PoseSet;
        private FeatureSet FeatureSet;
        private Transform[] SkeletonTransforms;
        private int CurrentFrame;
        private int SearchFrameCount;
        private FeatureVector QueryFeature;
        private NativeArray<int> SearchResult;
        private NativeArray<float> FeaturesWeightsNativeArray; // TODO: remove this... maybe it is not necessary float[] + Native Array with Custom Editor 
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
                Application.targetFrameRate = (int)(1.0f / PoseSet.Clips[0].FrameTime);
                Debug.Log("[Motion Matching] Updated Target FPS: " + Application.targetFrameRate);
            }
            else
            {
                Application.targetFrameRate = -1;
            }

            // Other initialization
            SearchResult = new NativeArray<int>(1, Allocator.Persistent);
            FeaturesWeightsNativeArray = new NativeArray<float>(FeatureWeights.Length, Allocator.Persistent);
            // Search first Frame valid (to start with a valid pose)
            for (int i = 0; i < FeatureSet.GetFeatures().Length; i++)
            {
                if (FeatureSet.GetFeature(i).IsValid)
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
                    int previousFrame = CurrentFrame;
                    CurrentFrame = nextFrame;

                    // Inertialize
                    if (Inertialize)
                    {
                        Inertialization.PoseTransition(PoseSet, previousFrame, CurrentFrame);
                    }
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
            // Normalize (only trajectory... because current FeatureVector is already normalized)
            QueryFeature = FeatureSet.NormalizeTrajectory(QueryFeature);
            // Weights
            for (int i = 0; i < FeatureWeights.Length; i++) FeaturesWeightsNativeArray[i] = FeatureWeights[i];
            // Search
            var job = new LinearMotionMatchingSearchBurst
            {
                Features = FeatureSet.GetFeatures(),
                QueryFeature = QueryFeature,
                Responsiveness = Responsiveness,
                Quality = Quality,
                FeatureWeights = FeaturesWeightsNativeArray,
                BestIndex = SearchResult
            };
            job.Schedule().Complete();
            return SearchResult[0];
        }

        private void UpdateTransformAndSkeleton(int frameIndex)
        {
            PoseVector pose = PoseSet.Poses[frameIndex];
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
            FeatureSet.Dispose();
            if (SearchResult != null && SearchResult.IsCreated) SearchResult.Dispose();
            if (FeaturesWeightsNativeArray != null && FeaturesWeightsNativeArray.IsCreated) FeaturesWeightsNativeArray.Dispose();
        }

        private void OnApplicationQuit()
        {
            FeatureSet.Dispose();
            if (SearchResult != null && SearchResult.IsCreated) SearchResult.Dispose();
            if (FeaturesWeightsNativeArray != null && FeaturesWeightsNativeArray.IsCreated) FeaturesWeightsNativeArray.Dispose();
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
            fv = FeatureSet.DenormalizeFeatureVector(fv);
            if (fv.IsValid)
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