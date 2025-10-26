using System;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using UnityEditor;
using System.Diagnostics;
using System.Collections.Generic;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;
    using Debug = UnityEngine.Debug;
    using static MotionMatching.MotionMatchingData;

    // Simulation bone is the transform
    public class MotionMatchingController : MonoBehaviour
    {
        public event Action OnSkeletonTransformUpdated;

        [Header("General")]
        public MotionMatchingCharacterController CharacterController;
        public MotionMatchingData MMData;
        public MotionMatchingSearch Search;
        public bool LockFPS = true;
        public float SearchTime = 10.0f / 60.0f; // Motion Matching search every SearchTime seconds
        public bool Inertialize = true; // Should inertialize transitions after a big change of the pose
        public bool FootLock = true; // Should lock the feet to the ground when contact information is true
        public float FootUnlockDistance = 0.2f; // Distance from actual pose to IK target to unlock the feet
        [Range(0.0f, 1.0f)] public float InertializeHalfLife = 0.1f; // Time needed to move half of the distance between the source to the target pose
        [Tooltip("How important is the trajectory (future positions + future directions)")][Range(0.0f, 1.0f)] public float Responsiveness = 1.0f;
        [Tooltip("How important is the current pose")][Range(0.0f, 1.0f)] public float Quality = 1.0f;
        [HideInInspector] public float[] FeatureWeights;
        [Header("Debug")]
        public float SpheresRadius = 0.1f;
        public bool DebugSkeleton = true;
        public bool DebugFutureSkeleton = true;
        public bool DebugCurrent = true;
        public bool DebugPose = true;
        public bool DebugTrajectory = true;
        public bool DebugEnvironment = true;
        public bool DebugSearch = true;
        public bool DebugContacts = true;
        public bool DebugGUI = true;

        public float3 Velocity { get; private set; }
        public float3 AngularVelocity { get; private set; }
        public float DatabaseFrameTime { get; private set; }
        public int DatabaseFrameRate { get; private set; }
        public PoseSet PoseSet { get; private set; }
        public FeatureSet FeatureSet { get; private set; }
        public float SearchTimeLeft { get; private set; }
        public Transform[] SkeletonTransforms { get; private set; }
        public NativeArray<float> QueryFeature { get; private set; }
        public NativeArray<float> FeaturesWeightsNativeArray { get; private set; }
        public int CurrentFrame { get; private set; } // Current frame index in the pose/feature set
        public int LastMMSearchFrame { get; private set; } // Frame before the last Motion Matching Search
        public NativeArray<bool> TagMask { get; private set; }

        private float3 AnimationSpaceOriginPos;
        private quaternion InverseAnimationSpaceOriginRot;
        private float3 MMTransformOriginPose; // Position of the transform right after motion matching search
        private quaternion MMTransformOriginRot; // Rotation of the transform right after motion matching search
        private float CurrentFrameTime; // Current frame index as float to keep track of variable frame rate
        private Inertialization Inertialization;
        // Foot Lock
        private bool IsLeftFootContact, IsRightFootContact;
        private float3 LeftToesContactTarget, RightToesContactTarget; // Target position of the toes
        private float3 LeftFootContact, RightFootContact; // Position of the foot
        private float3 LeftFootPoleContact, RightFootPoleContact; // Forward vector of the knee
        private float3 LeftLowerLegLocalForward, RightLowerLegLocalForward;
        private int LeftToesIndex, LeftFootIndex, LeftLowerLegIndex, LeftUpperLegIndex;
        private int RightToesIndex, RightFootIndex, RightLowerLegIndex, RightUpperLegIndex;
        // Other
        private bool IsDestroyed = false;
        private MotionMatchingSearch SearchInstance;

        private void Awake()
        {
            PoseSet = MMData.GetOrImportPoseSet();
            FeatureSet = MMData.GetOrImportFeatureSet();

            // Skeleton
            SkeletonTransforms = new Transform[PoseSet.Skeleton.Joints.Count];
            SkeletonTransforms[0] = transform; // Simulation Bone
            for (int j = 1; j < PoseSet.Skeleton.Joints.Count; j++)
            {
                // Joints
                Skeleton.Joint joint = PoseSet.Skeleton.Joints[j];
                Transform t = new GameObject().transform;
                t.name = joint.Name;
                t.SetParent(SkeletonTransforms[joint.ParentIndex], false);
                t.localPosition = joint.LocalOffset;
                SkeletonTransforms[j] = t;
            }

            // Inertialization
            Inertialization = new Inertialization(PoseSet.Skeleton);

            // FPS
            DatabaseFrameTime = PoseSet.FrameTime;
            DatabaseFrameRate = Mathf.RoundToInt(1.0f / DatabaseFrameTime);
            if (LockFPS)
            {
                Application.targetFrameRate = DatabaseFrameRate;
                Debug.Log("[Motion Matching] Updated Target FPS: " + Application.targetFrameRate);
            }
            else
            {
                Application.targetFrameRate = -1;
                Debug.LogWarning("[Motion Matching] LockFPS is not set. Motion Matching will malfunction if the application frame rate is higher than the animation database.");
            }

            // Other initialization
            int numberFeatures = MMData.TrajectoryFeatures.Count + MMData.PoseFeatures.Count + MMData.EnvironmentFeatures.Count;
            if (FeatureWeights == null || FeatureWeights.Length != numberFeatures)
            {
                float[] newWeights = new float[numberFeatures];
                for (int i = 0; i < newWeights.Length; ++i) newWeights[i] = 1.0f;
                for (int i = 0; i < Mathf.Min(FeatureWeights.Length, newWeights.Length); i++) newWeights[i] = FeatureWeights[i];
                FeatureWeights = newWeights;
            }
            FeaturesWeightsNativeArray = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);
            QueryFeature = new NativeArray<float>(FeatureSet.FeatureSize, Allocator.Persistent);

            // Tags
            TagMask = new NativeArray<bool>(FeatureSet.NumberFeatureVectors, Allocator.Persistent);
            DisableQueryTag();

            // Foot Lock
            if (!PoseSet.Skeleton.Find(HumanBodyBones.LeftToes, out Skeleton.Joint leftToesJoint)) Debug.LogError("[Motion Matching] LeftToes not found");
            LeftToesIndex = leftToesJoint.Index;
            if (!PoseSet.Skeleton.Find(HumanBodyBones.LeftFoot, out Skeleton.Joint leftFootJoint)) Debug.LogError("[Motion Matching] LeftFoot not found");
            LeftFootIndex = leftFootJoint.Index;
            if (!PoseSet.Skeleton.Find(HumanBodyBones.LeftLowerLeg, out Skeleton.Joint leftLowerLegJoint)) Debug.LogError("[Motion Matching] LeftLowerLeg not found");
            LeftLowerLegIndex = leftLowerLegJoint.Index;
            if (!PoseSet.Skeleton.Find(HumanBodyBones.LeftUpperLeg, out Skeleton.Joint leftUpperLegJoint)) Debug.LogError("[Motion Matching] LeftUpperLeg not found");
            LeftUpperLegIndex = leftUpperLegJoint.Index;
            if (!PoseSet.Skeleton.Find(HumanBodyBones.RightToes, out Skeleton.Joint rightToesJoint)) Debug.LogError("[Motion Matching] RightToes not found");
            RightToesIndex = rightToesJoint.Index;
            if (!PoseSet.Skeleton.Find(HumanBodyBones.RightFoot, out Skeleton.Joint rightFootJoint)) Debug.LogError("[Motion Matching] RightFoot not found");
            RightFootIndex = rightFootJoint.Index;
            if (!PoseSet.Skeleton.Find(HumanBodyBones.RightLowerLeg, out Skeleton.Joint rightLowerLegJoint)) Debug.LogError("[Motion Matching] RightLowerLeg not found");
            RightLowerLegIndex = rightLowerLegJoint.Index;
            if (!PoseSet.Skeleton.Find(HumanBodyBones.RightUpperLeg, out Skeleton.Joint rightUpperLegJoint)) Debug.LogError("[Motion Matching] RightUpperLeg not found");
            RightUpperLegIndex = rightUpperLegJoint.Index;
            LeftLowerLegLocalForward = MMData.GetLocalForward(LeftLowerLegIndex);
            RightLowerLegLocalForward = MMData.GetLocalForward(RightLowerLegIndex);

            // Init Pose
            SkeletonTransforms[0].SetPositionAndRotation(CharacterController.GetWorldInitPosition(),
                                                         quaternion.LookRotation(CharacterController.GetWorldInitDirection(), Vector3.up));

            // Search first Frame valid (to start with a valid pose)
            for (int i = 0; i < FeatureSet.NumberFeatureVectors; i++)
            {
                if (FeatureSet.IsValidFeature(i) && TagMask[i])
                {
                    LastMMSearchFrame = i;
                    CurrentFrame = i;
                    UpdateAnimationSpaceOrigin();
                    break;
                }
            }

            // Search Strategy
            SearchInstance = Instantiate<MotionMatchingSearch>(Search);
            SearchInstance.Initialize(this);
        }

        private void OnEnable()
        {
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
            if (SearchInstance.ShouldSearch(this))
            {
                // Motion Matching
                float currentDistance = PrepareQueryVector(out bool isCurrentValid);
                PROFILE.BEGIN_SAMPLE_PROFILING("Motion Matching Search");
                int bestFrame = SearchInstance.FindBestFrame(this, currentDistance);
                PROFILE.END_SAMPLE_PROFILING("Motion Matching Search");
                // Check if use current or best
                if (isCurrentValid && bestFrame == -1) bestFrame = CurrentFrame;
                Debug.Assert(bestFrame != -1, "Motion Matching is not able to find any valid pose. Maybe the motion database is empty or the query tag used produces an empty set of poses?");
                const int ignoreSurrounding = 20; // ignore near frames
                if (math.abs(bestFrame - CurrentFrame) > ignoreSurrounding)
                {
                    // Inertialize
                    if (Inertialize)
                    {
                        Inertialization.PoseTransition(PoseSet, CurrentFrame, bestFrame);
                    }
                    LastMMSearchFrame = CurrentFrame;
                    CurrentFrameTime = bestFrame + math.frac(CurrentFrameTime); // the fractional part is the error accumulated, add it to the current to avoid drifting
                    CurrentFrame = bestFrame;
                    UpdateAnimationSpaceOrigin();
                }
                SearchTimeLeft = SearchTime;
            }
            else
            {
                // Advance
                SearchTimeLeft -= deltaTime;
            }
            // Always advance one (bestFrame from motion matching is the best match to the current frame, but we want to move to the next frame)
            // Ideally the applications runs at 1.0f/FrameTime fps (to match the database) however, as this may not happen, we may need to skip some frames
            // from the database, e.g., if 1.0f/FrameTime = 60 and our game runes at 30, we need to advance 2 frames at each update
            // However, as we are using Application.targetFrameRate=1.0f/FrameTime, we do not consider the case where the application runs faster than the database
            CurrentFrameTime += DatabaseFrameRate * deltaTime; // DatabaseFrameRate / (1.0f / deltaTime)
            CurrentFrame = (int)math.floor(CurrentFrameTime);

            UpdateTransformAndSkeleton(CurrentFrame);
            PROFILE.END_SAMPLE_PROFILING("Motion Matching Total");

            SearchInstance.OnSearchCompleted(this);
        }

        private void UpdateAnimationSpaceOrigin()
        {
            PoseSet.GetPose(CurrentFrame, out PoseVector mmPose);
            AnimationSpaceOriginPos = mmPose.JointLocalPositions[0];
            InverseAnimationSpaceOriginRot = math.inverse(mmPose.JointLocalRotations[0]);
            MMTransformOriginPose = SkeletonTransforms[0].position;
            MMTransformOriginRot = SkeletonTransforms[0].rotation;
        }

        private void OnInputChangedQuickly()
        {
            SearchTimeLeft = 0; // Force search
        }

        private float PrepareQueryVector(out bool isCurrentValid)
        {
            // Weights
            UpdateAndGetFeatureWeights();

            // Init Query Vector
            FeatureSet.GetFeature(QueryFeature, CurrentFrame);
            FillQueryVector();

            // Get next feature vector (when doing motion matching search, they need less error than this)
            float currentDistance = float.MaxValue;
            isCurrentValid = FeatureSet.IsValidFeature(CurrentFrame) && TagMask[CurrentFrame];
            if (isCurrentValid)
            {
                currentDistance = 0.0f;
                // the pose is the same... the distance is only the trajectory
                for (int j = 0; j < FeatureSet.PoseOffset; j++)
                {
                    float diff = FeatureSet.GetFeatures()[CurrentFrame * FeatureSet.FeatureSize + j] - QueryFeature[j];
                    currentDistance += diff * diff * FeaturesWeightsNativeArray[j];
                }
            }
            return currentDistance;
        }

        public void FillQueryVector()
        {
            NativeArray<float> queryFeature = QueryFeature;
            int offset = 0;
            for (int i = 0; i < MMData.TrajectoryFeatures.Count; i++)
            {
                TrajectoryFeature feature = MMData.TrajectoryFeatures[i];
                for (int p = 0; p < feature.FramesPrediction.Length; ++p)
                {
                    int featureSize = feature.GetSize();
                    Debug.Assert(featureSize > 0, "Trajectory feature size must be larger than 0");
                    NativeArray<float> featureVector = new(featureSize, Allocator.Temp);
                    CharacterController.GetTrajectoryFeature(feature, p, SkeletonTransforms[0], featureVector);
                    for (int j = 0; j < featureSize; j++)
                    {
                        queryFeature[offset + j] = featureVector[j];
                    }
                    offset += featureSize;
                }
            }
            // Normalize (only trajectory... because current FeatureVector is already normalized)
            FeatureSet.NormalizeTrajectory(queryFeature);

            if (FeatureSet.EnvironmentOffset.Length > 0)
            {
                offset = FeatureSet.EnvironmentOffset[0];
                for (int i = 0; i < MMData.EnvironmentFeatures.Count; i++)
                {
                    TrajectoryFeature feature = MMData.EnvironmentFeatures[i];
                    for (int p = 0; p < feature.FramesPrediction.Length; p++)
                    {
                        int featureSize = feature.GetSize();
                        Debug.Assert(featureSize > 0, "Environment feature size must be larger than 0");
                        NativeArray<float> featureVector = new(featureSize, Allocator.Temp);
                        CharacterController.GetEnvironmentFeature(feature, p, SkeletonTransforms[0], featureVector);
                        for (int j = 0; j < featureSize; j++)
                        {
                            queryFeature[offset + j] = featureVector[j];
                        }
                        offset += featureSize;
                    }
                }
            }
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
            float3 previousPosition = SkeletonTransforms[0].position;
            quaternion previousRotation = SkeletonTransforms[0].rotation;
            // animation space to local space
            float3 localSpacePos = math.mul(InverseAnimationSpaceOriginRot, pose.JointLocalPositions[0] - AnimationSpaceOriginPos);
            quaternion localSpaceRot = math.mul(InverseAnimationSpaceOriginRot, pose.JointLocalRotations[0]);
            // local space to world space
            SkeletonTransforms[0].SetPositionAndRotation(math.mul(MMTransformOriginRot, localSpacePos) + MMTransformOriginPose,
                                                         math.mul(MMTransformOriginRot, localSpaceRot));
            // update velocity and angular velocity
            Velocity = ((float3)SkeletonTransforms[0].position - previousPosition) / Time.deltaTime;
            AngularVelocity = MathExtensions.AngularVelocity(previousRotation, SkeletonTransforms[0].rotation, Time.deltaTime);
            // Joints
            if (Inertialize)
            {
                for (int i = 1; i < Inertialization.InertializedRotations.Length; i++)
                {
                    SkeletonTransforms[i].localRotation = Inertialization.InertializedRotations[i];
                }
            }
            else
            {
                for (int i = 1; i < pose.JointLocalRotations.Length; i++)
                {
                    SkeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
                }
            }
            // Hips Position
            SkeletonTransforms[1].localPosition = Inertialize ? Inertialization.InertializedHips : pose.JointLocalPositions[1];
            // Foot Lock
            UpdateFootLock(pose);
            // Post processing the transforms
            if (OnSkeletonTransformUpdated != null) OnSkeletonTransformUpdated.Invoke();
        }

        private void UpdateFootLock(PoseVector pose)
        {
            float3 currentLeftToesPosition = SkeletonTransforms[LeftToesIndex].position;
            float3 currentRightToesPosition = SkeletonTransforms[RightToesIndex].position;
            // Compute input contact position velocity
            float3 currentLeftToesVelocity = (currentLeftToesPosition - (float3)LeftToesContactTarget) / Time.deltaTime;
            float3 currentRightToesVelocity = (currentRightToesPosition - (float3)RightToesContactTarget) / Time.deltaTime;
            LeftToesContactTarget = currentLeftToesPosition;
            RightToesContactTarget = currentRightToesPosition;

            // Update Inertializer
            Inertialization.UpdateContact(IsLeftFootContact ? LeftFootContact : currentLeftToesPosition,
                                              IsLeftFootContact ? float3.zero : currentLeftToesVelocity,
                                              IsRightFootContact ? RightFootContact : currentRightToesPosition,
                                              IsRightFootContact ? float3.zero : currentRightToesVelocity,
                                              InertializeHalfLife, Time.deltaTime);
            float3 leftContactPosition = Inertialization.InertializedLeftContact;
            float3 leftContactVelocity = Inertialization.InertializedLeftContactVelocity;
            float3 rightContactPosition = Inertialization.InertializedRightContact;
            float3 rightContactVelocity = Inertialization.InertializedRightContactVelocity;

            // If the contact point is too far from the current input position
            // unlock the contact
            bool unlockLeftContact = IsLeftFootContact && (math.length(LeftFootContact - currentLeftToesPosition) > FootUnlockDistance);
            bool unlockRightContact = IsRightFootContact && (math.length(RightFootContact - currentRightToesPosition) > FootUnlockDistance);

            // If the contact was previously inactive and now it is active,
            // transition to the locked contact state
            // Also, make sure the inertialization returns an almost 0 velocity before locking
            if (!IsLeftFootContact && pose.LeftFootContact && math.length(leftContactVelocity) < MMData.ContactVelocityThreshold)
            {
                // Contact point is the current position of the foot
                // projected onto the ground + foot height
                IsLeftFootContact = true;
                LeftFootContact = leftContactPosition;
                // LeftFootContact.y =  // TODO: Add foot height
                Transform leftLowerLeg = SkeletonTransforms[LeftLowerLegIndex];
                LeftFootPoleContact = math.mul(leftLowerLeg.rotation, LeftLowerLegLocalForward);

                if (Inertialize)
                {
                    Inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity, LeftFootContact, float3.zero);
                }
                else
                {
                    Inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity, currentLeftToesPosition, currentLeftToesVelocity);
                }
            }
            // If we need to unlock or previously in contact but now not
            // we transition to the input position
            else if (unlockLeftContact || (IsLeftFootContact && !pose.LeftFootContact))
            {
                IsLeftFootContact = false;

                if (Inertialize)
                {
                    Inertialization.LeftContactTransition(LeftFootContact, float3.zero, currentLeftToesPosition, currentLeftToesVelocity);
                }
                else
                {
                    Inertialization.LeftContactTransition(currentLeftToesPosition, currentLeftToesVelocity, currentLeftToesPosition, currentLeftToesVelocity);
                }
            }

            // Same for Right Foot
            if (!IsRightFootContact && pose.RightFootContact && math.length(rightContactVelocity) < MMData.ContactVelocityThreshold)
            {
                IsRightFootContact = true;
                RightFootContact = rightContactPosition;
                // RightFootContact.y = 0.0f;
                Transform rightLowerLeg = SkeletonTransforms[RightLowerLegIndex];
                RightFootPoleContact = math.mul(rightLowerLeg.rotation, RightLowerLegLocalForward);

                if (Inertialize)
                {
                    Inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity, RightFootContact, float3.zero);
                }
                else
                {
                    Inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity, currentRightToesPosition, currentRightToesVelocity);
                }
            }
            else if (unlockRightContact || (IsRightFootContact && !pose.RightFootContact))
            {
                IsRightFootContact = false;

                if (Inertialize)
                {
                    Inertialization.RightContactTransition(RightFootContact, float3.zero, currentRightToesPosition, currentRightToesVelocity);
                }
                else
                {
                    Inertialization.RightContactTransition(currentRightToesPosition, currentRightToesVelocity, currentRightToesPosition, currentRightToesVelocity);
                }
            }

            // IK to place the foot
            if (FootLock)
            {
                // Left Foot IK
                TwoJointIK.Solve((Vector3)leftContactPosition + (SkeletonTransforms[LeftFootIndex].position - SkeletonTransforms[LeftToesIndex].position),
                                 SkeletonTransforms[LeftUpperLegIndex],
                                 SkeletonTransforms[LeftLowerLegIndex],
                                 SkeletonTransforms[LeftFootIndex],
                                 LeftFootPoleContact);
                // Right Foot IK
                TwoJointIK.Solve((Vector3)rightContactPosition + (SkeletonTransforms[RightFootIndex].position - SkeletonTransforms[RightToesIndex].position),
                                 SkeletonTransforms[RightUpperLegIndex],
                                 SkeletonTransforms[RightLowerLegIndex],
                                 SkeletonTransforms[RightFootIndex],
                                 RightFootPoleContact);
            }
        }

        /// <summary>
        /// Disables any previous set tag or query so searches are performed over the entire pose set
        /// </summary>
        public void DisableQueryTag()
        {
            var job = new DisableTagBurst
            {
                TagMask = TagMask,
            };
            job.Schedule().Complete();
            OnInputChangedQuickly();
        }

        /// <summary>
        /// Motion Matching will only search over those poses belonging to the tag
        /// </summary>
        public void SetQueryTag(string name)
        {
            PoseSet.Tag tag = PoseSet.GetTag(name);
            // TODO: cache results to avoid duplicated computations...
            var job = new SetTagBurst
            {
                MaximumFramesPrediction = PoseSet.MaximumFramesPrediction,
                TagMask = TagMask,
                StartRanges = tag.GetStartRanges(),
                EndRanges = tag.GetEndRanges(),
            };
            job.Schedule().Complete();
            OnInputChangedQuickly();
        }

        /// <summary>
        /// Motion Matching will only search over those poses belonging to the query tag
        /// </summary>
        public void SetQueryTag(QueryTag query)
        {
            query.ComputeRanges(PoseSet);
            var job = new SetTagBurst
            {
                MaximumFramesPrediction = PoseSet.MaximumFramesPrediction,
                TagMask = TagMask,
                StartRanges = query.GetStartRanges(),
                EndRanges = query.GetEndRanges(),
            };
            job.Schedule().Complete();
            OnInputChangedQuickly();
        }

        /// <summary>
        /// Adds an offset to the current transform space (useful to move the character to a different position)
        /// Simply changing the transform won't work because motion matching applies root motion based on the current motion matching search space
        /// </summary>
        public void SetPosAdjustment(float3 posAdjustment)
        {
            MMTransformOriginPose += posAdjustment;
        }
        /// <summary>
        /// Adds a rot offset to the current transform space (useful to rotate the character to a different direction)
        /// Simply changing the transform won't work because motion matching applies root motion based on the current motion matching search space
        /// </summary>
        public void SetRotAdjustment(quaternion rotAdjustment)
        {
            MMTransformOriginRot = math.mul(rotAdjustment, MMTransformOriginRot);
        }

        public void SetCurrentFrame(int frame)
        {
            CurrentFrame = frame;
        }
        public NativeArray<float> UpdateAndGetFeatureWeights()
        {
            NativeArray<float> featuresWeightsNativeArray = FeaturesWeightsNativeArray;
            int offset = 0;
            for (int i = 0; i < MMData.TrajectoryFeatures.Count; i++)
            {
                TrajectoryFeature feature = MMData.TrajectoryFeatures[i];
                int featureSize = feature.GetSize();
                float weight = FeatureWeights[i] * Responsiveness;
                for (int p = 0; p < feature.FramesPrediction.Length; ++p)
                {
                    for (int f = 0; f < featureSize; f++)
                    {
                        featuresWeightsNativeArray[offset + f] = weight;
                    }
                    offset += featureSize;
                }
            }
            for (int i = 0; i < MMData.PoseFeatures.Count; i++)
            {
                float weight = FeatureWeights[i + MMData.TrajectoryFeatures.Count] * Quality;
                featuresWeightsNativeArray[offset + 0] = weight;
                featuresWeightsNativeArray[offset + 1] = weight;
                featuresWeightsNativeArray[offset + 2] = weight;
                offset += 3;
            }
            for (int i = 0; i < MMData.EnvironmentFeatures.Count; i++)
            {
                TrajectoryFeature feature = MMData.EnvironmentFeatures[i];
                int featureSize = feature.GetSize();
                float baseWeight = FeatureWeights[i + MMData.TrajectoryFeatures.Count + MMData.PoseFeatures.Count];
                float weight = SearchInstance.OnUpdateEnvironmentFeatureWeight(this, feature, baseWeight);
                for (int p = 0; p < feature.FramesPrediction.Length; ++p)
                {
                    for (int f = 0; f < featureSize; f++)
                    {
                        featuresWeightsNativeArray[offset + f] = weight;
                    }
                    offset += featureSize;
                }
            }
            return featuresWeightsNativeArray;
        }

        public float3 GetMainPositionFeature(int trajectoryIndex)
        {
            float3 characterOrigin = SkeletonTransforms[0].position;
            float3 characterForward = SkeletonTransforms[0].forward;
            quaternion characterRot = quaternion.LookRotation(characterForward, math.up());
            // Find Main Position Trajectory Index
            int t = -1;
            for (int i = 0; i < MMData.TrajectoryFeatures.Count; i++)
            {
                if (MMData.TrajectoryFeatures[i].IsMainPositionFeature)
                {
                    t = i;
                    break;
                }
            }
            if (t == -1)
            {
                Debug.LogError("[Motion Matching] No Main Position Trajectory Feature found.");
                return characterOrigin;
            }
            float3 value = FeatureDebug.Get3DValuePositionOrDirectionFeature(MMData.TrajectoryFeatures[t], FeatureSet, CurrentFrame, t, trajectoryIndex, isEnvironment: false);
            value = characterOrigin + math.mul(characterRot, value);
            return value;
        }

        public float4 GetEnvironmentFeature(string featureName, int trajectoryIndex)
        {
            float3 characterForward = SkeletonTransforms[0].forward;
            quaternion characterRot = quaternion.LookRotation(characterForward, math.up());
            int t = -1;
            for (int i = 0; i < MMData.EnvironmentFeatures.Count; i++)
            {
                if (MMData.EnvironmentFeatures[i].Name == featureName)
                {
                    t = i;
                    break;
                }
            }
            if (t == -1)
            {
                Debug.LogError("[Motion Matching] No Environment Feature with name " + featureName + " found.");
                return float4.zero;
            }
            float4 value = FeatureSet.Get4DEnvironmentFeature(CurrentFrame, t, trajectoryIndex);
            float primaryDistance = value.x;
            float secondaryDistance = value.y;
            float3 primaryAxisUnitCharacterSpace = new(value.z, 0.0f, value.w);
            float3 primaryAxisUnitWorldSpace = math.mul(characterRot, primaryAxisUnitCharacterSpace);
            float2 primaryAxisUnit = new(primaryAxisUnitWorldSpace.x, primaryAxisUnitWorldSpace.z);
            float2 secondaryAxisUnit = new(-primaryAxisUnit.y, primaryAxisUnit.x);
            return new float4(primaryAxisUnit * primaryDistance, secondaryAxisUnit * secondaryDistance);
        }

        private void OnDestroy()
        {
            if (IsDestroyed) return;
            IsDestroyed = true;
            MMData.Dispose();
            SearchInstance.Dispose();
            if (QueryFeature != null && QueryFeature.IsCreated) QueryFeature.Dispose();
            if (FeaturesWeightsNativeArray != null && FeaturesWeightsNativeArray.IsCreated) FeaturesWeightsNativeArray.Dispose();
            if (TagMask != null && TagMask.IsCreated) TagMask.Dispose();
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
                for (int i = 2; i < SkeletonTransforms.Length; i++) // skip Simulation Bone
                {
                    Transform t = SkeletonTransforms[i];
                    GizmosExtensions.DrawLine(t.parent.position, t.position, 6.0f);
                }
            }

            // Character
            if (PoseSet == null) return;

            int currentFrame = CurrentFrame;
            float3 characterOrigin = SkeletonTransforms[0].position;
            float3 characterForward = SkeletonTransforms[0].forward;

            if (DebugFutureSkeleton)
            {
                // Find Main Position Trajectory
                TrajectoryFeature feature = null;
                for (int i = 0; i < MMData.TrajectoryFeatures.Count; i++)
                {
                    if (MMData.TrajectoryFeatures[i].IsMainPositionFeature)
                    {
                        feature = MMData.TrajectoryFeatures[i];
                    }
                }
                if (feature != null)
                {
                    NativeArray<float3> worldPos = new(PoseSet.Skeleton.Joints.Count, Allocator.Temp);
                    for (int p = 0; p < feature.FramesPrediction.Length; p++)
                    {
                        Gizmos.color = Color.red + Color.cyan * ((float)p / feature.FramesPrediction.Length);
                        int frame = currentFrame + feature.FramesPrediction[p];
                        PoseSet.GetPose(frame, out PoseVector futurePose);
                        PoseSet.GetWorldPositions(futurePose, worldPos, InverseAnimationSpaceOriginRot, AnimationSpaceOriginPos, MMTransformOriginRot, MMTransformOriginPose);
                        for (int i = 2; i < PoseSet.Skeleton.Joints.Count; i++)
                        {
                            float3 child = worldPos[i];
                            float3 parent = worldPos[PoseSet.Skeleton.Joints[i].ParentIndex];
                            GizmosExtensions.DrawLine(parent, child, 3);
                        }
                    }
                    worldPos.Dispose();
                }
            }

            if (DebugCurrent)
            {
                Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
                Gizmos.DrawSphere(characterOrigin, SpheresRadius);
                GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward * 1.5f, thickness: 4);
            }

            if (DebugContacts)
            {
                Gizmos.color = Color.green;
                if (IsLeftFootContact)
                {
                    Gizmos.DrawSphere(SkeletonTransforms[LeftToesIndex].position, SpheresRadius);
                }
                if (IsRightFootContact)
                {
                    Gizmos.DrawSphere(SkeletonTransforms[RightToesIndex].position, SpheresRadius);
                }
            }

            // Feature Set
            if (FeatureSet == null) return;

            FeatureDebug.DrawFeatureGizmos(FeatureSet, MMData, SpheresRadius, currentFrame, characterOrigin, characterForward,
                                           SkeletonTransforms, PoseSet.Skeleton, Color.blue, DebugPose, DebugTrajectory, DebugEnvironment);

            if (DebugSearch)
            {
                SearchInstance.DrawGizmos(this, SpheresRadius);
            }
        }
#endif
    }
}