using System;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using System.Collections.Generic;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

    /// <summary>
    /// Stores all features vectors of all poses for Motion Matching
    /// </summary>
    public class FeatureSet
    {
        public int NumberFeatureVectors { get; private set; } // Total number of feature vectors
        public int FeatureSize { get; private set; } // Total size in floats of a feature vector
        public int FeatureStaticSize { get; private set; } // Total size in floats of a feature vector without dynamic features

        // Trajectory features
        public int NumberTrajectoryFeatures { get; private set; } // Number of different trajectory features (e.g. 2 = position and direction)
        private readonly int[] NumberPredictionsTrajectory; // Size: NumberTrajectoryFeatures. Number of predictions per trajectory feature (e.g. {3, 4}, means 3 predictions for position and 4 for direction)
        private readonly int[] NumberFloatsTrajectory; // Size: NumberTrajectoryFeatures. Number of floats per trajectory feature (e.g. {2, 3}, means 2 floats for position (float2) and 3 for direction (float3))
        private readonly int[] TrajectoryOffset; // Size: NumberTrajectoryFeatures. Offset of the trajectory feature in the feature vector (e.g. {0, 6}, means the position feature is at the beginning of the feature vector, and the direction feature starts at the sixth float)

        // Pose Features
        public int NumberPoseFeatures { get; private set; } // Number of different pose features (e.g. 3 = leftFootPosition, leftFootVelocity, hipsVelocity)
        private const int NumberFloatsPose = 3; // Number of floats per pose feature (e.g. 3 = float3)
        public int PoseOffset { get; private set; } // Offset of the pose feature in the feature vector (e.g. 3 = leftFootPosition is at the third float)
                                                    // Pose Features are always after the Trajectory Features

        // Dynamic
        public int NumberDynamicFeatures { get; private set; } // Number of dynamic features (e.g. 2 = spheres and ellipses)
        private readonly int[] NumberPredictionsDynamic; // Size: NumberDynamicFeatures. Number of predictions per dynamic feature (e.g. {3, 4}, means 3 predictions for spheres and 4 for ellipses)
        private readonly int[] NumberFloatsDynamic; // Size: NumberDynamicFeatures. Number of floats per dynamic feature (e.g. {1, 2}, means 1 floats for spheres (float) and 2 for ellipses (float2))
        public int[] DynamicOffset { get; private set; } // Size: NumberDynamicFeatures. Offset of the dynamic feature in the feature vector (e.g. 6 = spheres is at the sixth float)
                                                         // Dynamic Features are always after the Pose Features

        private NativeArray<bool> Valid; // TODO: Refactor to avoid needing this
        private NativeArray<float> Features; // Each feature: Trajectory + Pose + Dynamic 
        private float[] Mean; // Size: DynamicOffset[0]. Dynamic features are never normalized
        private float[] StandardDeviation; // Size: DynamicOffset[0]. Dynamic features are never normalized

        // BVH acceleration structures
        private NativeArray<float> LargeBoundingBoxMin;
        private NativeArray<float> LargeBoundingBoxMax;
        private NativeArray<float> SmallBoundingBoxMin;
        private NativeArray<float> SmallBoundingBoxMax;

        // Dynamic acceleration structures
        private NativeArray<int> AdaptativeFeaturesIndices; // Index to the real Features array

        public FeatureSet(MotionMatchingData mmData, int numberFeatureVectors)
        {
            NumberFeatureVectors = numberFeatureVectors;

            // Trajectory Features
            NumberTrajectoryFeatures = mmData.TrajectoryFeatures.Count;
            NumberPredictionsTrajectory = new int[NumberTrajectoryFeatures];
            NumberFloatsTrajectory = new int[NumberTrajectoryFeatures];
            TrajectoryOffset = new int[NumberTrajectoryFeatures];
            int offset = 0;
            for (int i = 0; i < NumberTrajectoryFeatures; i++)
            {
                TrajectoryOffset[i] = offset;
                NumberPredictionsTrajectory[i] = mmData.TrajectoryFeatures[i].FramesPrediction.Length;
                NumberFloatsTrajectory[i] = mmData.TrajectoryFeatures[i].GetSize();
                offset += NumberPredictionsTrajectory[i] * NumberFloatsTrajectory[i];
            }

            // Pose Features
            PoseOffset = offset;
            NumberPoseFeatures = mmData.PoseFeatures.Count;
            offset += NumberPoseFeatures * NumberFloatsPose; // + Pose

            FeatureStaticSize = offset;

            // Dynamic Features
            NumberDynamicFeatures = mmData.DynamicFeatures.Count;
            NumberPredictionsDynamic = new int[NumberDynamicFeatures];
            NumberFloatsDynamic = new int[NumberDynamicFeatures];
            DynamicOffset = new int[NumberDynamicFeatures];
            for (int i = 0; i < NumberDynamicFeatures; i++)
            {
                DynamicOffset[i] = offset;
                NumberPredictionsDynamic[i] = mmData.DynamicFeatures[i].FramesPrediction.Length;
                NumberFloatsDynamic[i] = mmData.DynamicFeatures[i].GetSize();
                offset += NumberPredictionsDynamic[i] * NumberFloatsDynamic[i];
            }

            FeatureSize = offset;
        }

        public bool IsValidFeature(int featureIndex)
        {
            return Valid[featureIndex];
        }

        public void GetFeature(NativeArray<float> feature, int featureIndex)
        {
            Debug.Assert(feature.Length == FeatureSize, "Feature vector has wrong size");
            for (int i = 0; i < FeatureSize; i++)
            {
                feature[i] = Features[featureIndex * FeatureSize + i];
            }
        }

        public float Get1DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            int featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
            }
            return x;
        }
        public float2 Get2DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            int featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            float y = Features[startIndex + 1];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
            }
            return new float2(x, y);
        }
        public float3 Get3DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            int featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            float y = Features[startIndex + 1];
            float z = Features[startIndex + 2];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
                z = z * StandardDeviation[featureOffset + 2] + Mean[featureOffset + 2];
            }
            return new float3(x, y, z);
        }
        public float4 Get4DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            int featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            float y = Features[startIndex + 1];
            float z = Features[startIndex + 2];
            float w = Features[startIndex + 3];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
                z = z * StandardDeviation[featureOffset + 2] + Mean[featureOffset + 2];
                w = w * StandardDeviation[featureOffset + 3] + Mean[featureOffset + 3];
            }
            return new float4(x, y, z, w);
        }
        public float3 GetPoseFeature(int featureIndex, int poseFeatureIndex, bool denormalize = false)
        {
            int featureOffset = PoseOffset + poseFeatureIndex * NumberFloatsPose;
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            float y = Features[startIndex + 1];
            float z = Features[startIndex + 2];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
                z = z * StandardDeviation[featureOffset + 2] + Mean[featureOffset + 2];
            }
            return new float3(x, y, z);
        }
        public float Get1DDynamicFeature(int featureIndex, int dynamicFeatureIndex, int predictionIndex)
        {
            int featureOffset = DynamicOffset[dynamicFeatureIndex] + predictionIndex * NumberFloatsDynamic[dynamicFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            return x;
        }
        public float2 Get2DDynamicFeature(int featureIndex, int dynamicFeatureIndex, int predictionIndex)
        {
            int featureOffset = DynamicOffset[dynamicFeatureIndex] + predictionIndex * NumberFloatsDynamic[dynamicFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            float y = Features[startIndex + 1];
            return new float2(x, y);
        }
        public float3 Get3DDynamicFeature(int featureIndex, int dynamicFeatureIndex, int predictionIndex)
        {
            int featureOffset = DynamicOffset[dynamicFeatureIndex] + predictionIndex * NumberFloatsDynamic[dynamicFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            float y = Features[startIndex + 1];
            float z = Features[startIndex + 2];
            return new float3(x, y, z);
        }
        public float4 Get4DDynamicFeature(int featureIndex, int dynamicFeatureIndex, int predictionIndex)
        {
            int featureOffset = DynamicOffset[dynamicFeatureIndex] + predictionIndex * NumberFloatsDynamic[dynamicFeatureIndex];
            int startIndex = featureIndex * FeatureSize + featureOffset;
            float x = Features[startIndex];
            float y = Features[startIndex + 1];
            float z = Features[startIndex + 2];
            float w = Features[startIndex + 3];
            return new float4(x, y, z, w);
        }

        public NativeArray<bool> GetValid()
        {
            return Valid;
        }
        public NativeArray<float> GetFeatures()
        {
            return Features;
        }

        public float GetMean(int dimension)
        {
            return Mean[dimension];
        }
        public float[] GetMeans()
        {
            return Mean;
        }
        public float GetStandardDeviation(int dimension)
        {
            return StandardDeviation[dimension];
        }
        public float[] GetStandardDeviations()
        {
            return StandardDeviation;
        }

        public void GetBVHBuffers(out NativeArray<float> largeBoundingBoxMin,
                                  out NativeArray<float> largeBoundingBoxMax,
                                  out NativeArray<float> smallBoundingBoxMin,
                                  out NativeArray<float> smallBoundingBoxMax)
        {
            if (LargeBoundingBoxMax == null || !LargeBoundingBoxMax.IsCreated)
            {
                // Build BVH Acceleration Structure
                int nFrames = GetFeatures().Length / FeatureSize;
                int numberBoundingBoxLarge = (nFrames + BVHConsts.LargeBVHSize - 1) / BVHConsts.LargeBVHSize;
                int numberBoundingBoxSmall = (nFrames + BVHConsts.SmallBVHSize - 1) / BVHConsts.SmallBVHSize;
                LargeBoundingBoxMin = new NativeArray<float>(numberBoundingBoxLarge * FeatureStaticSize, Allocator.Persistent);
                LargeBoundingBoxMax = new NativeArray<float>(numberBoundingBoxLarge * FeatureStaticSize, Allocator.Persistent);
                SmallBoundingBoxMin = new NativeArray<float>(numberBoundingBoxSmall * FeatureStaticSize, Allocator.Persistent);
                SmallBoundingBoxMax = new NativeArray<float>(numberBoundingBoxSmall * FeatureStaticSize, Allocator.Persistent);
                var job = new BVHMotionMatchingComputeBounds
                {
                    Features = GetFeatures(),
                    FeatureSize = FeatureSize,
                    FeatureStaticSize = FeatureStaticSize,
                    NumberBoundingBoxLarge = numberBoundingBoxLarge,
                    NumberBoundingBoxSmall = numberBoundingBoxSmall,
                    LargeBoundingBoxMin = LargeBoundingBoxMin,
                    LargeBoundingBoxMax = LargeBoundingBoxMax,
                    SmallBoundingBoxMin = SmallBoundingBoxMin,
                    SmallBoundingBoxMax = SmallBoundingBoxMax,
                };
                job.Schedule().Complete();
            }
            largeBoundingBoxMin = LargeBoundingBoxMin;
            largeBoundingBoxMax = LargeBoundingBoxMax;
            smallBoundingBoxMin = SmallBoundingBoxMin;
            smallBoundingBoxMax = SmallBoundingBoxMax;
        }

        public void GetDynamicAccelerationStructures(DynamicAccelerationConsts consts, 
                                                     out NativeArray<int> adaptativeFeaturesIndices)
        {
            if (AdaptativeFeaturesIndices == null || !AdaptativeFeaturesIndices.IsCreated)
            {
                // Build Dynamic Acceleration Structure
                NativeList<int> adaptativeIndices = new(Allocator.Persistent);
                var job = new DynamicAccelerationComputeAdaptativeIndices
                {
                    Features = GetFeatures(),
                    Valid = GetValid(),
                    FeatureSize = FeatureSize,
                    PoseOffset = PoseOffset,
                    DynamicAccelerationConsts = consts,
                    FeatureStaticSize = FeatureStaticSize,
                    AdaptativeIndices = adaptativeIndices,
                };
                job.Schedule().Complete();
                AdaptativeFeaturesIndices = adaptativeIndices.AsArray();
            }
            Debug.Log("Number of features: " + GetFeatures().Length / FeatureSize + " -----  Adaptative Length: " + AdaptativeFeaturesIndices.Length);
            adaptativeFeaturesIndices = AdaptativeFeaturesIndices;
        }

        // Deserialize ---------------------------------------
        public void SetValid(NativeArray<bool> valid)
        {
            Debug.Assert(valid.Length == NumberFeatureVectors, "Valid array has wrong size");
            if (Valid != null && Valid.IsCreated)
            {
                Valid.Dispose();
            }
            Valid = valid;
        }
        public void SetFeatures(NativeArray<float> features)
        {
            Debug.Assert(features.Length == NumberFeatureVectors * FeatureSize, "Feature vector has wrong size");
            if (Features != null && Features.IsCreated)
            {
                Features.Dispose();
            }
            Features = features;
        }
        public void SetMean(float[] mean)
        {
            Debug.Assert(mean.Length == FeatureStaticSize, mean.Length + " != " + FeatureStaticSize);
            Mean = mean;
        }
        public void SetStandardDeviation(float[] standardDeviation)
        {
            Debug.Assert(standardDeviation.Length == FeatureStaticSize, standardDeviation.Length + " != " + FeatureStaticSize);
            StandardDeviation = standardDeviation;
        }
        // --------------------------------------------------

        /// <summary>
        /// Normalizes the trajectory features (pose features remaing untouched)
        /// </summary>
        public void NormalizeTrajectory(NativeArray<float> featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");
            Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

            for (int i = 0; i < PoseOffset; i++)
            {
                featureVector[i] = (featureVector[i] - Mean[i]) / StandardDeviation[i];
            }
        }

        /// <summary>
        /// Normalizes all features (trajectory + pose)
        /// </summary>
        public void NormalizeFeatureVector(NativeArray<float> featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");
            Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

            for (int i = 0; i < FeatureStaticSize; i++)
            {
                featureVector[i] = (featureVector[i] - Mean[i]) / StandardDeviation[i];
            }
        }

        /// <summary>
        /// Returns a copy of the feature vector with the features before normalization
        /// </summary>
        public void DenormalizeFeatureVector(NativeArray<float> featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");
            Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

            for (int i = 0; i < FeatureStaticSize; i++)
            {
                featureVector[i] = featureVector[i] * StandardDeviation[i] + Mean[i];
            }
        }

        /// <summary>
        /// Normalizes the features by subtracting mean and dividing by the standard deviation
        /// </summary>
        public void NormalizeFeatures()
        {
            // Compute Mean and Standard Deviation
            ComputeMeanAndStandardDeviation();

            // Normalize all feature vectors
            for (int i = 0; i < NumberFeatureVectors; i++)
            {
                int featureIndex = i * FeatureSize;
                if (Valid[i])
                {
                    for (int j = 0; j < FeatureStaticSize; j++)
                    {
                        Features[featureIndex + j] = (Features[featureIndex + j] - Mean[j]) / StandardDeviation[j];
                    }
                }
            }
        }

        private void ComputeMeanAndStandardDeviation()
        {
            int nTotalDimensions = FeatureStaticSize;
            // Mean for each dimension
            Mean = new float[nTotalDimensions];
            // Variance for each dimension
            Span<float> variance = stackalloc float[nTotalDimensions];
            // Standard Deviation for each dimension
            StandardDeviation = new float[nTotalDimensions];

            // Compute Means for each dimension of each feature
            int count = 0;
            for (int i = 0; i < NumberFeatureVectors; i++)
            {
                if (Valid[i])
                {
                    int featureIndex = i * FeatureSize;
                    for (int j = 0; j < nTotalDimensions; j++)
                    {
                        Mean[j] += Features[featureIndex + j];
                    }
                    count += 1;
                }
            }
            for (int i = 0; i < nTotalDimensions; i++)
            {
                Mean[i] /= count;
            }
            // Compute Variance for each dimension of each feature - variance = (x - mean)^2 / n
            for (int i = 0; i < NumberFeatureVectors; i++)
            {
                int featureIndex = i * FeatureSize;
                if (Valid[i])
                {
                    for (int j = 0; j < nTotalDimensions; j++)
                    {
                        float diff = Features[featureIndex + j] - Mean[j];
                        variance[j] += diff * diff;
                    }
                }
            }
            for (int i = 0; i < nTotalDimensions; i++)
            {
                variance[i] /= count;
            }

            // Compute Standard Deviations of a feature as the average std across all dimensions - std = sqrt(variance)
            for (int d = 0; d < NumberTrajectoryFeatures; d++)
            {
                int offset = TrajectoryOffset[d];
                int nDimensions = NumberPredictionsTrajectory[d] * NumberFloatsTrajectory[d];
                float std = 0;
                for (int j = 0; j < nDimensions; j++)
                {
                    std += math.sqrt(variance[offset + j]);
                }
                std /= nDimensions;
                Debug.Assert(std > 0, "Standard deviation is zero, feature with no variation is probably a bug");
                if (std <= 0)
                {
                    std = 1.0f;
                }
                for (int j = 0; j < nDimensions; j++)
                {
                    StandardDeviation[offset + j] = std;
                }
            }
            for (int d = 0; d < NumberPoseFeatures; d++)
            {
                int offset = PoseOffset + d * NumberFloatsPose;
                float std = 0;
                for (int j = 0; j < NumberFloatsPose; j++)
                {
                    std += math.sqrt(variance[offset + j]);
                }
                std /= NumberFloatsPose;
                Debug.Assert(std > 0, "Standard deviation is zero, feature with no variation is probably a bug");
                if (std <= 0)
                {
                    std = 1.0f;
                }
                for (int j = 0; j < NumberFloatsPose; j++)
                {
                    StandardDeviation[offset + j] = std;
                }
            }
        }

        /// <summary>
        /// Extract the feature vectors from poseSet
        /// </summary>
        public void Extract(PoseSet poseSet, MotionMatchingData mmData)
        {
            // Init
            int nPoses = poseSet.NumberPoses;
            Valid = new NativeArray<bool>(nPoses, Allocator.Persistent);
            Features = new NativeArray<float>(nPoses * FeatureSize, Allocator.Persistent);
            // Check skeleton has all needed joints
            Joint[] jointsTrajectory = new Joint[NumberTrajectoryFeatures];
            int i = 0;
            foreach (var trajectoryFeature in mmData.TrajectoryFeatures)
            {
                if ((trajectoryFeature.FeatureType == TrajectoryFeature.Type.Position ||
                    trajectoryFeature.FeatureType == TrajectoryFeature.Type.Direction)
                    && !trajectoryFeature.SimulationBone)
                {
                    if (!poseSet.Skeleton.Find(trajectoryFeature.Bone, out jointsTrajectory[i])) Debug.Assert(false, "The skeleton does not contain any joint of type " + trajectoryFeature.Bone);
                }
                i += 1;
            }
            Joint[] jointsPose = new Joint[NumberPoseFeatures];
            i = 0;
            foreach (var poseFeature in mmData.PoseFeatures)
            {
                if (!poseSet.Skeleton.Find(poseFeature.Bone, out jointsPose[i])) Debug.Assert(false, "The skeleton does not contain any joint of type " + poseFeature.Bone);
                i += 1;
            }
            Joint[] jointsDynamic = new Joint[NumberDynamicFeatures];
            i = 0;
            foreach (var dynamicFeature in mmData.DynamicFeatures)
            {
                if ((dynamicFeature.FeatureType == TrajectoryFeature.Type.Position ||
                    dynamicFeature.FeatureType == TrajectoryFeature.Type.Direction)
                    && !dynamicFeature.SimulationBone)
                {
                    if (!poseSet.Skeleton.Find(dynamicFeature.Bone, out jointsDynamic[i])) Debug.Assert(false, "The skeleton does not contain any joint of type " + dynamicFeature.Bone);
                }
                i += 1;
            }
            // Extract Features
            for (int poseIndex = 0; poseIndex < nPoses; ++poseIndex)
            {
                if (poseSet.IsPoseValidForPrediction(poseIndex))
                {
                    Valid[poseIndex] = true;
                    ExtractFeature(poseSet, poseIndex, jointsTrajectory, jointsPose, jointsDynamic, mmData);
                }
                else Valid[poseIndex] = false;
            }
        }

        /// <summary>
        /// Extract the feature vectors from poseSet
        /// </summary>
        private void ExtractFeature(PoseSet poseSet, int poseIndex, Joint[] jointsTrajectory, Joint[] jointsPose, Joint[] jointsDynamic, MotionMatchingData mmData)
        {
            int featureIndex = poseIndex * FeatureSize;
            int nextPose = poseIndex + 1;
            if (nextPose >= poseSet.NumberPoses - 60) // HARDCODED
            {
                nextPose = poseIndex;
            }
            int nextFeatureIndex = nextPose * FeatureSize;
            poseSet.GetPose(poseIndex, out PoseVector pose);
            poseSet.GetPose(nextPose, out PoseVector poseNext);
            // Compute local features based on the Simulation Bone
            // so hips and feet are local to a stable position with respect to the character
            GetWorldOriginCharacter(pose, out float3 characterOrigin, out float3 characterForward);

            // Trajectory Features -------------------------------------------------------------
            for (int i = 0; i < NumberTrajectoryFeatures; i++)
            {
                TrajectoryFeature trajectoryFeature = mmData.TrajectoryFeatures[i];
                int featureOffset = featureIndex + TrajectoryOffset[i];
                int nextFeatureOffset = nextFeatureIndex + TrajectoryOffset[i];
                bool isStartFeature = true;
                for (int p = 0; p < trajectoryFeature.FramesPrediction.Length; ++p)
                {
                    int predictionOffset = featureOffset + p * NumberFloatsTrajectory[i];
                    int nextPredictionOffset = nextFeatureOffset + p * NumberFloatsTrajectory[i];
                    int futurePoseIndex = poseIndex + trajectoryFeature.FramesPrediction[p];
                    int nextFuturePoseIndex = nextPose + trajectoryFeature.FramesPrediction[p];

                    isStartFeature = ExtractTrajectoryFeature(trajectoryFeature, poseSet, futurePoseIndex, nextFuturePoseIndex,
                                                              jointsTrajectory, i, predictionOffset, characterOrigin, characterForward,
                                                              mmData, isStartFeature);
                }
            }

            // Pose Features -------------------------------------------------------------
            for (int i = 0; i < NumberPoseFeatures; i++)
            {
                MotionMatchingData.PoseFeature poseFeature = mmData.PoseFeatures[i];
                int featureOffset = featureIndex + PoseOffset + i * NumberFloatsPose;
                float3 feature = new();
                switch (poseFeature.FeatureType)
                {
                    case MotionMatchingData.PoseFeature.Type.Position:
                        GetJointPosition(pose, poseSet.Skeleton, jointsPose[i], characterOrigin, characterForward,
                                         out feature);
                        break;
                    case MotionMatchingData.PoseFeature.Type.Velocity:
                        GetJointVelocity(pose, poseNext, poseSet.Skeleton, jointsPose[i], characterOrigin, characterForward, poseSet.FrameTime,
                                         out feature);
                        break;
                    default:
                        Debug.Assert(false, "Unknown PoseFeature.Type: " + poseFeature.FeatureType);
                        break;
                }
                Features[featureOffset + 0] = feature.x;
                Features[featureOffset + 1] = feature.y;
                Features[featureOffset + 2] = feature.z;
            }

            // Dynamic Features -------------------------------------------------------------
            for (int i = 0; i < NumberDynamicFeatures; i++)
            {
                TrajectoryFeature dynamicFeature = mmData.DynamicFeatures[i];
                int featureOffset = featureIndex + DynamicOffset[i];
                int nextFeatureOffset = nextFeatureIndex + DynamicOffset[i];
                bool isStartFeature = true;
                for (int p = 0; p < dynamicFeature.FramesPrediction.Length; ++p)
                {
                    int predictionOffset = featureOffset + p * NumberFloatsDynamic[i];
                    int nextPredictionOffset = nextFeatureOffset + p * NumberFloatsDynamic[i];
                    int futurePoseIndex = poseIndex + dynamicFeature.FramesPrediction[p];
                    int nextFuturePoseIndex = nextPose + dynamicFeature.FramesPrediction[p];

                    isStartFeature = ExtractTrajectoryFeature(dynamicFeature, poseSet, futurePoseIndex, nextFuturePoseIndex,
                                                              jointsDynamic, i, predictionOffset, characterOrigin, characterForward,
                                                              mmData, isStartFeature);
                }
            }
        }

        private bool ExtractTrajectoryFeature(TrajectoryFeature feature, PoseSet poseSet, int futurePoseIndex, int nextFuturePoseIndex,
                                              Joint[] joints, int featureIt, int predictionOffset, float3 characterOrigin, float3 characterForward,
                                              MotionMatchingData mmData, bool isStartFeature)
        {
            poseSet.GetPose(futurePoseIndex, out PoseVector futurePose, out int animationClip);
            poseSet.GetPose(nextFuturePoseIndex, out PoseVector nextFuturePose, out int nextAnimationClip);

            float3 value = new();
            switch (feature.FeatureType)
            {
                case TrajectoryFeature.Type.Position:
                    {
                        GetTrajectoryPosition(futurePose, poseSet.Skeleton, feature.SimulationBone, joints[featureIt], characterOrigin, characterForward,
                                              out value);
                    }
                    break;
                case TrajectoryFeature.Type.Direction:
                    {
                        GetTrajectoryDirection(futurePose, poseSet.Skeleton, feature.SimulationBone, joints[featureIt], characterForward, mmData,
                                               out value);
                        if (feature.ZeroX) value.x = 0;
                        if (feature.ZeroY) value.y = 0;
                        if (feature.ZeroZ) value.z = 0;
                        value = math.normalize(value);
                    }
                    break;
                case TrajectoryFeature.Type.Custom1D:
                    {
                        Feature1DExtractor extractor1D = feature.FeatureExtractor as Feature1DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor1D.StartExtracting(poseSet.Skeleton);
                        }
                        float value1D = extractor1D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value1D;
                    }
                    break;
                case TrajectoryFeature.Type.Custom2D:
                    {
                        Feature2DExtractor extractor2D = feature.FeatureExtractor as Feature2DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor2D.StartExtracting(poseSet.Skeleton);
                        }
                        float2 value2D = extractor2D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value2D.x;
                        Features[predictionOffset + 1] = value2D.y;
                    }
                    break;
                case TrajectoryFeature.Type.Custom3D:
                    {
                        Feature3DExtractor extractor3D = feature.FeatureExtractor as Feature3DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor3D.StartExtracting(poseSet.Skeleton);
                        }
                        float3 value3D = extractor3D.ExtractFeature(futurePose, futurePoseIndex, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value3D.x;
                        Features[predictionOffset + 1] = value3D.y;
                        Features[predictionOffset + 2] = value3D.z;
                    }
                    break;
                case TrajectoryFeature.Type.Custom4D:
                    {
                        Feature4DExtractor extractor4D = feature.FeatureExtractor as Feature4DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor4D.StartExtracting(poseSet.Skeleton);
                        }
                        float4 value4D = extractor4D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value4D.x;
                        Features[predictionOffset + 1] = value4D.y;
                        Features[predictionOffset + 2] = value4D.z;
                        Features[predictionOffset + 3] = value4D.w;
                    }
                    break;
                default:
                    Debug.Assert(false, "Unsupported Feature Type: " + feature.FeatureType);
                    break;
            }
            if (feature.FeatureType == TrajectoryFeature.Type.Position ||
                feature.FeatureType == TrajectoryFeature.Type.Direction)
            {
                int offsetIndex = 0;
                int valueIndex = 0;
                int size = feature.GetSize();
                for (int f = 0; f < size; ++f)
                {
                    if (valueIndex == 0 && feature.ZeroX) valueIndex += 1;
                    if (valueIndex == 1 && feature.ZeroY) valueIndex += 1;
                    Features[predictionOffset + offsetIndex] = value[valueIndex];
                    valueIndex += 1;
                    offsetIndex += 1;
                }
            }
            return isStartFeature;
        }

        private static void GetTrajectoryPosition(PoseVector pose, Skeleton skeleton, bool simulationBone, Joint joint, float3 characterOrigin, float3 characterForward,
                                                  out float3 futureLocalPosition)
        {
            float3 worldPosition;
            if (simulationBone)
            {
                worldPosition = pose.JointLocalPositions[0];
            }
            else
            {
                worldPosition = GetWorldPosition(skeleton, pose, joint);
            }
            futureLocalPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
        }
        private static void GetTrajectoryDirection(PoseVector pose, Skeleton skeleton, bool simulationBone, Joint joint, float3 characterForward, MotionMatchingData mmData,
                                                   out float3 futureLocalDirection)
        {
            quaternion worldRotation;
            float3 localForward;
            if (simulationBone)
            {
                worldRotation = pose.JointLocalRotations[0];
                localForward = math.forward();
            }
            else
            {
                worldRotation = GetWorldRotation(skeleton, pose, joint);
                localForward = mmData.GetLocalForward(joint.Index);
            }
            float3 worldDirection = math.mul(worldRotation, localForward);
            futureLocalDirection = GetLocalDirectionFromCharacter(worldDirection, characterForward);
        }

        private static void GetJointPosition(PoseVector pose, Skeleton skeleton, Joint joint, float3 characterOrigin, float3 characterForward,
                                             out float3 localPosition)
        {
            float3 worldPosition = GetWorldPosition(skeleton, pose, joint);
            localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
        }
        private static void GetJointVelocity(PoseVector pose, PoseVector poseNext, Skeleton skeleton, Joint joint, float3 characterOrigin, float3 characterForward, float frameTime,
                                             out float3 localVelocity)
        {
            float3 worldPosition = GetWorldPosition(skeleton, pose, joint);
            float3 worldPositionNext = GetWorldPosition(skeleton, poseNext, joint);
            float3 localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            localVelocity = (GetLocalPositionFromCharacter(worldPositionNext, characterOrigin, characterForward) - localPosition) / frameTime;
        }

        /// <summary>
        /// Returns the position of the joint in world space after applying FK using the pose
        /// </summary>
        public static float3 GetWorldPosition(Skeleton skeleton, PoseVector pose, Joint joint)
        {
            Matrix4x4 localToWorld = Matrix4x4.identity;
            while (joint.Index != 0) // while not root
            {
                localToWorld = Matrix4x4.TRS(pose.JointLocalPositions[joint.Index], pose.JointLocalRotations[joint.Index], new float3(1.0f, 1.0f, 1.0f)) * localToWorld;
                joint = skeleton.GetParent(joint);
            }
            localToWorld = Matrix4x4.TRS(pose.JointLocalPositions[0], pose.JointLocalRotations[0], new float3(1.0f, 1.0f, 1.0f)) * localToWorld; // root
            return localToWorld.MultiplyPoint3x4(Vector3.zero);
        }

        /// <summary>
        /// Returns the rotation of the joint in world space after applying FK using the pose
        /// </summary>
        public static quaternion GetWorldRotation(Skeleton skeleton, PoseVector pose, Joint joint)
        {
            quaternion worldRot = quaternion.identity;
            while (joint.Index != 0) // while not root
            {
                worldRot = math.mul(pose.JointLocalRotations[joint.Index], worldRot);
                joint = skeleton.GetParent(joint);
            }
            worldRot = math.mul(pose.JointLocalRotations[0], worldRot); // root
            return worldRot;
        }

        /// <summary>
        /// Returns the position and forward vector of the character in world space using the pose vector simulation bone
        /// </summary>
        /// <param name="hipsForwardLocalVector">forward vector of the hips in world space when in bind pose</param>
        public static void GetWorldOriginCharacter(PoseVector poseVector, out float3 center, out float3 forward)
        {
            center = poseVector.JointLocalPositions[0]; // Simulation Bone World Position
            forward = math.mul(poseVector.JointLocalRotations[0], math.forward()); // Simulation Bone World Rotation
        }

        public static float3 GetLocalPositionFromCharacter(float3 worldPos, float3 characterOrigin, float3 characterForward)
        {
            return math.mul(math.inverse(quaternion.LookRotation(characterForward, math.up())), worldPos - characterOrigin);
        }

        public static float3 GetLocalDirectionFromCharacter(float3 worldDir, float3 characterForward)
        {
            float3 localDir = math.mul(math.inverse(quaternion.LookRotation(characterForward, math.up())), worldDir);
            return localDir;
        }
        public static float3 GetWorldPositionFromCharacter(float3 localPos, float3 characterOrigin, float3 characterForward)
        {
            return characterOrigin + math.mul(quaternion.LookRotation(characterForward, math.up()), localPos);
        }
        public static float3 GetWorldDirectionFromCharacter(float3 localDir, float3 characterForward)
        {
            return math.mul(quaternion.LookRotation(characterForward, math.up()), localDir);
        }

        public void Dispose()
        {
            if (Valid != null && Valid.IsCreated) Valid.Dispose();
            if (Features != null && Features.IsCreated) Features.Dispose();
            if (LargeBoundingBoxMin != null && LargeBoundingBoxMin.IsCreated) LargeBoundingBoxMin.Dispose();
            if (LargeBoundingBoxMax != null && LargeBoundingBoxMax.IsCreated) LargeBoundingBoxMax.Dispose();
            if (SmallBoundingBoxMin != null && SmallBoundingBoxMin.IsCreated) SmallBoundingBoxMin.Dispose();
            if (SmallBoundingBoxMax != null && SmallBoundingBoxMax.IsCreated) SmallBoundingBoxMax.Dispose();
            if (AdaptativeFeaturesIndices != null && AdaptativeFeaturesIndices.IsCreated) AdaptativeFeaturesIndices.Dispose();
        }
    }
}