using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    public interface IFeatureExtractor1D
    {
        /// <summary>
        /// Called once at the beginning of the feature extraction process.
        /// </summary
        public void StartExtracting(Skeleton skeleton);
        /// <summary>
        /// Called for each pose in the motion clip.
        /// StartExtracting(...) is always called once before ExtractFeature(...) is called multiple times.
        /// </summary
        public float ExtractFeature(PoseVector pose, int poseIndex, Skeleton skeleton, float3 characterOrigin, float3 characterForward);
        /// <summary>
        /// Called when Gizmos are drawn to debug features
        /// StartExtracting(...) is not called before DrawGizmos(...)
        /// </summary>
        public void DrawGizmos(float feature, float radius,
                               float3 characterOrigin, float3 characterForward,
                               Transform[] joints, Skeleton skeleton);
    }
    public interface IFeatureExtractor2D
    {
        /// <summary>
        /// Called once at the beginning of the feature extraction process.
        /// </summary
        public void StartExtracting(Skeleton skeleton);
        /// <summary>
        /// Called for each pose in the motion clip.
        /// StartExtracting(...) is always called once before ExtractFeature(...) is called multiple times.
        /// </summary
        public float2 ExtractFeature(PoseVector pose, int poseIndex, Skeleton skeleton, float3 characterOrigin, float3 characterForward);
        /// <summary>
        /// Called when Gizmos are drawn to debug features
        /// StartExtracting(...) is not called before DrawGizmos(...)
        /// </summary>
        public void DrawGizmos(float2 feature, float radius,
                               float3 characterOrigin, float3 characterForward,
                               Transform[] joints, Skeleton skeleton);
    }
    public interface IFeatureExtractor3D
    {
        /// <summary>
        /// Called once at the beginning of the feature extraction process.
        /// </summary
        public void StartExtracting(Skeleton skeleton);
        /// <summary>
        /// Called for each pose in the motion clip.
        /// StartExtracting(...) is always called once before ExtractFeature(...) is called multiple times.
        /// </summary
        public float3 ExtractFeature(PoseVector pose, int poseIndex, Skeleton skeleton, float3 characterOrigin, float3 characterForward);
        /// <summary>
        /// Called when Gizmos are drawn to debug features
        /// StartExtracting(...) is not called before DrawGizmos(...)
        /// </summary>
        public void DrawGizmos(float3 feature, float radius,
                               float3 characterOrigin, float3 characterForward,
                               Transform[] joints, Skeleton skeleton);
    }
    public interface IFeatureExtractor4D
    {
        /// <summary>
        /// Called once at the beginning of the feature extraction process.
        /// </summary
        public void StartExtracting(Skeleton skeleton);
        /// <summary>
        /// Called for each pose in the motion clip.
        /// StartExtracting(...) is always called once before ExtractFeature(...) is called multiple times.
        /// </summary
        public float4 ExtractFeature(PoseVector pose, int poseIndex, Skeleton skeleton, float3 characterOrigin, float3 characterForward);
        /// <summary>
        /// Called when Gizmos are drawn to debug features
        /// StartExtracting(...) is not called before DrawGizmos(...)
        /// </summary>
        public void DrawGizmos(float4 feature, float radius,
                               float3 characterOrigin, float3 characterForward,
                               Transform[] joints, Skeleton skeleton);
    }
}