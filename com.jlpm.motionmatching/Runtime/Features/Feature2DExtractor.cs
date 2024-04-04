using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    public abstract class Feature2DExtractor : ScriptableObject
    {
        /// <summary>
        /// Called once at the beginning of the feature extraction process.
        /// </summary
        public abstract void StartExtracting(Skeleton skeleton);
        /// <summary>
        /// Called for each pose in the motion clip.
        /// StartExtracting(...) is always called once before ExtractFeature(...) is called multiple times.
        /// </summary
        public abstract float2 ExtractFeature(PoseVector pose, int poseIndex, int animationClip, Skeleton skeleton, float3 characterOrigin, float3 characterForward);
        /// <summary>
        /// Called when Gizmos are drawn to debug features
        /// StartExtracting(...) is not called before DrawGizmos(...)
        /// </summary>
        public abstract void DrawGizmos(float2 feature, float radius,
                                        float3 characterOrigin, float3 characterForward,
                                        Transform[] joints, Skeleton skeleton);
    }
}
