using Unity.Collections;
using UnityEngine;
using static MotionMatching.MotionMatchingData;

namespace MotionMatching
{
    public abstract class MotionMatchingSearch : ScriptableObject
    {
        public virtual void Initialize(MotionMatchingController controller) { }
        public virtual void OnEnabled() { }
        public virtual void OnDisabled() { }
        public abstract bool ShouldSearch(MotionMatchingController controller);
        public abstract int FindBestFrame(MotionMatchingController controller, float currentDistance);
        public virtual void OnSearchCompleted(MotionMatchingController controller) { }
        public virtual float OnUpdateEnvironmentFeatureWeight(MotionMatchingController controller, TrajectoryFeature environmentFeature, float defaultWeight)
        {
            return defaultWeight;
        }
        public virtual void Dispose() { }
        public virtual void DrawGizmos(MotionMatchingController controller, float radius) { }
    }
}