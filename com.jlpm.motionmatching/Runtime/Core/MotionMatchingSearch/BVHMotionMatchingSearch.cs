using UnityEngine;
using Unity.Collections;
using Unity.Jobs;

namespace MotionMatching
{
    [CreateAssetMenu(fileName = "BVHMotionMatchingSearch", menuName = "MotionMatching/Search/BVH Motion Matching Search")]
    public class BVHMotionMatchingSearch : MotionMatchingSearch
    {
        private NativeArray<int> SearchResult;
        private bool IsDisposed = false;

        // Acceleration Structure
        private NativeArray<float> LargeBoundingBoxMin;
        private NativeArray<float> LargeBoundingBoxMax;
        private NativeArray<float> SmallBoundingBoxMin;
        private NativeArray<float> SmallBoundingBoxMax;

        public override void Initialize(MotionMatchingController controller)
        {
            controller.FeatureSet.GetBVHBuffers(out LargeBoundingBoxMin,
                                                out LargeBoundingBoxMax,
                                                out SmallBoundingBoxMin,
                                                out SmallBoundingBoxMax);

            SearchResult = new NativeArray<int>(2, Allocator.Persistent);
            SearchResult[0] = 0;
            SearchResult[1] = 0;

            IsDisposed = false;
        }

        public override bool ShouldSearch(MotionMatchingController controller)
        {
            return controller.SearchTimeLeft <= 0;
        }

        public override int FindBestFrame(MotionMatchingController controller, float currentDistance)
        {
            if (IsDisposed) return controller.CurrentFrame;

            var job = new BVHMotionMatchingSearchBurst
            {
                Valid = controller.FeatureSet.GetValid(),
                TagMask = controller.TagMask,
                Features = controller.FeatureSet.GetFeatures(),
                QueryFeature = controller.QueryFeature,
                FeatureWeights = controller.FeaturesWeightsNativeArray,
                FeatureSize = controller.FeatureSet.FeatureSize,
                FeatureStaticSize = controller.FeatureSet.FeatureStaticSize,
                PoseOffset = controller.FeatureSet.PoseOffset,
                CurrentDistance = currentDistance,
                LargeBoundingBoxMin = LargeBoundingBoxMin,
                LargeBoundingBoxMax = LargeBoundingBoxMax,
                SmallBoundingBoxMin = SmallBoundingBoxMin,
                SmallBoundingBoxMax = SmallBoundingBoxMax,
                BestIndex = SearchResult
            };
            job.Schedule().Complete();

            return SearchResult[0];
        }

        public override void OnSearchCompleted(MotionMatchingController controller) { }

        public override void OnEnabled() { }

        public override void OnDisabled() { }

        public override void Dispose()
        {
            if (SearchResult != null && SearchResult.IsCreated) SearchResult.Dispose();
            IsDisposed = true;
        }
    }
}
