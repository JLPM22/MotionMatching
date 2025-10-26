using Unity.Collections;
using UnityEngine;
using Unity.Jobs;

namespace MotionMatching
{
    [CreateAssetMenu(fileName = "LinearMotionMatchingSearch", menuName = "MotionMatching/Search/Linear Motion Matching Search")]
    public class LinearMotionMatchingSearch : MotionMatchingSearch
    {
        private NativeArray<int> SearchResult;
        private bool IsDisposed = false;

        public override void Initialize(MotionMatchingController controller)
        {
            SearchResult = new NativeArray<int>(2, Allocator.Persistent);
            SearchResult[0] = 0;
            SearchResult[1] = 0;

            IsDisposed = false;
        }

        public override void OnEnabled() { }

        public override void OnDisabled() { }

        public override bool ShouldSearch(MotionMatchingController controller)
        {
            return controller.SearchTimeLeft <= 0;
        }

        public override int FindBestFrame(MotionMatchingController controller, float currentDistance)
        {
            if (IsDisposed) return controller.CurrentFrame;

            var job = new LinearMotionMatchingSearchBurst
            {
                Valid = controller.FeatureSet.GetValid(),
                TagMask = controller.TagMask,
                Features = controller.FeatureSet.GetFeatures(),
                QueryFeature = controller.QueryFeature,
                FeatureWeights = controller.FeaturesWeightsNativeArray,
                FeatureSize = controller.FeatureSet.FeatureSize,
                FeatureStaticSize = controller.FeatureSet.FeatureStaticSize,
                CurrentDistance = currentDistance,
                BestIndex = SearchResult
            };
            job.Schedule().Complete();

            return SearchResult[0];
        }

        public override void OnSearchCompleted(MotionMatchingController controller) { }

        public override void Dispose()
        {
            if (SearchResult != null && SearchResult.IsCreated) SearchResult.Dispose();
            IsDisposed = true;
        }
    }
}