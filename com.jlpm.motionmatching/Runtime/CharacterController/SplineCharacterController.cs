using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

    public class SplineCharacterController : MotionMatchingCharacterController
    {
        public string TrajectoryPositionFeatureName = "FuturePosition";
        public string TrajectoryDirectionFeatureName = "FutureDirection";

        public SplineContainer SplineContainer;
        public float Speed = 1.0f;

        private float T;

        private float2 CurrentPosition;
        private float2 CurrentDirection;
        private float2[] PredictedPositions;
        private float2[] PredictedDirections;

        // Features -----------------------------------------------------------------
        private int TrajectoryPosFeatureIndex;
        private int TrajectoryRotFeatureIndex;
        private int[] TrajectoryPosPredictionFrames;
        private int[] TrajectoryRotPredictionFrames;
        private int NumberPredictionPos { get { return TrajectoryPosPredictionFrames.Length; } }
        private int NumberPredictionRot { get { return TrajectoryRotPredictionFrames.Length; } }
        // --------------------------------------------------------------------------

        private void Start()
        {
            // Get the feature indices
            TrajectoryPosFeatureIndex = -1;
            TrajectoryRotFeatureIndex = -1;
            for (int i = 0; i < MotionMatching.MMData.TrajectoryFeatures.Count; ++i)
            {
                if (MotionMatching.MMData.TrajectoryFeatures[i].Name == TrajectoryPositionFeatureName) TrajectoryPosFeatureIndex = i;
                if (MotionMatching.MMData.TrajectoryFeatures[i].Name == TrajectoryDirectionFeatureName) TrajectoryRotFeatureIndex = i;
            }
            Debug.Assert(TrajectoryPosFeatureIndex != -1, "Trajectory Position Feature not found");
            Debug.Assert(TrajectoryRotFeatureIndex != -1, "Trajectory Direction Feature not found");

            TrajectoryPosPredictionFrames = MotionMatching.MMData.TrajectoryFeatures[TrajectoryPosFeatureIndex].FramesPrediction;
            TrajectoryRotPredictionFrames = MotionMatching.MMData.TrajectoryFeatures[TrajectoryRotFeatureIndex].FramesPrediction;
            // TODO: generalize this, allow for different number of prediction frames
            Debug.Assert(TrajectoryPosPredictionFrames.Length == TrajectoryRotPredictionFrames.Length, "Trajectory Position and Trajectory Direction Prediction Frames must be the same for PathCharacterController");
            for (int i = 0; i < TrajectoryPosPredictionFrames.Length; ++i)
            {
                Debug.Assert(TrajectoryPosPredictionFrames[i] == TrajectoryRotPredictionFrames[i], "Trajectory Position and Trajectory Direction Prediction Frames must be the same for PathCharacterController");
            }

            PredictedPositions = new float2[NumberPredictionPos];
            PredictedDirections = new float2[NumberPredictionRot];
        }

        protected override void OnUpdate()
        {
            float speed = Speed / SplineContainer.CalculateLength();

            float delta = speed * DatabaseDeltaTime * 0.1f;

            float3 pos = SplineContainer.EvaluatePosition(T);
            CurrentPosition = pos.xz;
            float3 nextPos = SplineContainer.EvaluatePosition(math.frac(T + delta));
            CurrentDirection = math.normalize(new float2(nextPos.x - pos.x, nextPos.z - pos.z));

            for (int i = 0; i < NumberPredictionPos; i++)
            {
                float t = math.frac(T + TrajectoryPosPredictionFrames[i] * speed * DatabaseDeltaTime);
                float3 predPos = SplineContainer.EvaluatePosition(t);
                PredictedPositions[i] = predPos.xz;
                float3 predNextPos = SplineContainer.EvaluatePosition(math.frac(t + delta));
                PredictedDirections[i] = math.normalize(new float2(predNextPos.x - predPos.x, predNextPos.z - predPos.z));
            }

            T += speed * Time.deltaTime;
            T = math.frac(T);
        }

        public float3 GetCurrentPosition()
        {
            return new Vector3(CurrentPosition.x, 0, CurrentPosition.y);
        }
        public quaternion GetCurrentRotation()
        {
            Quaternion rot = Quaternion.LookRotation(new Vector3(CurrentDirection.x, 0, CurrentDirection.y));
            return rot;
        }

        public override void GetTrajectoryFeature(TrajectoryFeature feature, int index, Transform character, NativeArray<float> output)
        {
            if (!feature.SimulationBone) Debug.Assert(false, "Trajectory should be computed using the SimulationBone");
            switch (feature.FeatureType)
            {
                case TrajectoryFeature.Type.Position:
                    float2 world = GetWorldPredictedPos(index);
                    float3 local = character.InverseTransformPoint(new float3(world.x, 0.0f, world.y));
                    output[0] = local.x;
                    output[1] = local.z;
                    break;
                case TrajectoryFeature.Type.Direction:
                    float2 worldDir = GetWorldPredictedDir(index);
                    float3 localDir = character.InverseTransformDirection(new Vector3(worldDir.x, 0.0f, worldDir.y));
                    output[0] = localDir.x;
                    output[1] = localDir.z;
                    break;
                default:
                    Debug.Assert(false, "Unknown feature type: " + feature.FeatureType);
                    break;
            }
        }

        private float2 GetWorldPredictedPos(int index)
        {
            return PredictedPositions[index];
        }
        private float2 GetWorldPredictedDir(int index)
        {
            return PredictedDirections[index];
        }

        public override float3 GetWorldInitPosition()
        {
            return (float3)transform.position;
        }
        public override float3 GetWorldInitDirection()
        {
            return math.normalize(new float3(transform.forward.x, 0, transform.forward.z));
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (SplineContainer == null) return;

            const float heightOffset = 0.01f;

            // Draw Current Position And Direction
            if (!Application.isPlaying) return;
            Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
            Vector3 currentPos = (Vector3)GetCurrentPosition() + Vector3.up * heightOffset * 2;
            Gizmos.DrawSphere(currentPos, 0.1f);
            GizmosExtensions.DrawLine(currentPos, currentPos + (Quaternion)GetCurrentRotation() * Vector3.forward, 12);
            // Draw Prediction
            if (PredictedPositions == null || PredictedPositions.Length != NumberPredictionPos ||
                PredictedDirections == null || PredictedDirections.Length != NumberPredictionRot) return;
            Gizmos.color = new Color(0.6f, 0.3f, 0.8f, 1.0f);
            for (int i = 0; i < NumberPredictionPos; i++)
            {
                float2 predictedPosf2 = GetWorldPredictedPos(i);
                Vector3 predictedPos = new Vector3(predictedPosf2.x, heightOffset * 2, predictedPosf2.y);
                Gizmos.DrawSphere(predictedPos, 0.1f);
                float2 dirf2 = GetWorldPredictedDir(i);
                GizmosExtensions.DrawLine(predictedPos, predictedPos + new Vector3(dirf2.x, 0.0f, dirf2.y) * 0.5f, 12);
            }
        }

        public override NativeArray<(float2, float)> GetAllObstacles(Transform character)
        {
            throw new System.NotImplementedException();
        }

        public override NativeArray<(float2, float)> GetNearbyObstacles(Transform character)
        {
            throw new System.NotImplementedException();
        }
#endif
    }
}
