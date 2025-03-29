using System;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Splines;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

    public class CrowdSplineCharacterController : MotionMatchingCharacterController
    {
        public string TrajectoryPositionFeatureName = "FuturePosition";
        public string TrajectoryDirectionFeatureName = "FutureDirection";

        [Header("Crowds")]
        public Obstacle IgnoreObstacle;
        public SplineContainer SplineContainer;
        public bool Loop = true;
        public float Speed = 1.0f;
        public bool UpdateOnlyWhenCharacterMoving = false;
        public float DistanceToMove = 1.0f; // If UpdateOnlyWhenCharacterMoving is true, the character will only move if the distance to the next point is greater than this value
        public float DistanceResumeMoving = 0.75f; // If UpdateOnlyWhenCharacterMoving is true and DistanceToMove was not reached, the character will resume moving if the distance to the next point is greater than this value
        public bool DoSteering = false;
        public float SteeringLookAhead = 4.0f;
        public float SteeringForce = 2.0f;
        public float SteeringSplineForce = 0.5f;
        public float SteeringChangeFactor = 5.0f;

        public bool DebugDraw = true;

        private float T;
        private bool IsStopped;
        private float2 Steering;
        private float2 SteeringOffset;

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
        // Crowds ------------------------------------------------------------------
        private Obstacle[] Obstacles;
        private NativeArray<(float2, float)> ObstaclesArray;
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
            // Crowds
            Obstacle[] candidates = FindObjectsByType<Obstacle>(FindObjectsInactive.Exclude, FindObjectsSortMode.None);
            Obstacles = new Obstacle[candidates.Length - 1];
            int it = 0;
            for (int i = 0; i < candidates.Length; i++)
            {
                if (candidates[i] != IgnoreObstacle)
                {
                    Obstacles[it] = candidates[i];
                    it += 1;
                }
            }
            ObstaclesArray = new NativeArray<(float2, float)>(Obstacles.Length, Allocator.Persistent);
        }

        protected override void OnUpdate()
        {
            float speed = Speed / SplineContainer.CalculateLength();
            float delta = speed * DatabaseDeltaTime * 0.1f;

            void updateT()
            {
                T += speed * Time.deltaTime;
                if (Loop)
                {
                    T = math.frac(T);
                }
            }

            float getTDelta(float t, float delta)
            {
                float tDelta = t + delta;
                if (SplineContainer.Spline.Closed)
                {
                    tDelta = math.frac(tDelta);
                }
                else if (tDelta > 1.0f)
                {
                    tDelta = 1.0f;
                }
                return tDelta;
            }

            float3 pos = SplineContainer.EvaluatePosition(T);
            CurrentPosition = pos.xz;
            float3 nextPos = SplineContainer.EvaluatePosition(getTDelta(T, delta));
            float2 predDir = new(nextPos.x - pos.x, nextPos.z - pos.z);
            if (math.lengthsq(predDir) < 1e-9) predDir = CurrentDirection;
            CurrentDirection = math.normalize(predDir);

            if (!IsStopped)
            {
                for (int i = 0; i < NumberPredictionPos; i++)
                {
                    float t = getTDelta(T, TrajectoryPosPredictionFrames[i] * speed * DatabaseDeltaTime);
                    float3 predPos = SplineContainer.EvaluatePosition(t);
                    PredictedPositions[i] = predPos.xz;
                    float3 predNextPos = SplineContainer.EvaluatePosition(getTDelta(t, delta));
                    float2 predNextDir = new(predNextPos.x - predPos.x, predNextPos.z - predPos.z);
                    if (math.lengthsq(predNextDir) < 1e-9) predNextDir = CurrentDirection;
                    PredictedDirections[i] = math.normalize(predNextDir);
                }
            }

            if (UpdateOnlyWhenCharacterMoving)
            {
                float2 characterPos = new(MotionMatching.transform.position.x, MotionMatching.transform.position.z);
                float distance = math.length(new float2(nextPos.x - characterPos.x, nextPos.z - characterPos.y));
                float3 deltaPos = SplineContainer.EvaluatePosition(getTDelta(T, TrajectoryPosPredictionFrames[^1] * speed * DatabaseDeltaTime));
                float distanceDelta = math.length(new float2(deltaPos.x - characterPos.x, deltaPos.z - characterPos.y));
                if (distanceDelta < distance || (!IsStopped && distance < DistanceToMove))
                {
                    IsStopped = false;
                    updateT();
                }
                else
                {
                    IsStopped = distance > DistanceResumeMoving;
                    for (int i = 0; i < NumberPredictionPos; i++)
                    {
                        PredictedPositions[i] = math.lerp(PredictedPositions[i], CurrentPosition + SteeringOffset, Time.deltaTime);
                        PredictedDirections[i] = PredictedDirections[i];
                    }
                }
            }
            else
            {
                updateT();
                IsStopped = false;
            }

            if (DoSteering)
            {
                if (IsStopped)
                {
                    // PredictedPositions was not updated
                    for (int i = 0; i < NumberPredictionPos; i++)
                    {
                        PredictedPositions[i] -= SteeringOffset;
                    }
                }

                float2 steeringPos = CurrentPosition + SteeringOffset;
                float2 targetSteering = CrowdCharacterController.ComputeSteering(steeringPos, new Vector3(CurrentDirection.x, 0.0f, CurrentDirection.y),
                                                                                 Obstacles, SteeringLookAhead, SteeringForce);
                targetSteering += -SteeringOffset * SteeringSplineForce;
                Steering = math.lerp(Steering, targetSteering, Time.deltaTime * SteeringChangeFactor);
                SteeringOffset += Steering * Time.deltaTime;

                for (int i = 0; i < NumberPredictionPos; i++)
                {
                    PredictedPositions[i] += SteeringOffset;
                }
            }
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

        public override void GetDynamicFeature(TrajectoryFeature feature, int index, Transform character, NativeArray<float> output)
        {
            if (feature.Name == "FutureEllipse")
            {
                output[0] = 0.0f;
                output[1] = 0.0f;
                output[2] = 0.0f;
                output[3] = 0.0f;
                for (int i = 0; i < Obstacles.Length; i++)
                {
                    float3 world = Obstacles[i].GetProjWorldPosition(); ;
                    float3 localPos = character.InverseTransformPoint(world);
                    // HARDCODED: circle radius
                    ObstaclesArray[i] = (new float2(localPos.x, localPos.z), Obstacles[i].Radius);
                }
                MotionMatching.SetObstacle(ObstaclesArray);
            }
            else
            {
                Debug.Assert(false, "Unknown feature name: " + feature.Name);
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
            return SplineContainer.EvaluatePosition(0.0f);
        }
        public override float3 GetWorldInitDirection()
        {
            float3 start = SplineContainer.EvaluatePosition(0.0f);
            float3 delta = SplineContainer.EvaluatePosition(0.01f) - start;
            return math.normalize(new float3(delta.x, 0.0f, delta.z));
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (!DebugDraw) return;

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

            // Draw Steering
            if (DoSteering && math.lengthsq(Steering) > 0.0001f)
            {
                Gizmos.color = new Color(0.1f, 0.8f, 0.1f, 1.0f);
                GizmosExtensions.DrawLine(MotionMatching.transform.position, MotionMatching.transform.position + new Vector3(Steering.x, 0.0f, Steering.y) / SteeringForce, 3);
            }
        }
#endif
    }
}
