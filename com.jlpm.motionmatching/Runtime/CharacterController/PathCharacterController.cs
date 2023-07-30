using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

    public class PathCharacterController : MotionMatchingCharacterController
    {
        public string TrajectoryPositionFeatureName = "FuturePosition";
        public string TrajectoryDirectionFeatureName = "FutureDirection";
        public KeyPoint[] Path; // The key points of the path

        private int CurrentKeyPoint; // The current key point index
        private float CurrentKeyPointT; // [0..1] which part of the current keypoint is the character currently at

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
            // Predict the future positions and directions
            for (int i = 0; i < NumberPredictionPos; i++)
            {
                SimulatePath(DatabaseDeltaTime * TrajectoryPosPredictionFrames[i], CurrentKeyPoint, CurrentKeyPointT,
                             out _, out _,
                             out PredictedPositions[i], out PredictedDirections[i]);
            }
            // Update Current Position and Direction
            SimulatePath(Time.deltaTime, CurrentKeyPoint, CurrentKeyPointT,
                         out CurrentKeyPoint, out CurrentKeyPointT,
                         out CurrentPosition, out CurrentDirection);
        }

        private void SimulatePath(float remainingTime, int currentKeypoint, float currentKeyPointT,
                                  out int nextKeypoint, out float nextKeyPointTime,
                                  out float2 nextPos, out float2 nextDir)
        {
            // Just in case remainingTime is negative or 0
            nextPos = float2.zero;
            nextDir = float2.zero;
            if (remainingTime <= 0)
            {
                KeyPoint current = Path[currentKeypoint];
                KeyPoint next = Path[(currentKeypoint + 1) % Path.Length];
                float2 dir = next.Position - current.Position;
                nextPos = current.Position + dir * currentKeyPointT;
                nextDir = math.normalize(dir);
            }
            // Loop until the character has moved enough
            while (remainingTime > 0)
            {
                KeyPoint current = Path[currentKeypoint];
                KeyPoint next = Path[(currentKeypoint + 1) % Path.Length];
                float2 dir = next.Position - current.Position;
                float2 dirNorm = math.normalize(dir);
                float2 currentPos = current.Position + dir * currentKeyPointT;
                float timeToNext = math.distance(currentPos, next.Position) / current.Velocity; // Time needed to get to the next keypoint
                float dt = math.min(remainingTime, timeToNext);
                remainingTime -= dt;
                if (remainingTime <= 0)
                {
                    // Move
                    currentPos += dirNorm * current.Velocity * dt;
                    currentKeyPointT = math.distance(current.Position, currentPos) / math.distance(current.Position, next.Position);
                    nextPos = currentPos;
                    nextDir = dirNorm;
                }
                else
                {
                    // Advance to next keypoint
                    currentKeypoint = (currentKeypoint + 1) % Path.Length;
                    currentKeyPointT = 0;
                }
            }
            Debug.Assert(math.abs(remainingTime) < 0.0001f, "Character did not move enough or moved to much. remainingTime = " + remainingTime);
            nextKeypoint = currentKeypoint;
            nextKeyPointTime = currentKeyPointT;
        }

        public float3 GetCurrentPosition()
        {
            return transform.position + new Vector3(CurrentPosition.x, 0, CurrentPosition.y);
        }
        public quaternion GetCurrentRotation()
        {
            Quaternion rot = Quaternion.LookRotation(new Vector3(CurrentDirection.x, 0, CurrentDirection.y));
            return rot * transform.rotation;
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
            return PredictedPositions[index] + new float2(transform.position.x, transform.position.z);
        }
        private float2 GetWorldPredictedDir(int index)
        {
            return PredictedDirections[index];
        }

        public override float3 GetWorldInitPosition()
        {
            return new float3(Path[0].Position.x, 0, Path[0].Position.y) + (float3)transform.position;
        }
        public override float3 GetWorldInitDirection()
        {
            float2 dir = Path.Length > 0 ? Path[1].Position - Path[0].Position : new float2(0, 1);
            return math.normalize(new float3(dir.x, 0, dir.y));
        }

        [System.Serializable]
        public struct KeyPoint
        {
            public float2 Position;
            public float Velocity;

            public float3 GetWorldPosition(Transform transform)
            {
                return transform.position + new Vector3(Position.x, 0, Position.y);
            }
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            if (Path == null) return;

            const float heightOffset = 0.01f;

            // Draw KeyPoints
            Gizmos.color = Color.red;
            for (int i = 0; i < Path.Length; i++)
            {
                float3 pos = Path[i].GetWorldPosition(transform);
                Gizmos.DrawSphere(new Vector3(pos.x, heightOffset, pos.z), 0.1f);
            }
            // Draw Path
            Gizmos.color = new Color(0.5f, 0.0f, 0.0f, 1.0f);
            for (int i = 0; i < Path.Length - 1; i++)
            {
                float3 pos = Path[i].GetWorldPosition(transform);
                float3 nextPos = Path[i + 1].GetWorldPosition(transform);
                GizmosExtensions.DrawLine(new Vector3(pos.x, heightOffset, pos.z), new Vector3(nextPos.x, heightOffset, nextPos.z), 6);
            }
            // Last Line
            float3 lastPos = Path[Path.Length - 1].GetWorldPosition(transform);
            float3 firstPos = Path[0].GetWorldPosition(transform);
            GizmosExtensions.DrawLine(new Vector3(lastPos.x, heightOffset, lastPos.z), new Vector3(firstPos.x, heightOffset, firstPos.z), 6);
            // Draw Velocity
            for (int i = 0; i < Path.Length - 1; i++)
            {
                float3 pos = Path[i].GetWorldPosition(transform);
                float3 nextPos = Path[i + 1].GetWorldPosition(transform);
                Vector3 start = new Vector3(pos.x, heightOffset, pos.z);
                Vector3 end = new Vector3(nextPos.x, heightOffset, nextPos.z);
                GizmosExtensions.DrawArrow(start, start + (end - start).normalized * math.min(Path[i].Velocity, math.distance(pos, nextPos)), thickness: 6);
            }
            // Last Line
            float3 lastPos2 = Path[Path.Length - 1].GetWorldPosition(transform);
            float3 firstPos2 = Path[0].GetWorldPosition(transform);
            Vector3 start2 = new Vector3(lastPos2.x, heightOffset, lastPos2.z);
            Vector3 end2 = new Vector3(firstPos2.x, heightOffset, firstPos2.z);
            GizmosExtensions.DrawArrow(start2, start2 + (end2 - start2).normalized * Path[Path.Length - 1].Velocity, thickness: 3);

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
#endif
    }
}