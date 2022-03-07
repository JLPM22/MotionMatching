using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    public class PathCharacterController : MotionMatchingCharacterController
    {
        public KeyPoint[] Path; // The key points of the path

        private int CurrentKeyPoint; // The current key point index
        private float CurrentKeyPointT; // [0..1] which part of the current keypoint is the character currently at

        private float2 CurrentPosition;
        private float2 CurrentDirection;
        private float2[] PredictedPositions;
        private float2[] PredictedDirections;

        private void Start()
        {
            PredictedPositions = new float2[NumberPrediction];
            PredictedDirections = new float2[NumberPrediction];
        }

        protected override void OnUpdate()
        {
            // Predict the future positions and directions
            for (int i = 0; i < NumberPrediction; i++)
            {
                SimulatePath(AveragedDeltaTime * (i + 1) * PredictionFrames, CurrentKeyPoint, CurrentKeyPointT,
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

        public override float3 GetCurrentPosition()
        {
            return transform.position + new Vector3(CurrentPosition.x, 0, CurrentPosition.y);
        }
        public override quaternion GetCurrentRotation()
        {
            Quaternion rot = Quaternion.LookRotation(new Vector3(CurrentDirection.x, 0, CurrentDirection.y));
            return rot * transform.rotation;
        }

        public override float2 GetWorldPredictedPosition(int index)
        {
            return PredictedPositions[index] + new float2(transform.position.x, transform.position.z);
        }
        public override float2 GetWorldPredictedDirection(int index)
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
                Gizmos.DrawLine(new Vector3(pos.x, heightOffset, pos.z), new Vector3(nextPos.x, heightOffset, nextPos.z));
            }
            // Last Line
            float3 lastPos = Path[Path.Length - 1].GetWorldPosition(transform);
            float3 firstPos = Path[0].GetWorldPosition(transform);
            Gizmos.DrawLine(new Vector3(lastPos.x, heightOffset, lastPos.z), new Vector3(firstPos.x, heightOffset, firstPos.z));
            // Draw Velocity
            for (int i = 0; i < Path.Length - 1; i++)
            {
                float3 pos = Path[i].GetWorldPosition(transform);
                float3 nextPos = Path[i + 1].GetWorldPosition(transform);
                Vector3 start = new Vector3(pos.x, heightOffset, pos.z);
                Vector3 end = new Vector3(nextPos.x, heightOffset, nextPos.z);
                GizmosExtensions.DrawArrow(start, start + (end - start).normalized * math.min(Path[i].Velocity, math.distance(pos, nextPos)));
            }
            // Last Line
            float3 lastPos2 = Path[Path.Length - 1].GetWorldPosition(transform);
            float3 firstPos2 = Path[0].GetWorldPosition(transform);
            Vector3 start2 = new Vector3(lastPos2.x, heightOffset, lastPos2.z);
            Vector3 end2 = new Vector3(firstPos2.x, heightOffset, firstPos2.z);
            GizmosExtensions.DrawArrow(start2, start2 + (end2 - start2).normalized * Path[Path.Length - 1].Velocity);

            // Draw Current Position And Direction
            if (!Application.isPlaying) return;
            Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
            Vector3 currentPos = (Vector3)GetCurrentPosition() + Vector3.up * heightOffset * 2;
            Gizmos.DrawSphere(currentPos, 0.1f);
            Gizmos.DrawLine(currentPos, currentPos + (Quaternion)GetCurrentRotation() * Vector3.forward);
            // Draw Prediction
            if (PredictedPositions == null || PredictedPositions.Length != NumberPrediction ||
                PredictedDirections == null || PredictedDirections.Length != NumberPrediction) return;
            Gizmos.color = new Color(0.6f, 0.3f, 0.8f, 1.0f);
            for (int i = 0; i < NumberPrediction; i++)
            {
                float2 predictedPosf2 = GetWorldPredictedPosition(i);
                Vector3 predictedPos = new Vector3(predictedPosf2.x, heightOffset * 2, predictedPosf2.y);
                Gizmos.DrawSphere(predictedPos, 0.1f);
                float2 dirf2 = GetWorldPredictedDirection(i);
                Gizmos.DrawLine(predictedPos, predictedPos + new Vector3(dirf2.x, 0.0f, dirf2.y));
            }
        }
#endif
    }
}