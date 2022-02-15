using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionMatching
{
    // Read to improve correspondance between input and animation
    /* https://theorangeduck.com/page/code-vs-data-driven-displacement */

    // TODO: Smooth change of direction as Daniel Holden in https://theorangeduck.com/page/code-vs-data-driven-displacement

    public class SpringCharacterController : MonoBehaviour
    {
        public event Action<float> OnUpdate;

        public int NumberPrediction = 3;
        public int PredictionFrames = 20;
        public float MaxSpeed = 1.0f;
        [Range(0.0f, 1.0f)] public float Responsiveness = 0.75f;
        public float MinimumVelocityClamp = 0.01f;

        private Vector2 InputMovement;
        private Vector2[] PredictedPosition;
        private Vector2 Velocity;
        private Vector2 Direction;
        private Vector2[] PredictedVelocity;
        private Vector2 Acceleration;
        private Queue<float> LastDeltaTime = new Queue<float>();
        private float SumDeltaTime;

        private void Start()
        {
            PredictedPosition = new Vector2[NumberPrediction];
            PredictedVelocity = new Vector2[NumberPrediction];
            for (int i = 0; i < NumberPrediction; ++i) PredictedPosition[i] = new Vector2(transform.position.x, transform.position.z);
            Direction = new Vector2(transform.forward.x, transform.forward.z);
        }

        // Input a change in the movement direction
        public void SetMovementDirection(Vector2 movementDirection)
        {
            InputMovement = movementDirection;
        }

        private void Update()
        {
            // Average DeltaTime
            const int nAverageDeltaTime = 20;
            SumDeltaTime += Time.deltaTime;
            LastDeltaTime.Enqueue(Time.deltaTime);
            if (LastDeltaTime.Count == nAverageDeltaTime + 1) SumDeltaTime -= LastDeltaTime.Dequeue();

            // Positions
            Vector2 speed = InputMovement * MaxSpeed;
            Vector2 newPos = new Vector2(transform.position.x, transform.position.z);
            for (int i = 0; i < 2; ++i)
            {
                // Update Current Position
                float newX = newPos[i], newVel = Velocity[i], newAcc = Acceleration[i];
                SpringCharacterUpdate(ref newX, ref newVel, ref newAcc, speed[i], 1.0f - Responsiveness, Time.deltaTime);
                newPos[i] = newX;
                Velocity[i] = newVel;
                Acceleration[i] = newAcc;
                // Predict
                Span<float> predictPos = stackalloc float[NumberPrediction];
                Span<float> predictVel = stackalloc float[NumberPrediction];
                SpringCharacterPredict(predictPos, predictVel,
                                       NumberPrediction, newPos[i], Velocity[i], Acceleration[i], speed[i],
                                       1.0f - Responsiveness, SumDeltaTime / nAverageDeltaTime);
                for (int j = 0; j < NumberPrediction; ++j)
                {
                    PredictedPosition[j][i] = predictPos[j];
                    PredictedVelocity[j][i] = predictVel[j];
                }
            }
            if (Velocity.sqrMagnitude > MinimumVelocityClamp * MinimumVelocityClamp)
            {
                transform.position = new Vector3(newPos.x, transform.position.y, newPos.y);
                Direction = Velocity.normalized;
            }

            if (OnUpdate != null) OnUpdate(Time.deltaTime);
        }

        /* https://theorangeduck.com/page/spring-roll-call#controllers */
        private void SpringCharacterPredict(Span<float> pPose, Span<float> pVelocity,
                                            int count, float pos, float velocity, float acceleration, float velocityGoal,
                                            float halfLife, float deltaTime)
        {
            for (int i = 0; i < count; ++i)
            {
                pPose[i] = pos;
                pVelocity[i] = velocity;
                float a = acceleration;
                SpringCharacterUpdate(ref pPose[i], ref pVelocity[i], ref a, velocityGoal, halfLife, (i + 1) * PredictionFrames * deltaTime);
            }
        }
        private void SpringCharacterUpdate(ref float pos, ref float velocity, ref float acceleration,
                                           float velocityGoal, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f;
            float j0 = velocity - velocityGoal;
            float j1 = acceleration + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime);

            pos = eyedt * (((-j1) / (y * y)) + ((-j0 - j1 * deltaTime) / y)) +
                  (j1 / (y * y)) + j0 / y + velocityGoal * deltaTime + pos;
            velocity = eyedt * (j0 + j1 * deltaTime) + velocityGoal;
            acceleration = eyedt * (acceleration - j1 * y * deltaTime);
        }
        private static float HalfLifeToDamping(float halfLife, float eps = 1e-5f)
        {
            return (4.0f * 0.69314718056f) / (halfLife + eps);
        }
        private static float FastNEgeExp(float x)
        {
            return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
        }

        // public Vector2 GetLocalPredictedPosition(int index)
        // {
        //     Vector3 localPos = new Vector3(PredictedPosition[index].x, transform.position.y, PredictedPosition[index].y);
        //     localPos -= transform.position;
        //     localPos = Quaternion.Inverse(Quaternion.LookRotation(new Vector3(Direction.x, 0.0f, Direction.y), Vector3.up)) * localPos;
        //     return new Vector2(localPos.x, localPos.z);
        // }

        // public Vector2 GetLocalPredictedDirection(int index)
        // {
        //     Vector3 localVel = new Vector3(PredictedVelocity[index].x, 0.0f, PredictedVelocity[index].y);
        //     if (localVel.sqrMagnitude > MinimumVelocityClamp * MinimumVelocityClamp)
        //     {
        //         localVel = Quaternion.Inverse(Quaternion.LookRotation(new Vector3(Direction.x, 0.0f, Direction.y), Vector3.up)) * localVel;
        //     }
        //     else
        //     {
        //         localVel = Vector3.forward;
        //     }
        //     return (new Vector3(localVel.x, localVel.z)).normalized;
        // }

        public Vector2 GetWorldPredictedPosition(int index)
        {
            return PredictedPosition[index];
        }

        public Vector2 GetWorldPredictedDirection(int index)
        {
            return Direction;
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            const float radius = 0.05f;
            const float vectorReduction = 0.5f;
            // Draw Current Position & Velocity
            Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
            Gizmos.DrawWireSphere(transform.position, radius);
            Gizmos.DrawLine(transform.position, transform.position + (new Vector3(Velocity.x, 0.0f, Velocity.y)) * vectorReduction);

            if (PredictedPosition == null || PredictedVelocity == null) return;

            // Draw Predicted Position & Velocity
            Gizmos.color = new Color(0.1f, 0.3f, 1.0f, 1.0f);
            for (int i = 0; i < PredictedPosition.Length; ++i)
            {
                Vector3 current = new Vector3(PredictedPosition[i].x, 0.0f, PredictedPosition[i].y);
                Vector3 currentVelocity = new Vector3(PredictedVelocity[i].x, 0.0f, PredictedVelocity[i].y);
                Gizmos.DrawWireSphere(current, radius);
                Gizmos.DrawLine(current, current + currentVelocity * vectorReduction);
            }
        }
#endif
    }
}