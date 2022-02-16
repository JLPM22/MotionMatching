using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Mathematics;

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
        [Header("Adjustment")] // Move Simulation Bone towards the Simulation Object (motion matching towards character controller)
        public MotionMatchingController SimulationBone; // MotionMatchingController's transform is the SimulationBone of the character
        [Range(0.0f, 1.0f)] public float PositionAdjustmentHalflife = 0.1f; // Time needed to move half of the distance between SimulationBone and SimulationObject
        // [Range(0.0f, 1.0f)] public float RotationAdjustmentHalflife = 0.2f;
        [Range(0.0f, 1.0f)] public float MaxDistanceSimulationBoneAndObject = 0.1f; // Max distance between SimulationBone and SimulationObject

        private float2 InputMovement;
        private float2[] PredictedPosition;
        private float2 Velocity;
        private float2 Direction;
        private float2[] PredictedVelocity;
        private float2 Acceleration;
        private Queue<float> LastDeltaTime = new Queue<float>();
        private float SumDeltaTime;

        private void Start()
        {
            PredictedPosition = new float2[NumberPrediction];
            PredictedVelocity = new float2[NumberPrediction];
            for (int i = 0; i < NumberPrediction; ++i) PredictedPosition[i] = new float2(transform.position.x, transform.position.z);
            Direction = new float2(transform.forward.x, transform.forward.z);
        }

        // Input a change in the movement direction
        public void SetMovementDirection(float2 movementDirection)
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
            float2 speed = InputMovement * MaxSpeed;
            float2 newPos = new float2(transform.position.x, transform.position.z);
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
            if (math.lengthsq(Velocity) > MinimumVelocityClamp * MinimumVelocityClamp)
            {
                transform.position = new float3(newPos.x, transform.position.y, newPos.y);
                Direction = math.normalize(Velocity);
            }

            if (OnUpdate != null) OnUpdate(Time.deltaTime);

            // Adjust SimulationBone to pull the character (moving SimulationBone) towards the Simulation Object (character controller)
            AdjustSimulationBone();
        }

        private void AdjustSimulationBone()
        {
            AdjustCharacterPosition();
            //AdjustCharacterRotation();

            // Clamp Position
            float3 simulationObject = transform.position;
            float3 simulationBone = SimulationBone.transform.position;
            if (math.distance(simulationObject, simulationBone) > MaxDistanceSimulationBoneAndObject)
            {
                SimulationBone.transform.position = MaxDistanceSimulationBoneAndObject * math.normalize(simulationBone - simulationObject) + simulationObject;
            }
        }

        private void AdjustCharacterPosition()
        {
            float3 simulationObject = transform.position;
            float3 simulationBone = SimulationBone.transform.position;
            float3 differencePosition = simulationObject - simulationBone;
            // Damp the difference using the adjustment halflife and dt
            float3 adjustmentPosition = DampAdjustmentImplicit(differencePosition, PositionAdjustmentHalflife, Time.deltaTime);
            // Move the simulation bone towards the simulation object
            SimulationBone.transform.position = simulationBone + adjustmentPosition;
        }

        // private void AdjustCharacterRotation()
        // {
        //     quaternion simulationObject = transform.rotation;
        //     quaternion simulationBone = SimulationBone.transform.rotation;
        //     // Find the difference in rotation (from character to simulation object)
        //     // Note: if numerically unstable, try quaternion.Normalize(quaternion.Inverse(simulationObject) * simulationBone)
        //     quaternion differenceRotation = quaternion.Inverse(simulationObject) * simulationBone;
        //     // Damp the difference using the adjustment halflife and dt
        //     quaternion adjustmentRotation = DampAdjustmentImplicit(differenceRotation, RotationAdjustmentHalflife, Time.deltaTime);
        //     // Rotate the simulation bone towards the simulation object
        //     SimulationBone.transform.rotation = simulationBone * adjustmentRotation;
        // }

        /// <summary>
        /// Variation of the damper code that damps a point starting at zero moving toward the desired difference
        /// </summary>
        private static float3 DampAdjustmentImplicit(float3 goal, float halfLife, float dt, float eps = 1e-5f)
        {
            const float LN2f = 0.69314718056f;
            return goal * (1.0f - FastNEgeExp((LN2f * dt) / (halfLife + eps)));
        }

        // /// <summary>
        // /// Variation of the damper code that damps a rotation starting at the identity rotation toward the desired difference
        // /// </summary>
        // private static quaternion DampAdjustmentImplicit(quaternion goal, float halfLife, float dt, float eps = 1e-5f)
        // {
        //     const float LN2f = 0.69314718056f;
        //     return quaternion.Slerp(quaternion.identity, goal, 1.0f - FastNEgeExp((LN2f * dt) / (halfLife + eps)));
        // }

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
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float j0 = velocity - velocityGoal;
            float j1 = acceleration + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            pos = eyedt * (((-j1) / (y * y)) + ((-j0 - j1 * deltaTime) / y)) +
                  (j1 / (y * y)) + j0 / y + velocityGoal * deltaTime + pos;
            velocity = eyedt * (j0 + j1 * deltaTime) + velocityGoal;
            acceleration = eyedt * (acceleration - j1 * y * deltaTime);
        }
        private static float HalfLifeToDamping(float halfLife, float eps = 1e-5f)
        {
            const float LN2f = 0.69314718056f;
            return (4.0f * LN2f) / (halfLife + eps);
        }
        private static float FastNEgeExp(float x)
        {
            return 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
        }

        // public float2 GetLocalPredictedPosition(int index)
        // {
        //     float3 localPos = new float3(PredictedPosition[index].x, transform.position.y, PredictedPosition[index].y);
        //     localPos -= transform.position;
        //     localPos = quaternion.Inverse(quaternion.LookRotation(new float3(Direction.x, 0.0f, Direction.y), float3.up)) * localPos;
        //     return new float2(localPos.x, localPos.z);
        // }

        // public float2 GetLocalPredictedDirection(int index)
        // {
        //     float3 localVel = new float3(PredictedVelocity[index].x, 0.0f, PredictedVelocity[index].y);
        //     if (localVel.sqrMagnitude > MinimumVelocityClamp * MinimumVelocityClamp)
        //     {
        //         localVel = quaternion.Inverse(quaternion.LookRotation(new float3(Direction.x, 0.0f, Direction.y), float3.up)) * localVel;
        //     }
        //     else
        //     {
        //         localVel = float3.forward;
        //     }
        //     return (new float3(localVel.x, localVel.z)).normalized;
        // }

        public float2 GetWorldPredictedPosition(int index)
        {
            return PredictedPosition[index];
        }

        public float2 GetWorldPredictedDirection(int index)
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
            Gizmos.DrawSphere(transform.position, radius);
            Gizmos.DrawLine(transform.position, transform.position + (Vector3)(new float3(Velocity.x, 0.0f, Velocity.y)) * vectorReduction);

            if (PredictedPosition == null || PredictedVelocity == null) return;

            // Draw Predicted Position & Velocity
            Gizmos.color = new Color(0.6f, 0.3f, 0.8f, 1.0f);
            for (int i = 0; i < PredictedPosition.Length; ++i)
            {
                float3 current = new float3(PredictedPosition[i].x, 0.0f, PredictedPosition[i].y);
                float3 currentVelocity = new float3(PredictedVelocity[i].x, 0.0f, PredictedVelocity[i].y);
                Gizmos.DrawSphere(current, radius);
                Gizmos.DrawLine(current, current + currentVelocity * vectorReduction);
            }

            // Draw Clamp Circle
            Gizmos.color = new Color(0.1f, 1.0f, 0.1f, 1.0f);
            GizmosExtensions.DrawWireCircle(transform.position, MaxDistanceSimulationBoneAndObject, quaternion.identity);
        }
#endif
    }
}