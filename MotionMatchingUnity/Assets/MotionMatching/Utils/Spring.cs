using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    /* Thanks to: https://theorangeduck.com/page/spring-roll-call */
    public static class Spring
    {
        /// <summary>
        /// Variation of the damper code that damps a point starting at zero moving toward the desired difference
        /// </summary>
        public static float3 DampAdjustmentImplicit(float3 goal, float halfLife, float dt, float eps = 1e-5f)
        {
            const float LN2f = 0.69314718056f;
            return goal * (1.0f - FastNEgeExp((LN2f * dt) / (halfLife + eps)));
        }
        /// <summary>
        /// Variation of the damper code that damps a rotation starting at the identity rotation toward the desired difference
        /// </summary>
        public static quaternion DampAdjustmentImplicit(quaternion goal, float halfLife, float dt, float eps = 1e-5f)
        {
            const float LN2f = 0.69314718056f;
            return math.slerp(quaternion.identity, goal, 1.0f - FastNEgeExp((LN2f * dt) / (halfLife + eps)));
        }
        /// <summary>
        /// Given a position, velocity, desired velocity and acceleration, returns the new
        /// position, velocity and acceleration after deltaTime seconds.
        /// </summary>
        public static void CharacterPositionUpdate(ref float2 pos, ref float2 velocity, ref float2 acceleration,
                                                   float2 velocityGoal, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float2 j0 = velocity - velocityGoal;
            float2 j1 = acceleration + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            pos = eyedt * (((-j1) / (y * y)) + ((-j0 - j1 * deltaTime) / y)) +
                  (j1 / (y * y)) + j0 / y + velocityGoal * deltaTime + pos;
            velocity = eyedt * (j0 + j1 * deltaTime) + velocityGoal;
            acceleration = eyedt * (acceleration - j1 * y * deltaTime);
        }
        /// <summary>
        /// Given a position, velocity, desired velocity and acceleration, returns the new
        /// position, velocity and acceleration after deltaTime seconds.
        /// </summary>
        public static void CharacterPositionUpdate(ref float3 pos, ref float3 velocity, ref float3 acceleration,
                                                   float3 velocityGoal, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float3 j0 = velocity - velocityGoal;
            float3 j1 = acceleration + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            pos = eyedt * (((-j1) / (y * y)) + ((-j0 - j1 * deltaTime) / y)) +
                  (j1 / (y * y)) + j0 / y + velocityGoal * deltaTime + pos;
            velocity = eyedt * (j0 + j1 * deltaTime) + velocityGoal;
            acceleration = eyedt * (acceleration - j1 * y * deltaTime);
        }

        /// <summary>
        /// Given the current rotation, angular velocity and the desired rotation, returns the new
        /// rotation and angular velocity after deltaTime seconds.
        /// </summary>
        public static void SimpleSpringDamperImplicit(ref quaternion rot, ref float3 angularVel, quaternion rotGoal, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float3 j0 = MathExtensions.QuaternionToScaledAngleAxis(MathExtensions.Abs(math.mul(rot, math.inverse(rotGoal))));
            float3 j1 = angularVel + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            rot = math.mul(MathExtensions.QuaternionFromScaledAngleAxis(eyedt * (j0 + j1 * deltaTime)), rotGoal);
            angularVel = eyedt * (angularVel - j1 * y * deltaTime);
        }
        /// <summary>
        /// Given the current position, velocity and the desired position, returns the new
        /// position and velocity after deltaTime seconds.
        /// </summary>
        public static void SimpleSpringDamperImplicit(ref float3 pos, ref float3 velocity, float3 posGoal, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float3 j0 = pos - posGoal;
            float3 j1 = velocity + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            pos = eyedt * (j0 + j1 * deltaTime) + posGoal;
            velocity = eyedt * (velocity - j1 * y * deltaTime);
        }
        /// <summary>
        /// Given the current position, velocity and the desired position, returns the new
        /// position and velocity after deltaTime seconds.
        /// </summary>
        public static void SimpleSpringDamperImplicit(ref float2 pos, ref float2 velocity, float2 posGoal, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float2 j0 = pos - posGoal;
            float2 j1 = velocity + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            pos = eyedt * (j0 + j1 * deltaTime) + posGoal;
            velocity = eyedt * (velocity - j1 * y * deltaTime);
        }

        /// <summary>
        /// Special type of SpringDamperImplicit when the desired rotation is the identity
        /// </summary>
        public static void DecaySpringDamperImplicit(ref quaternion rot, ref float3 angularVel, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float3 j0 = MathExtensions.QuaternionToScaledAngleAxis(rot);
            float3 j1 = angularVel + j0 * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            rot = MathExtensions.QuaternionFromScaledAngleAxis(eyedt * (j0 + j1 * deltaTime));
            angularVel = eyedt * (angularVel - j1 * y * deltaTime);
        }
        /// <summary>
        /// Special type of SpringDamperImplicit when the desired position is 0
        /// </summary>
        public static void DecaySpringDamperImplicit(ref float3 pos, ref float3 velocity, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float3 j1 = velocity + pos * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            pos = eyedt * (pos + j1 * deltaTime);
            velocity = eyedt * (velocity - j1 * y * deltaTime);
        }
        /// <summary>
        /// Special type of SpringDamperImplicit when the value is 0
        /// </summary>
        public static void DecaySpringDamperImplicit(ref float value, ref float velocity, float halfLife, float deltaTime)
        {
            float y = HalfLifeToDamping(halfLife) / 2.0f; // this could be precomputed
            float j1 = velocity + value * y;
            float eyedt = FastNEgeExp(y * deltaTime); // this could be precomputed if several agents use it the same frame

            value = eyedt * (value + j1 * deltaTime);
            velocity = eyedt * (velocity - j1 * y * deltaTime);
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
    }
}