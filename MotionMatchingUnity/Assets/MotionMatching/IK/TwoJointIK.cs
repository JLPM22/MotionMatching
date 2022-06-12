using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    public static class TwoJointIK
    {
        /// <summary>
        /// Solve 2-joint IK considering target as end effector for the joint c.
        /// It is not limited to legs, but for example:
        /// a can be seen as the root/hips, b as the knee, c as the ankle,
        /// and the forward can be seen as the knee forward.
        /// </summary>
        public static void Solve(float3 targetPos, Transform jointA, Transform jointB, Transform jointC, float3 forward)
        {
            float lengthAB = math.distance(jointA.position, jointB.position);
            float lengthBC = math.distance(jointB.position, jointC.position);

            float3 aPos = jointA.position;
            float3 bPos = jointB.position;
            float3 cPos = jointC.position;

            if (math.lengthsq(targetPos - (float3)jointC.position) < 0.001f * 0.001f) return;

            float lengthAT = GetLengthAT(targetPos, aPos, lengthAB, lengthBC);
            if (math.length(targetPos - aPos) > lengthAT)
            {
                targetPos = aPos + math.normalize(targetPos - aPos) * lengthAT;
            }
            float3 axisAC = math.normalize(cPos - aPos);
            forward = math.normalize(forward);
            float3 rotationAxis = math.normalize(math.cross(axisAC, forward));
            // First make the vector AC have the same length as AT
            // Use the dot product formula to obtain interior angles
            float interiorAngleA = math.acos(math.clamp( // clamp to avoid numerical errors
                                                math.dot(
                                                    axisAC,
                                                    math.normalize(bPos - aPos)),
                                                -1f, 1f));
            float interiorAngleB = math.acos(math.clamp(
                                                math.dot(
                                                    math.normalize(aPos - bPos),
                                                    math.normalize(cPos - bPos)),
                                                -1f, 1f));
            // Use the cosine rule to get the desired interior angles
            Debug.Assert(!float.IsNaN(math.acos(math.clamp((lengthBC * lengthBC - lengthAB * lengthAB - lengthAT * lengthAT) / (-2f * lengthAB * lengthAT), -1f, 1f))), "Numerical error");
            Debug.Assert(!float.IsNaN(math.acos(math.clamp((lengthAT * lengthAT - lengthAB * lengthAB - lengthBC * lengthBC) / (-2f * lengthAB * lengthBC), -1f, 1f))), "Numerical error");
            float desiredInteriorAngleA = math.acos(math.clamp((lengthBC * lengthBC - lengthAB * lengthAB - lengthAT * lengthAT) / (-2f * lengthAB * lengthAT), -1f, 1f));
            float desiredInteriorAngleB = math.acos(math.clamp((lengthAT * lengthAT - lengthAB * lengthAB - lengthBC * lengthBC) / (-2f * lengthAB * lengthBC), -1f, 1f));
            // Local rotation angles for A and B
            quaternion rotA = quaternion.AxisAngle(math.mul(math.inverse(jointA.rotation), rotationAxis), desiredInteriorAngleA - interiorAngleA); // mul by the inverse to make it joint local space
            quaternion rotB = quaternion.AxisAngle(math.mul(math.inverse(jointB.rotation), rotationAxis), desiredInteriorAngleB - interiorAngleB);
            // Now we have the leg rotated so that the vector AC has the same length as AT
            // Then, we rotate A by the rotation axis formed by cross product of AC and AT
            // First, angle between AC and AT
            float3 axisAT = math.normalize(targetPos - aPos);
            float angleACAT = math.acos(math.clamp(
                                            math.dot(
                                                axisAC,
                                                axisAT),
                                            -1f, 1f));
            float3 rotationAxisACAT = math.normalize(math.cross(axisAC, axisAT));
            quaternion rotA2 = quaternion.AxisAngle(math.mul(math.inverse(jointA.rotation), rotationAxisACAT), angleACAT);
            // Apply the rotations
            jointA.rotation = math.mul(jointA.rotation, math.mul(rotA, rotA2));
            jointB.rotation = math.mul(jointB.rotation, rotB);
        }

        private static float GetLengthAT(float3 targetPos, float3 aPos, float lengthAB, float lengthBC)
        {
            const float epsilon = 0.001f;
            return math.clamp(math.distance(aPos, targetPos), epsilon, lengthAB + lengthBC - epsilon);
        }
    }
}