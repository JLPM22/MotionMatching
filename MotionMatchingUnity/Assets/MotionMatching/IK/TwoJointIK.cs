using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

public static class TwoJointIK
{
    /// <summary>
    /// Solve 2-joint IK considering target as end effector for the joint c.
    /// It is not limited to legs, but for example:
    /// a can be seen as the root/hips, b as the knee, c as the ankle,
    /// and the forward can be seen as the knee forward.
    /// </summary>
    public static void Solve(float3 targetPos, Transform jointA, Transform jointB, Transform jointC, float3 polePosition)
    {
        float lengthAB = math.distance(jointA.position, jointB.position);
        float lengthBC = math.distance(jointB.position, jointC.position);

        float3 aPos = jointA.position;
        float3 bPos = jointB.position;
        float3 cPos = jointC.position;

        // Correct bPos
        // There are infinite solutions (rotation axis vector AC)
        // if Pole is defined we can find a solution in which B is pointing to Pole
        float3 rotationAxisPole = math.normalize(cPos - aPos);
        /* Find angle of rotation between two vectors given a specific axis:
        https://math.stackexchange.com/questions/2548811/find-an-angle-to-rotate-a-vector-around-a-ray-so-that-the-vector-gets-as-close-a */
        float3 a = math.normalize(bPos - aPos);
        float3 b = math.normalize(polePosition - aPos);
        float3 e = math.normalize(a - math.dot(a, rotationAxisPole) * rotationAxisPole);
        float3 f = math.cross(rotationAxisPole, e);
        float anglePole = math.atan2(math.dot(b, f), math.dot(b, e));
        quaternion rotPole = quaternion.AxisAngle(math.mul(math.inverse(jointA.rotation), rotationAxisPole), anglePole);
        quaternion correctedJointARot = math.mul(jointA.rotation, rotPole);
        float3 newbPos = math.mul(correctedJointARot, math.mul(math.inverse(jointA.rotation), bPos - aPos)) + aPos;
        if (math.length(newbPos - bPos) > 0.001f)
        {
            // Rotate A so that JointB actually corresponds to newbPos
            float3 newBA = math.normalize(newbPos - aPos);
            float3 rotAxis = math.normalize(math.cross(a, newBA));
            float angle = math.acos(math.clamp(math.dot(a, newBA), -1, 1));
            quaternion poleRotA = quaternion.AxisAngle(math.mul(math.inverse(jointA.rotation), rotAxis), angle);
            jointA.rotation = math.mul(jointA.rotation, poleRotA);
            bPos = jointB.position; // newbPos
            Debug.Assert(math.length(newbPos - bPos) < 0.001f, "New bPos is not equal to bPos. New bPos = " + newbPos + ", bPos = " + bPos);
            // Rotate B so that JointC corresponds to cPos (it was modified when A was rotated)
            float3 newCB = math.normalize((float3)jointC.position - bPos);
            float3 oldCB = math.normalize(cPos - bPos);
            rotAxis = math.normalize(math.cross(newCB, oldCB));
            angle = math.acos(math.clamp(math.dot(newCB, oldCB), -1, 1));
            quaternion poleRotB = quaternion.AxisAngle(math.mul(math.inverse(jointB.rotation), rotAxis), angle);
            jointB.rotation = math.mul(jointB.rotation, poleRotB);
            Debug.Assert(math.length((float3)jointC.position - cPos) < 0.001f, "JointC position is not equal to cPos. JointC position = " + jointC.position + ", cPos = " + cPos);
        }

        if (math.lengthsq(targetPos - (float3)jointC.position) < 0.001f * 0.001f) return;

        float lengthAT = GetLengthAT(targetPos, aPos, lengthAB, lengthBC);
        // First make the vector AC have the same length as AT
        // Use the dot product formula to obtain interior angles
        float interiorAngleA = math.acos(math.clamp( // clamp to avoid numerical errors
                                            math.dot(
                                                math.normalize(cPos - aPos),
                                                math.normalize(bPos - aPos)),
                                            -1f, 1f));
        float interiorAngleB = math.acos(math.clamp(
                                            math.dot(
                                                math.normalize(aPos - bPos),
                                                math.normalize(cPos - bPos)),
                                            -1f, 1f));
        // Use the cosine rule to get the desired interior angles
        float desiredInteriorAngleA = math.acos(math.clamp((lengthBC * lengthBC - lengthAB * lengthAB - lengthAT * lengthAT) / (-2f * lengthAB * lengthAT), -1f, 1f));
        float desiredInteriorAngleB = math.acos(math.clamp((lengthAT * lengthAT - lengthAB * lengthAB - lengthBC * lengthBC) / (-2f * lengthAB * lengthBC), -1f, 1f));
        // Local rotation angles for A and B
        float3 rotationAxis = math.normalize(math.cross(cPos - aPos, bPos - aPos));
        quaternion rotA = quaternion.AxisAngle(math.mul(math.inverse(jointA.rotation), rotationAxis), desiredInteriorAngleA - interiorAngleA); // mul by the inverse to make it joint local space
        quaternion rotB = quaternion.AxisAngle(math.mul(math.inverse(jointB.rotation), rotationAxis), desiredInteriorAngleB - interiorAngleB);
        // Now we have the leg rotated so that the vector AC has the same length as AT
        // Then, we rotate A by the rotation axis formed by cross product of AC and AT
        // First, angle between AC and AT
        float angleACAT = math.acos(math.clamp(
                                        math.dot(
                                            math.normalize(cPos - aPos),
                                            math.normalize(targetPos - aPos)),
                                        -1f, 1f));
        float3 rotationAxisACAT = math.normalize(math.cross(cPos - aPos, targetPos - aPos));
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
