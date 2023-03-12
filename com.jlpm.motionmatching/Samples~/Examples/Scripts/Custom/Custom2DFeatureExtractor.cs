using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotionMatching;
using Unity.Mathematics;

public class Custom2DFeatureExtractor : IFeatureExtractor2D
{
    private Skeleton.Joint HeadJoint;

    public void StartExtracting(Skeleton skeleton)
    {
        bool found = skeleton.Find(HumanBodyBones.Head, out HeadJoint);
        Debug.Assert(found, "Head Joint could not be found");
    }

    public float2 ExtractFeature(PoseVector pose, int poseIndex, Skeleton skeleton, float3 characterOrigin, float3 characterForward)
    {
        float3 worldPos = FeatureSet.GetWorldPosition(skeleton, pose, HeadJoint);
        float3 localPos = FeatureSet.GetLocalPositionFromCharacter(worldPos, characterOrigin, characterForward);
        return localPos.xz;
    }
    
    public void DrawGizmos(float2 feature, float radius, float3 characterOrigin, float3 characterForward, Transform[] joints, Skeleton skeleton)
    {
        bool found = skeleton.Find(HumanBodyBones.Head, out HeadJoint);
        Debug.Assert(found, "Head Joint could not be found");
        float3 headPos = joints[HeadJoint.Index].position;
        headPos.x = feature.x;
        headPos.z = feature.y;
        quaternion characterRot = quaternion.LookRotation(characterForward, new float3(0, 1, 0));
        float3 worldHeadPos = characterOrigin + math.mul(characterRot, headPos);
        GizmosExtensions.DrawWireSphere(worldHeadPos, radius);
    }
}
