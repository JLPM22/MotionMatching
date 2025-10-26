using UnityEngine;
using MotionMatching;
using Unity.Mathematics;

/*
    This is an example class of a custom feature extractor for two-dimensional features.
    This class extracts the 2D position of the head joint in the local space of the character.
*/
[CreateAssetMenu(fileName = "NewCustom2DFeatureExtractor", menuName = "Custom/Custom2DFeatureExtractor")]
public class Custom2DFeatureExtractor : Feature2DExtractor
{
    private Skeleton.Joint HeadJoint;

    // This function is called once at the beginning of the extraction process.
    public override void StartExtracting(Skeleton skeleton)
    {
        bool found = skeleton.Find(HumanBodyBones.Head, out HeadJoint);
        Debug.Assert(found, "Head Joint could not be found");
    }

    // This function is called for each pose in the pose database.
    public override float2 ExtractFeature(PoseVector pose, int poseIndex, PoseVector nextPose, int animationClip, Skeleton skeleton, float3 characterOrigin, float3 characterForward)
    {
        float3 worldPos = FeatureSet.GetWorldPosition(skeleton, pose, HeadJoint);
        float3 localPos = FeatureSet.GetLocalPositionFromCharacter(worldPos, characterOrigin, characterForward);
        return localPos.xz;
    }

    // This function is called when features are visualized during gizmo drawing. It can be left empty if visualization is not needed.
    public override void DrawGizmos(float2 feature, float radius, float3 characterOrigin, float3 characterForward, Transform[] joints, Skeleton skeleton, float3 posOffset)
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
