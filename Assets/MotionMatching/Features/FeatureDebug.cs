using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotionMatching;
using Unity.Mathematics;

/// <summary>
/// Import a BVH, create PoseSet and FeatureSet and visualize it using Gizmos.
/// </summary>
public class FeatureDebug : MonoBehaviour
{
    public TextAsset BVH;
    public bool Play;
    public float UnitScale = 1;
    public float SpheresRadius = 0.1f;
    public float VectorLength = 1.0f;
    public bool LockFPS = true;
    public string LeftFootName, RightFootName, HipsName;
    public Vector3 DefaultHipsForward = new Vector3(0, 0, 1);

    private BVHAnimation Animation;
    private PoseSet PoseSet;
    private FeatureSet FeatureSet;
    private Transform[] SkeletonTransforms;
    private int CurrentFrame;

    private void Awake()
    {
        BVHImporter importer = new BVHImporter();
        Animation = importer.Import(BVH, UnitScale);

        // HARDCODED: hardcode name of important joints for the feature set
        for (int i = 0; i < Animation.Skeleton.Joints.Count; ++i)
        {
            Skeleton.Joint j = Animation.Skeleton.Joints[i];
            if (j.Name == LeftFootName)
            {
                j.Type = Skeleton.JointType.LeftFoot;
                Animation.Skeleton.Joints[i] = j;
            }
            else if (j.Name == RightFootName)
            {
                j.Type = Skeleton.JointType.RightFoot;
                Animation.Skeleton.Joints[i] = j;
            }
            else if (j.Name == HipsName)
            {
                j.Type = Skeleton.JointType.Hips;
                Animation.Skeleton.Joints[i] = j;
            }
        }

        PoseExtractor poseExtractor = new PoseExtractor();
        PoseSet = new PoseSet();
        if (!poseExtractor.Extract(Animation, PoseSet, DefaultHipsForward))
        {
            Debug.LogError("[FeatureDebug] Failed to extract pose from BVHAnimation");
        }

        FeatureExtractor featureExtractor = new FeatureExtractor();
        FeatureSet = featureExtractor.Extract(PoseSet, DefaultHipsForward);

        // Skeleton
        SkeletonTransforms = new Transform[Animation.Skeleton.Joints.Count];
        foreach (Skeleton.Joint joint in Animation.Skeleton.Joints)
        {
            Transform t = (new GameObject()).transform;
            t.name = joint.Name;
            if (joint.Index == 0) t.SetParent(transform, false);
            else t.SetParent(SkeletonTransforms[joint.ParentIndex], false);
            t.localPosition = joint.LocalOffset;
            SkeletonTransforms[joint.Index] = t;
        }

        // FPS
        if (LockFPS)
        {
            Application.targetFrameRate = (int)(1.0f / Animation.FrameTime);
            Debug.Log("[BVHDebug] Updated Target FPS: " + Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
        }
    }

    private void Update()
    {
        if (Play)
        {
            BVHAnimation.Frame frame = Animation.Frames[CurrentFrame];
            SkeletonTransforms[0].localPosition = frame.RootMotion;
            for (int i = 0; i < frame.LocalRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = frame.LocalRotations[i];
            }
            CurrentFrame = (CurrentFrame + 1) % Animation.Frames.Length;
        }
        else
        {
            CurrentFrame = 0;
            SkeletonTransforms[0].localPosition = float3.zero;
            for (int i = 0; i < SkeletonTransforms.Length; i++)
            {
                SkeletonTransforms[i].localRotation = quaternion.identity;
            }
        }
    }

    private void OnDestroy()
    {
        FeatureSet.Dispose();
    }

    private void OnApplicationQuit()
    {
        FeatureSet.Dispose();
    }


#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        // Skeleton
        if (SkeletonTransforms == null || Animation == null || Animation.EndSites == null) return;

        Gizmos.color = Color.red;
        for (int i = 1; i < SkeletonTransforms.Length; i++)
        {
            Transform t = SkeletonTransforms[i];
            Gizmos.DrawLine(t.parent.position, t.position);
        }
        foreach (BVHAnimation.EndSite endSite in Animation.EndSites)
        {
            Transform t = SkeletonTransforms[endSite.ParentIndex];
            Gizmos.DrawLine(t.position, t.TransformPoint(endSite.Offset));
        }

        // Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
        // foreach (Transform t in SkeletonTransforms)
        // {
        //     if (t.name == "End Site") continue;
        //     Gizmos.DrawWireSphere(t.position, SpheresRadius);
        // }

        if (!Play) return;
        // Character
        if (PoseSet == null) return;
        int currentFrame = CurrentFrame - 1; // OnDrawGizmos is called after Update
        if (currentFrame < 0) currentFrame = Animation.Frames.Length - 1;
        PoseVector pose = PoseSet.Poses[currentFrame];
        FeatureExtractor.GetWorldOriginCharacter(pose.RootWorld, pose.RootWorldRot, DefaultHipsForward, out float3 characterOrigin, out float3 characterForward);
        Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
        Gizmos.DrawWireSphere(characterOrigin, SpheresRadius);
        Debug.Assert(math.length(characterForward) > 0.99f && math.length(characterForward) < 1.01f, "characterForward.magnitude = " + math.length(characterForward) + " should be 1");
        GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward * VectorLength, 0.25f * VectorLength);

        // Feature Set
        if (FeatureSet == null) return;

        FeatureVector fv = FeatureSet.GetFeature(currentFrame);
        if (fv.Valid)
        {
            quaternion characterRot = quaternion.LookRotation(characterForward, new float3(0, 1, 0));
            // Left Foot
            Gizmos.color = Color.cyan;
            float3 leftFootWorld = characterOrigin + math.mul(characterRot, fv.LeftFootLocalPosition);
            Gizmos.DrawWireSphere(leftFootWorld, SpheresRadius);
            float3 leftFootVelWorld = math.mul(characterRot, fv.LeftFootLocalVelocity);
            GizmosExtensions.DrawArrow(leftFootWorld, leftFootWorld + leftFootVelWorld * 0.1f, 0.25f * math.length(leftFootVelWorld) * 0.1f);
            // Right Foot
            Gizmos.color = Color.yellow;
            float3 rightFootWorld = characterOrigin + math.mul(characterRot, fv.RightFootLocalPosition);
            Gizmos.DrawWireSphere(rightFootWorld, SpheresRadius);
            float3 rightFootVelWorld = math.mul(characterRot, fv.RightFootLocalVelocity);
            GizmosExtensions.DrawArrow(rightFootWorld, rightFootWorld + rightFootVelWorld * 0.1f, 0.25f * math.length(rightFootVelWorld) * 0.1f);
            // Hips
            Gizmos.color = Color.green;
            float3 hipsVelWorld = math.mul(characterRot, fv.HipsLocalVelocity);
            GizmosExtensions.DrawArrow(pose.RootWorld, pose.RootWorld + hipsVelWorld * 0.1f, 0.25f * math.length(hipsVelWorld) * 0.1f);
            // Trajectory
            for (int i = 0; i < FeatureVector.GetFutureTrajectoryLength(); ++i)
            {
                Gizmos.color = Color.blue * (1.0f - (float)i / (FeatureVector.GetFutureTrajectoryLength() * 1.25f));
                float2 futurePos = fv.GetFutureTrajectoryLocalPosition(i);
                float3 futureWorld = characterOrigin + math.mul(characterRot, (new float3(futurePos.x, 0.0f, futurePos.y)));
                Gizmos.DrawWireSphere(futureWorld, SpheresRadius);
                float2 futureDir = fv.GetFutureTrajectoryLocalDirection(i);
                float3 futureDirWorld = math.mul(characterRot, (new float3(futureDir.x, 0.0f, futureDir.y)));
                GizmosExtensions.DrawArrow(futureWorld, futureWorld + futureDirWorld * VectorLength, 0.25f * VectorLength);
            }
        }
    }
#endif
}
