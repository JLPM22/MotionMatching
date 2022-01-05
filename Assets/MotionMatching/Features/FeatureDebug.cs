using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotionMatching;

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

    private BVHAnimation Animation;
    private PoseSet PoseSet;
    private FeatureSet FeatureSet;
    private Transform[] SkeletonTransforms;
    private int CurrentFrame;

    private void Awake()
    {
        BVHImporter importer = new BVHImporter();
        Animation = importer.Import(BVH);

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
        if (!poseExtractor.Extract(Animation, PoseSet))
        {
            Debug.LogError("[FeatureDebug] Failed to extract pose from BVHAnimation");
        }

        FeatureExtractor featureExtractor = new FeatureExtractor();
        FeatureSet = featureExtractor.Extract(PoseSet);

        // Skeleton
        SkeletonTransforms = new Transform[Animation.Skeleton.Joints.Count];
        foreach (Skeleton.Joint joint in Animation.Skeleton.Joints)
        {
            Transform t = (new GameObject()).transform;
            t.name = joint.Name;
            if (joint.Index == 0) t.SetParent(transform, false);
            else t.SetParent(SkeletonTransforms[joint.ParentIndex], false);
            t.localPosition = joint.LocalOffset * UnitScale;
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
            SkeletonTransforms[0].localPosition = frame.RootMotion * UnitScale;
            for (int i = 0; i < frame.LocalRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = frame.LocalRotations[i];
            }
            CurrentFrame = (CurrentFrame + 1) % Animation.Frames.Length;
        }
        else
        {
            CurrentFrame = 0;
            SkeletonTransforms[0].localPosition = Vector3.zero;
            for (int i = 0; i < SkeletonTransforms.Length; i++)
            {
                SkeletonTransforms[i].localRotation = Quaternion.identity;
            }
        }
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
            Gizmos.DrawLine(t.position, t.TransformPoint(endSite.Offset * UnitScale));
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
        FeatureExtractor.GetWorldOriginCharacter(pose.RootWorld, pose.RootWorldRot, out Vector3 characterOrigin, out Vector3 characterForward);
        Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
        Gizmos.DrawWireSphere(characterOrigin, SpheresRadius);
        Debug.Assert(characterForward.magnitude > 0.99f && characterForward.magnitude < 1.01f, "characterForward.magnitude = " + characterForward.magnitude + " should be 1");
        GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward * VectorLength, 0.25f * VectorLength);

        // Feature Set
        if (FeatureSet == null) return;

        FeatureVector fv = FeatureSet.Features[currentFrame];
        if (fv.Valid)
        {
            Quaternion characterRot = Quaternion.LookRotation(characterForward, Vector3.up);
            // Left Foot
            Gizmos.color = Color.cyan;
            Vector3 leftFootWorld = characterOrigin + characterRot * fv.LeftFootLocalPosition;
            Gizmos.DrawWireSphere(leftFootWorld, SpheresRadius);
            Vector3 leftFootVelWorld = characterRot * fv.LeftFootLocalVelocity;
            GizmosExtensions.DrawArrow(leftFootWorld, leftFootWorld + leftFootVelWorld * 0.1f, 0.25f * leftFootVelWorld.magnitude * 0.1f);
            // Right Foot
            Gizmos.color = Color.yellow;
            Vector3 rightFootWorld = characterOrigin + characterRot * fv.RightFootLocalPosition;
            Gizmos.DrawWireSphere(rightFootWorld, SpheresRadius);
            Vector3 rightFootVelWorld = characterRot * fv.RightFootLocalVelocity;
            GizmosExtensions.DrawArrow(rightFootWorld, rightFootWorld + rightFootVelWorld * 0.1f, 0.25f * rightFootVelWorld.magnitude * 0.1f);
            // Hips
            Gizmos.color = Color.green;
            Vector3 hipsVelWorld = characterRot * fv.HipsLocalVelocity;
            GizmosExtensions.DrawArrow(pose.RootWorld, pose.RootWorld + hipsVelWorld * 0.1f, 0.25f * hipsVelWorld.magnitude * 0.1f);
            // Trajectory
            for (int i = 0; i < fv.FutureTrajectoryLocalPosition.Length; ++i)
            {
                Gizmos.color = Color.blue * (1.0f - (float)i / (fv.FutureTrajectoryLocalPosition.Length * 1.25f));
                Vector2 futurePos = fv.FutureTrajectoryLocalPosition[i];
                Vector3 futureWorld = characterOrigin + characterRot * (new Vector3(futurePos.x, 0.0f, futurePos.y));
                Gizmos.DrawWireSphere(futureWorld, SpheresRadius);
                Vector2 futureDir = fv.FutureTrajectoryLocalDirection[i];
                Vector3 futureDirWorld = characterRot * (new Vector3(futureDir.x, 0.0f, futureDir.y));
                GizmosExtensions.DrawArrow(futureWorld, futureWorld + futureDirWorld * VectorLength, 0.25f * VectorLength);
            }
        }
    }
#endif
}
