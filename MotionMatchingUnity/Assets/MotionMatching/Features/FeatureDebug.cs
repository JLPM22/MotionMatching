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
    public MotionMatchingData MMData;
    public bool Play;
    public float SpheresRadius = 0.1f;
    public bool LockFPS = true;

    private PoseSet PoseSet;
    private FeatureSet FeatureSet;
    private Transform[] SkeletonTransforms;
    [SerializeField] private int CurrentFrame;

    private void Awake()
    {
        // PoseSet
        PoseSet = MMData.GetOrImportPoseSet();

        // FeatureSet
        FeatureSet = MMData.GetOrImportFeatureSet();

        // Skeleton
        SkeletonTransforms = new Transform[PoseSet.Skeleton.Joints.Count];
        foreach (Skeleton.Joint joint in PoseSet.Skeleton.Joints)
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
            Application.targetFrameRate = (int)(1.0f / PoseSet.FrameTime);
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
            PoseSet.GetPose(CurrentFrame, out PoseVector pose);
            SkeletonTransforms[0].localPosition = pose.RootWorld;
            SkeletonTransforms[0].localRotation = pose.RootWorldRot;
            for (int i = 1; i < pose.JointLocalRotations.Length; i++)
            {
                SkeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            }
            CurrentFrame = (CurrentFrame + 1) % PoseSet.NumberPoses;
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
        if (SkeletonTransforms == null || PoseSet == null) return;

        Gizmos.color = Color.red;
        for (int i = 1; i < SkeletonTransforms.Length; i++)
        {
            Transform t = SkeletonTransforms[i];
            Gizmos.DrawLine(t.parent.position, t.position);
        }

        if (!Play) return;
        // Character
        int currentFrame = CurrentFrame;
        PoseSet.GetPose(currentFrame, out PoseVector pose);
        FeatureSet.GetWorldOriginCharacter(pose.RootWorld, pose.RootWorldRot, MMData.HipsForwardLocalVector, out float3 characterOrigin, out float3 characterForward);
        Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
        Gizmos.DrawSphere(characterOrigin, SpheresRadius);
        GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward);

        // Feature Set
        if (FeatureSet == null) return;

        DrawFeatureGizmos(FeatureSet, MMData, SpheresRadius, currentFrame, characterOrigin, characterForward,
                          SkeletonTransforms, PoseSet.Skeleton);
    }

    public static void DrawFeatureGizmos(FeatureSet set, MotionMatchingData mmData, float spheresRadius, int currentFrame,
                                         float3 characterOrigin, float3 characterForward, Transform[] joints, Skeleton skeleton,
                                         bool debugPose = true, bool debugTrajectory = true)
    {
        if (!set.IsValidFeature(currentFrame)) return;

        quaternion characterRot = quaternion.LookRotation(characterForward, new float3(0, 1, 0));
        // Trajectory
        if (debugTrajectory)
        {
            Gizmos.color = Color.blue;
            for (int t = 0; t < mmData.TrajectoryFeatures.Count; t++)
            {
                var trajectoryFeature = mmData.TrajectoryFeatures[t];
                for (int p = 0; p < trajectoryFeature.FramesPrediction.Length; p++)
                {
                    float3 value;
                    if (trajectoryFeature.Project)
                    {
                        float2 value2D = set.GetProjectedTrajectoryFeature(currentFrame, t, p, true);
                        value = new float3(value2D.x, 0.0f, value2D.y);
                    }
                    else
                    {
                        value = set.GetTrajectoryFeature(currentFrame, t, p, true);
                    }
                    switch (trajectoryFeature.FeatureType)
                    {
                        case MotionMatchingData.TrajectoryFeature.Type.Position:
                            value = characterOrigin + math.mul(characterRot, value);
                            Gizmos.DrawSphere(value, spheresRadius);
                            break;
                        case MotionMatchingData.TrajectoryFeature.Type.Direction:
                            value = math.mul(characterRot, value);
                            GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + value * 0.25f, 0.1f);
                            break;
                    }
                }
            }
        }
        // Pose
        if (debugPose)
        {
            Gizmos.color = Color.cyan;
            for (int p = 0; p < mmData.PoseFeatures.Count; p++)
            {
                var poseFeature = mmData.PoseFeatures[p];
                float3 value = set.GetPoseFeature(currentFrame, p, true);
                switch (poseFeature.FeatureType)
                {
                    case MotionMatchingData.PoseFeature.Type.Position:
                        value = characterOrigin + math.mul(characterRot, value);
                        Gizmos.DrawWireSphere(value, spheresRadius);
                        break;
                    case MotionMatchingData.PoseFeature.Type.Velocity:
                        value = math.mul(characterRot, value);
                        if (math.length(value) > 0.001f)
                        {
                            skeleton.Find(poseFeature.Bone, out Skeleton.Joint joint);
                            float3 jointPos = joints[joint.Index].position;
                            GizmosExtensions.DrawArrow(jointPos, jointPos + value * 0.1f, 0.25f * math.length(value) * 0.1f);
                        }
                        break;
                }
            }
        }
    }
#endif
}
