using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotionMatching;
using Unity.Mathematics;
using static MotionMatching.MotionMatchingData;

/// <summary>
/// Import a BVH, create PoseSet and FeatureSet and visualize it using Gizmos.
/// </summary>
public class FeatureDebug : MonoBehaviour
{
    public MotionMatchingData MMData;
    public bool Play;
    public float SpheresRadius = 0.1f;
    public bool LockFPS = true;
    public bool DebugTrajectory = true;
    public bool DebugPose = true;
    public bool DebugEnvironment = true;
    public bool DebugContacts = true;

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
        SkeletonTransforms[0] = transform; // Simulation Bone
        for (int j = 1; j < PoseSet.Skeleton.Joints.Count; j++)
        {
            // Joints
            Skeleton.Joint joint = PoseSet.Skeleton.Joints[j];
            Transform t = (new GameObject()).transform;
            t.name = joint.Name;
            t.SetParent(SkeletonTransforms[joint.ParentIndex], false);
            t.localPosition = joint.LocalOffset;
            SkeletonTransforms[j] = t;
        }

        // FPS
        if (LockFPS)
        {
            Application.targetFrameRate = Mathf.RoundToInt(1.0f / PoseSet.FrameTime);
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
            SkeletonTransforms[0].localPosition = pose.JointLocalPositions[0];
            SkeletonTransforms[1].localPosition = pose.JointLocalPositions[1];
            for (int i = 0; i < pose.JointLocalRotations.Length; i++)
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
        MMData.Dispose();
        PoseSet.Dispose();
        FeatureSet.Dispose();
    }

    private void OnApplicationQuit()
    {
        MMData.Dispose();
        PoseSet.Dispose();
        FeatureSet.Dispose();
    }


#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        // Skeleton
        if (SkeletonTransforms == null || PoseSet == null) return;

        Gizmos.color = Color.red;
        for (int i = 2; i < SkeletonTransforms.Length; i++) // skip Simulation Bone
        {
            Transform t = SkeletonTransforms[i];
            GizmosExtensions.DrawLine(t.parent.position, t.position, 3);
        }

        if (!Play) return;
        // Character
        int currentFrame = math.max(0, CurrentFrame - 1); // FeatureDebug increments CurrentFrame after update... OnDrawGizmos is called after update
        PoseSet.GetPose(currentFrame, out PoseVector pose);
        FeatureSet.GetWorldOriginCharacter(pose, out float3 characterOrigin, out float3 characterForward);
        Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
        Gizmos.DrawSphere(characterOrigin, SpheresRadius);
        GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward, thickness: 3);

        // Forward Trajectory Direction Features
        Gizmos.color = Color.gray;
        for (int t = 0; t < MMData.TrajectoryFeatures.Count; t++)
        {
            var trajectoryFeature = MMData.TrajectoryFeatures[t];
            if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Direction &&
                !trajectoryFeature.SimulationBone)
            {
                if (!PoseSet.Skeleton.Find(trajectoryFeature.Bone, out Skeleton.Joint joint)) Debug.Assert(false, "Bone not found");
                float3 dir = SkeletonTransforms[joint.Index].TransformDirection(MMData.GetLocalForward(joint.Index));
                float3 jointPos = SkeletonTransforms[joint.Index].position;
                GizmosExtensions.DrawArrow(jointPos, jointPos + dir * 0.5f, 0.1f, thickness: 3);
            }
        }

        // Contacts
        if (DebugContacts)
        {
            if (!PoseSet.Skeleton.Find(HumanBodyBones.LeftToes, out Skeleton.Joint leftToesJoint)) Debug.Assert(false, "Bone not found");
            if (!PoseSet.Skeleton.Find(HumanBodyBones.RightToes, out Skeleton.Joint rightToesJoint)) Debug.Assert(false, "Bone not found");
            int leftToesIndex = leftToesJoint.Index;
            int rightToesIndex = rightToesJoint.Index;
            Gizmos.color = Color.green;
            if (pose.LeftFootContact)
            {
                Gizmos.DrawSphere(SkeletonTransforms[leftToesIndex].position, SpheresRadius);
            }
            if (pose.RightFootContact)
            {
                Gizmos.DrawSphere(SkeletonTransforms[rightToesIndex].position, SpheresRadius);
            }
        }

        // Feature Set
        if (FeatureSet == null) return;

        DrawFeatureGizmos(FeatureSet, MMData, SpheresRadius, currentFrame, characterOrigin, characterForward,
                          SkeletonTransforms, PoseSet.Skeleton, Color.blue, debugPose: DebugPose, debugTrajectory: DebugTrajectory, debugEnvironment: DebugEnvironment);
    }

    private static List<float3> PositionFeatures = new();
    public static void DrawFeatureGizmos(FeatureSet set, MotionMatchingData mmData, float spheresRadius, int currentFrame,
                                         float3 characterOrigin, float3 characterForward, Transform[] joints, Skeleton skeleton,
                                         Color trajectoryColor, bool debugPose = true, bool debugTrajectory = true, bool debugEnvironment = true)
    {
        if (!set.IsValidFeature(currentFrame)) return;

        quaternion characterRot = quaternion.LookRotation(characterForward, math.up());

        // TODO: find a better way to store this information
        PositionFeatures.Clear();

        // Trajectory Features ---------------------------------------------------------------------------
        // Find the Main Position Feature (if exists)
        for (int t = 0; t < mmData.TrajectoryFeatures.Count; t++)
        {
            var trajectoryFeature = mmData.TrajectoryFeatures[t];
            if (trajectoryFeature.IsMainPositionFeature)
            {
                Debug.Assert(trajectoryFeature.FeatureType == TrajectoryFeature.Type.Position, "The main position feature should be of type Position");
                for (int p = 0; p < trajectoryFeature.FramesPrediction.Length; p++)
                {
                    float3 value = Get3DValuePositionOrDirectionFeature(trajectoryFeature, set, currentFrame, t, p, isEnvironment: false);
                    value = characterOrigin + math.mul(characterRot, value);
                    PositionFeatures.Add(value);
                }
            }
        }
        // Draw Trajectory Features
        if (debugTrajectory)
        {
            for (int t = 0; t < mmData.TrajectoryFeatures.Count; t++)
            {
                var trajectoryFeature = mmData.TrajectoryFeatures[t];
                for (int p = 0; p < trajectoryFeature.FramesPrediction.Length; p++)
                {
                    DrawTrajectoryPoint(trajectoryFeature, set, currentFrame, t, p, trajectoryColor, characterOrigin, characterForward,
                                        characterRot, spheresRadius, joints, skeleton, isEnvironment: false);
                }
            }
        }
        // Pose Features ---------------------------------------------------------------------------
        if (debugPose)
        {
            Gizmos.color = new Color(0.0f, 0.8f, 0.8f);
            for (int p = 0; p < mmData.PoseFeatures.Count; p++)
            {
                var poseFeature = mmData.PoseFeatures[p];
                float3 value = set.GetPoseFeature(currentFrame, p, true);
                switch (poseFeature.FeatureType)
                {
                    case PoseFeature.Type.Position:
                        value = characterOrigin + math.mul(characterRot, value);
                        Gizmos.DrawSphere(value, spheresRadius);
                        break;
                    case PoseFeature.Type.Velocity:
                        value = math.mul(characterRot, value);
                        if (math.length(value) > 0.001f)
                        {
                            skeleton.Find(poseFeature.Bone, out Skeleton.Joint joint);
                            float3 jointPos = joints[joint.Index].position;
                            GizmosExtensions.DrawArrow(jointPos, jointPos + value * 0.2f, 0.25f * math.length(value) * 0.2f, thickness: 4, useDepth: false);
                        }
                        break;
                }
            }
        }

        // Environment Features ---------------------------------------------------------------------------
        if (debugEnvironment)
        {
            for (int t = 0; t < mmData.EnvironmentFeatures.Count; t++)
            {
                var environmentFeature = mmData.EnvironmentFeatures[t];
                for (int p = 0; p < environmentFeature.FramesPrediction.Length; p++)
                {
                    DrawTrajectoryPoint(environmentFeature, set, currentFrame, t, p, trajectoryColor, characterOrigin, characterForward,
                                        characterRot, spheresRadius, joints, skeleton, isEnvironment: true);
                }
            }
        }
    }

    public static float3 Get3DValuePositionOrDirectionFeature(TrajectoryFeature trajectoryFeature, FeatureSet set, int currentFrame, int trajectoryFeatureIndex, int predictionIndex, bool isEnvironment)
    {
        int t = trajectoryFeatureIndex;
        int p = predictionIndex;

        float3 value;
        if (!trajectoryFeature.ZeroX && !trajectoryFeature.ZeroY && !trajectoryFeature.ZeroZ)
        {
            value = isEnvironment ? set.Get3DEnvironmentFeature(currentFrame, t, p) : set.Get3DTrajectoryFeature(currentFrame, t, p, true);
        }
        else if (!trajectoryFeature.ZeroX && !trajectoryFeature.ZeroY)
        {
            float2 value2D = isEnvironment ? set.Get2DEnvironmentFeature(currentFrame, t, p) : set.Get2DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(value2D.x, value2D.y, 0);
        }
        else if (!trajectoryFeature.ZeroX && !trajectoryFeature.ZeroZ)
        {
            float2 value2D = isEnvironment ? set.Get2DEnvironmentFeature(currentFrame, t, p) : set.Get2DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(value2D.x, 0.0f, value2D.y);
        }
        else if (!trajectoryFeature.ZeroY && !trajectoryFeature.ZeroZ)
        {
            float2 value2D = isEnvironment ? set.Get2DEnvironmentFeature(currentFrame, t, p) : set.Get2DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(0.0f, value2D.x, value2D.y);
        }
        else if (!trajectoryFeature.ZeroX)
        {
            float value1D = isEnvironment ? set.Get1DEnvironmentFeature(currentFrame, t, p) : set.Get1DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(value1D, 0.0f, 0.0f);
        }
        else if (!trajectoryFeature.ZeroY)
        {
            float value1D = isEnvironment ? set.Get1DEnvironmentFeature(currentFrame, t, p) : set.Get1DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(0.0f, value1D, 0.0f);
        }
        else if (!trajectoryFeature.ZeroZ)
        {
            float value1D = isEnvironment ? set.Get1DEnvironmentFeature(currentFrame, t, p) : set.Get1DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(0.0f, 0.0f, value1D);
        }
        else
        {
            Debug.Assert(false, "Invalid trajectory feature");
            value = float3.zero;
        }
        return value;
    }

    private static void DrawTrajectoryPoint(TrajectoryFeature trajectoryFeature, FeatureSet set, int currentFrame, int trajectoryFeatureIndex,
                                            int predictionIndex, Color trajectoryColor, float3 characterOrigin, float3 characterForward,
                                            quaternion characterRot, float spheresRadius, Transform[] joints, Skeleton skeleton, bool isEnvironment)
    {
        int t = trajectoryFeatureIndex;
        int p = predictionIndex;
        //Gizmos.color = trajectoryColor * (1.25f - (float)p / trajectoryFeature.FramesPrediction.Length);
        Gizmos.color = trajectoryColor + (new Color(1.0f, 1.0f, 1.0f) - trajectoryColor) * ((float)p / trajectoryFeature.FramesPrediction.Length);
        if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Position ||
            trajectoryFeature.FeatureType == TrajectoryFeature.Type.Direction)
        {
            float3 value = Get3DValuePositionOrDirectionFeature(trajectoryFeature, set, currentFrame, t, p, isEnvironment);
            switch (trajectoryFeature.FeatureType)
            {
                case TrajectoryFeature.Type.Position:
                    value = characterOrigin + math.mul(characterRot, value);
                    Gizmos.DrawSphere(value, spheresRadius);
                    break;
                case TrajectoryFeature.Type.Direction:
                    float3 jointPos;
                    if (trajectoryFeature.SimulationBone)
                    {
                        jointPos = PositionFeatures.Count > 0 ? PositionFeatures[p] : float3.zero;
                    }
                    else
                    {
                        if (!skeleton.Find(trajectoryFeature.Bone, out Skeleton.Joint joint)) Debug.Assert(false, "Bone not found");
                        jointPos = joints[joint.Index].position;
                    }
                    value = math.mul(characterRot, value);
                    //GizmosExtensions.DrawArrow(jointPos, jointPos + value, 0.1f, thickness: 3);
                    GizmosExtensions.DrawArrow(jointPos, jointPos + value * 0.4f, 0.15f, thickness: 4);
                    break;
            }
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom1D)
        {
            Feature1DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature1DExtractor;
            float value = isEnvironment ? set.Get1DEnvironmentFeature(currentFrame, t, p) : set.Get1DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, PositionFeatures.Count > 0 ? PositionFeatures[p] : float3.zero);
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom2D)
        {
            Feature2DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature2DExtractor;
            float2 value = isEnvironment ? set.Get2DEnvironmentFeature(currentFrame, t, p) : set.Get2DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, PositionFeatures.Count > 0 ? PositionFeatures[p] : float3.zero);
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom3D)
        {
            Feature3DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature3DExtractor;
            float3 value = isEnvironment ? set.Get3DEnvironmentFeature(currentFrame, t, p) : set.Get3DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, PositionFeatures.Count > 0 ? PositionFeatures[p] : float3.zero);
        }
        else if (trajectoryFeature.FeatureType == TrajectoryFeature.Type.Custom4D)
        {
            Feature4DExtractor featureExtractor = trajectoryFeature.FeatureExtractor as Feature4DExtractor;
            float4 value = isEnvironment ? set.Get4DEnvironmentFeature(currentFrame, t, p) : set.Get4DTrajectoryFeature(currentFrame, t, p, true);
            featureExtractor.DrawGizmos(value, spheresRadius, characterOrigin, characterForward, joints, skeleton, PositionFeatures.Count > 0 ? PositionFeatures[p] : float3.zero);
        }
    }
#endif
}
