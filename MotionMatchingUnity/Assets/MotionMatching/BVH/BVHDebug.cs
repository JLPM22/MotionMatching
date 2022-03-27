using System.Collections;
using System.Collections.Generic;
using MotionMatching;
using UnityEngine;

/// <summary>
/// Import a BVH and visualize it using Gizmos.
/// </summary>
public class BVHDebug : MonoBehaviour
{
    public TextAsset BVH;
    public float UnitScale = 1;
    public int BVHIndex = 0;
    public bool Play;
    public float SpheresRadius = 0.1f;
    public bool LockFPS = true;

    private BVHAnimation Animation;
    private Transform[] Skeleton;
    private int CurrentFrame;

    private void Awake()
    {
        BVHImporter importer = new BVHImporter();
        Animation = importer.Import(BVH, UnitScale);

        Skeleton = new Transform[Animation.Skeleton.Joints.Count];
        foreach (Skeleton.Joint joint in Animation.Skeleton.Joints)
        {
            Transform t = (new GameObject()).transform;
            t.name = joint.Name;
            if (joint.Index == 0) t.SetParent(transform, false);
            else t.SetParent(Skeleton[joint.ParentIndex], false);
            t.localPosition = joint.LocalOffset;
            Skeleton[joint.Index] = t;
        }

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
            Skeleton[0].localPosition = frame.RootMotion;
            for (int i = 0; i < frame.LocalRotations.Length; i++)
            {
                Skeleton[i].localRotation = frame.LocalRotations[i];
            }
            CurrentFrame = (CurrentFrame + 1) % Animation.Frames.Length;
        }
        else
        {
            CurrentFrame = 0;
            Skeleton[0].localPosition = Vector3.zero;
            for (int i = 0; i < Skeleton.Length; i++)
            {
                Skeleton[i].localRotation = Quaternion.identity;
            }
        }
    }

#if UNITY_EDITOR
    private void OnDrawGizmos()
    {
        if (Skeleton == null || Animation == null || Animation.EndSites == null) return;

        Gizmos.color = Color.red;
        for (int i = 1; i < Skeleton.Length; i++)
        {
            Transform t = Skeleton[i];
            GizmosExtensions.DrawLine(t.parent.position, t.position, 3);
        }
        // Uncomment to show end sites
        // foreach (BVHAnimation.EndSite endSite in Animation.EndSites)
        // {
        //     Transform t = Skeleton[endSite.ParentIndex];
        //     GizmosExtensions.DrawLine(t.position, t.TransformPoint(endSite.Offset), 3);
        // }

        Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
        foreach (Transform t in Skeleton)
        {
            if (t.name == "End Site") continue;
            Gizmos.DrawWireSphere(t.position, SpheresRadius);
        }
    }
#endif
}
