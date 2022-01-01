using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    // Simulation bone is the transform
    public class MotionMatchingController : MonoBehaviour
    {
        public SpringCharacterController CharacterController;
        public TextAsset BVH;
        public float SpheresRadius = 0.1f;
        public bool LockFPS = true;
        public string LeftFootName, RightFootName, HipsName;
        public int SearchFrames = 10; // Motion Matching every SearchFrames frames
        public FeatureSet.NormalizeType NormalizeType = FeatureSet.NormalizeType.Magnitude;

        private BVHAnimation Animation;
        private PoseSet PoseSet;
        private FeatureSet FeatureSet;
        private Transform[] SkeletonTransforms;
        private int CurrentFrame;
        private int SearchFrameCount;
        private FeatureVector QueryFeature;

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
            // FeatureSet.NormalizeFeatures(NormalizeType);

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

        private void OnEnable()
        {
            SearchFrameCount = 0;
            CharacterController.OnUpdate += OnUpdate;
        }

        private void OnDisable()
        {
            CharacterController.OnUpdate -= OnUpdate;
        }

        private void Start()
        {
            QueryFeature = new FeatureVector();
            QueryFeature.FutureTrajectoryLocalDirection = new Vector2[CharacterController.NumberPrediction];
            QueryFeature.FutureTrajectoryLocalPosition = new Vector2[CharacterController.NumberPrediction];
        }

        private void OnUpdate(float deltaTime)
        {
            if (SearchFrameCount == 0)
            {
                // Motion Matching
                int nextFrame = SearchMotionMatching();
                if (nextFrame != CurrentFrame)
                {
                    CurrentFrame = nextFrame;
                }
                else
                {
                    CurrentFrame += 1;
                }
                SearchFrameCount = SearchFrames;
            }
            else
            {
                // Advance
                CurrentFrame += 1;
                SearchFrameCount -= 1;
            }
            UpdateFrame(CurrentFrame);
        }

        private int SearchMotionMatching()
        {
            // Init Query Vector
            for (int i = 0; i < CharacterController.NumberPrediction; ++i)
            {
                QueryFeature.FutureTrajectoryLocalPosition[i] = GetPositionLocalCharacter(CharacterController.GetWorldPredictedPosition(i));
                QueryFeature.FutureTrajectoryLocalDirection[i] = GetDirectionLocalCharacter(CharacterController.GetWorldPredictedDirection(i));
            }
            FeatureVector current = FeatureSet.Features[CurrentFrame];
            QueryFeature.LeftFootLocalPosition = current.LeftFootLocalPosition;
            QueryFeature.LeftFootLocalVelocity = current.LeftFootLocalVelocity;
            QueryFeature.RightFootLocalPosition = current.RightFootLocalPosition;
            QueryFeature.RightFootLocalVelocity = current.RightFootLocalVelocity;
            QueryFeature.HipsLocalVelocity = current.HipsLocalVelocity;
            // Normalize
            // FeatureSet.NormalizeFeatureVector(ref QueryFeature);
            // Search
            float min = float.MaxValue;
            int minIndex = -1;
            for (int i = 0; i < FeatureSet.Features.Length; ++i)
            {
                FeatureVector feature = FeatureSet.Features[i];
                if (feature.Valid)
                {
                    float dist = QueryFeature.SqrDistance(feature);
                    if (dist < min)
                    {
                        min = dist;
                        minIndex = i;
                    }
                }
            }
            // DEBUG
            Debug.Log(Time.frameCount + " --- ");
            for (int i = 0; i < QueryFeature.FutureTrajectoryLocalPosition.Length; ++i)
            {
                Debug.Log(i + " - ");
                Debug.Log(QueryFeature.FutureTrajectoryLocalPosition[0].ToString());
                Debug.Log(FeatureSet.Features[minIndex].FutureTrajectoryLocalPosition[0].ToString());
            }
            return minIndex;
        }

        private void UpdateFrame(int frameIndex)
        {
            PoseVector pose = PoseSet.Poses[frameIndex];
            // Simulation Bone
            transform.position += transform.TransformDirection(pose.RootVelocity);
            transform.rotation = transform.rotation * pose.RootRotVelocity; // FIXME: rotation should be only around the Y axis
            // Joints
            // for (int i = 0; i < pose.JointLocalRotations.Length; i++)
            // {
            //     SkeletonTransforms[i].localRotation = pose.JointLocalRotations[i];
            // }
        }

        private Vector2 GetPositionLocalCharacter(Vector2 worldPosition)
        {
            Vector3 localPosition = transform.InverseTransformPoint(new Vector3(worldPosition.x, 0.0f, worldPosition.y));
            return new Vector2(localPosition.x, localPosition.z);
        }

        private Vector2 GetDirectionLocalCharacter(Vector2 worldDir)
        {
            Vector3 localDir = transform.InverseTransformDirection(new Vector3(worldDir.x, 0.0f, worldDir.y));
            return new Vector2(localDir.x, localDir.z);
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

            // Character
            if (PoseSet == null) return;

            int currentFrame = CurrentFrame - 1; // OnDrawGizmos is called after Update
            if (currentFrame < 0) currentFrame = Animation.Frames.Length - 1;
            PoseVector pose = PoseSet.Poses[currentFrame];
            Vector3 characterOrigin = transform.position;
            Vector3 characterForward = transform.forward;
            Gizmos.color = new Color(1.0f, 0.0f, 0.5f, 1.0f);
            Gizmos.DrawWireSphere(characterOrigin, SpheresRadius);
            GizmosExtensions.DrawArrow(characterOrigin, characterOrigin + characterForward);

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
                GizmosExtensions.DrawArrow(pose.JointLocalPositions[0], pose.JointLocalPositions[0] + hipsVelWorld * 0.1f, 0.25f * hipsVelWorld.magnitude * 0.1f);
                // Trajectory
                for (int i = 0; i < fv.FutureTrajectoryLocalPosition.Length; ++i)
                {
                    Gizmos.color = Color.blue * (1.0f - (float)i / (fv.FutureTrajectoryLocalPosition.Length * 1.25f));
                    Vector2 futurePos = fv.FutureTrajectoryLocalPosition[i];
                    Vector3 futureWorld = characterOrigin + characterRot * (new Vector3(futurePos.x, 0.0f, futurePos.y));
                    Gizmos.DrawWireSphere(futureWorld, SpheresRadius);
                    Vector2 futureDir = fv.FutureTrajectoryLocalDirection[i];
                    Vector3 futureDirWorld = characterRot * (new Vector3(futureDir.x, 0.0f, futureDir.y));
                    GizmosExtensions.DrawArrow(futureWorld, futureWorld + futureDirWorld);
                }
            }
        }
#endif
    }
}