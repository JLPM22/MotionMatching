using MotionMatching;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    public class Obstacle : MonoBehaviour
    {
        public float Radius = 1.0f;
        public bool IsStatic = false;
        // Used for steering coordination, can be left null if not used
        public CrowdSplineCharacterController CrowdSplineCharacterController;
        public CrowdCharacterController CrowdCharacter;

        private void OnEnable()
        {
            ObstacleManager.Instance.RegisterObstacle(this);
        }

        private void OnDisable()
        {
            ObstacleManager.Instance.UnregisterObstacle(this);
        }

        public bool GetCurrentSteering(out float2 steering)
        {
            if (CrowdSplineCharacterController != null)
            {
                steering = CrowdSplineCharacterController.Steering;
                return true;
            }
            else if (CrowdCharacter != null)
            {
                steering = CrowdCharacter.Steering;
                return true;
            }
            steering = float2.zero;
            return false;
        }

        public Vector3 GetWorldPosition()
        {
            return transform.position;
        }

        public Vector3 GetProjWorldPosition()
        {
            return new Vector3(transform.position.x, 0, transform.position.z);
        }

        /* https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection.html */
        public bool Intersect(float2 rayOrigin, float2 rayDirection, out float2 hitPoint1, out float hitDistance1, out float2 hitPoint2, out float hitDistance2)
        {
            hitPoint1 = float2.zero;
            hitPoint2 = float2.zero;
            hitDistance1 = 0;
            hitDistance2 = 0;
            float2 center = new(transform.position.x, transform.position.z);
            float2 L = center - rayOrigin;
            float tca = math.dot(L, rayDirection);
            if (tca < 0) return false;
            float d2 = math.dot(L, L) - tca * tca;
            if (d2 > Radius * Radius) return false;
            float thc = math.sqrt(Radius * Radius - d2);
            float t0 = tca - thc;
            float t1 = tca + thc;
            if (t0 > t1)
            {
                (t1, t0) = (t0, t1);
            }
            if (t0 < 0)
            {
                t0 = t1; // If t0 is negative, let's use t1 instead.
                if (t0 < 0) return false; // Both t0 and t1 are negative.
            }
            hitPoint1 = rayOrigin + rayDirection * t0;
            hitDistance1 = t0;
            hitPoint2 = rayOrigin + rayDirection * t1;
            hitDistance2 = t1;
            return true;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            GizmosExtensions.DrawWireCircle(transform.position, Radius, Quaternion.identity);
        }
    }
}