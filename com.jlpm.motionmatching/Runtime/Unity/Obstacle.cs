using MotionMatching;
using UnityEngine;

namespace MotionMatching
{
    public class Obstacle : MonoBehaviour
    {
        public float Radius = 1.0f;

        public Vector3 GetWorldPosition()
        {
            return transform.position;
        }

        public Vector3 GetProjWorldPosition()
        {
            return new Vector3(transform.position.x, 0, transform.position.z);
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            GizmosExtensions.DrawWireCircle(transform.position, Radius, Quaternion.identity);
        }
    }
}