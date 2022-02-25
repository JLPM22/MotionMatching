using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

namespace MotionMatching
{
    public static class MathExtensions
    {
        /// <summary>
        /// Returns the rotation between two vectors, from and to are normalized
        /// If from and to are coplanar and opposite, coplanarNormal is used to determine the axis of rotation
        /// </summary>
        public static quaternion FromToRotationSafe(float3 from, float3 to, float3 coplanarNormal)
        {
            float dotFT = math.dot(math.normalize(from), math.normalize(to));
            if (dotFT > 0.99999f) // cross(from, to) is zero
            {
                return quaternion.identity;
            }
            else if (dotFT < -0.99999f) // cross(from, to) is zero
            {
                return quaternion.AxisAngle(
                    angle: math.PI,
                    axis: coplanarNormal
                );
            }
            return quaternion.AxisAngle(
                angle: math.acos(math.clamp(dotFT, -1f, 1f)),
                axis: math.normalize(math.cross(from, to))
            );
        }

        /// <summary>
        /// Returns the rotation between two vectors, from and to are ASSUMED to be normalized
        /// If from and to are coplanar and opposite, coplanarNormal is used to determine the axis of rotation
        /// </summary>
        public static quaternion FromToRotation(float3 from, float3 to, float3 coplanarNormal)
        {
            float dotFT = math.dot(from, to);
            if (dotFT > 0.99999f) // cross(from, to) is zero
            {
                return quaternion.identity;
            }
            else if (dotFT < -0.99999f) // cross(from, to) is zero
            {
                return quaternion.AxisAngle(
                    angle: math.PI,
                    axis: coplanarNormal
                );
            }
            return quaternion.AxisAngle(
                angle: math.acos(math.clamp(dotFT, -1f, 1f)),
                axis: math.normalize(math.cross(from, to))
            );
        }

        /// <summary>
        /// Quaternion absolute forces the quaternion to take the shortest path
        /// </summary>
        public static quaternion Absolute(quaternion q)
        {
            return q.value.w < 0.0f ? new quaternion(-q.value.x, -q.value.y, -q.value.z, -q.value.w) : q;
        }

        /* https://theorangeduck.com/page/exponential-map-angle-axis-angular-velocity */
        public static float3 QuaternionToScaledAngleAxis(quaternion q, float eps = 1e-8f)
        {
            return 2.0f * Log(q, eps);
        }

        public static quaternion QuaternionFromScaledAngleAxis(float3 angleAxis, float eps = 1e-8f)
        {
            return Exp(angleAxis * 0.5f, eps);
        }

        public static float3 Log(quaternion q, float eps = 1e-8f)
        {
            float length = math.sqrt(q.value.x * q.value.x + q.value.y * q.value.y + q.value.z * q.value.z);
            if (length < eps)
            {
                return new float3(q.value.x, q.value.y, q.value.z);
            }
            else
            {
                float halfangle = math.acos(math.clamp(q.value.w, -1f, 1f));
                return halfangle * (new float3(q.value.x, q.value.y, q.value.z) / length);
            }
        }

        public static quaternion Exp(float3 angleAxis, float eps = 1e-8f)
        {
            float halfangle = math.sqrt(angleAxis.x * angleAxis.x + angleAxis.y * angleAxis.y + angleAxis.z * angleAxis.z);
            if (halfangle < eps)
            {
                return math.normalize(new quaternion(angleAxis.x, angleAxis.y, angleAxis.z, 1f));
            }
            else
            {
                float c = math.cos(halfangle);
                float s = math.sin(halfangle) / halfangle;
                return new quaternion(s * angleAxis.x, s * angleAxis.y, s * angleAxis.z, c);
            }
        }
    }
}