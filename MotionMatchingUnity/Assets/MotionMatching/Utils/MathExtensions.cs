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
        public static quaternion Abs(quaternion q)
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

        // Source: https://theorangeduck.com/page/exponential-map-angle-axis-angular-velocity
        public static float3 AngularVelocity(quaternion current, quaternion next, float dt)
        {
            // Rln = Rotation from local to world next
            // Rlc = Rotation from local to world current
            // Rcn = Rotation from world current to world next (angular velocity if divided by dt)
            // Rln = Rcn * Rlc * vl <- where vl is a vector in local space
            // Rcn = Rln * Rlc^-1
            // IF quaternions are not normalized try: return QuaternionToScaledAngleAxis(math.normalizesafe(Abs(math.mul(next, math.inverse(current))))) / dt;
            return QuaternionToScaledAngleAxis(Abs(math.mul(next, math.inverse(current)))) / dt;
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

        /* Source: https://stackoverflow.com/a/33999726 */
        /// <summary>
        /// Mirror a quaternion along the X axis.
        /// </summary>
        public static quaternion MirrorX(quaternion q)
        {
            return new quaternion(q.value.x, -q.value.y, -q.value.z, q.value.w);
        }
        /// <summary>
        /// Mirror a quaternion along the Y axis.
        /// </summary>
        public static quaternion MirrorY(quaternion q)
        {
            return new quaternion(-q.value.x, q.value.y, -q.value.z, q.value.w);
        }
        /// <summary>
        /// Mirror a quaternion along the Z axis.
        /// </summary>
        public static quaternion MirrorZ(quaternion q)
        {
            return new quaternion(-q.value.x, -q.value.y, q.value.z, q.value.w);
        }

        /* Source: 'On the Continuity of Rotation Representations in Neural Networks' by Yi Zhou et al., 2019 */
        /// <summary>
        /// Transform a quaternion 4D to a continuous 6D representation
        /// </summary>
        public static float3x2 QuaternionToContinuous(quaternion q)
        {
            float3x3 rotation = new float3x3(q);
            return new float3x2(rotation.c0, rotation.c1);
        }
        /// <summary>
        /// Transform a continuous 6D to a quaternion 4D representation
        /// </summary>
        public static quaternion QuaternionFromContinuous(float3x2 m)
        {
            // Gram-Schmidt-like orthogonalization
            float3 b1 = math.normalize(m.c0);
            float3 b2 = math.normalize(m.c1 - (math.dot(b1, m.c1) * b1));
            float3 b3 = math.cross(b1, b2);
            float3x3 rotation = new float3x3(b1, b2, b3);
            return new quaternion(rotation);
        }
    }
}