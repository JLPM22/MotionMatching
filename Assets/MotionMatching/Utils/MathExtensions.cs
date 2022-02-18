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
        /// </summary>
        public static quaternion FromToRotationSafe(float3 from, float3 to)
        {
            return quaternion.AxisAngle(
                angle: math.acos(math.clamp(math.dot(math.normalize(from), math.normalize(to)), -1f, 1f)),
                axis: math.normalize(math.cross(from, to))
            );
        }

        /// <summary>
        /// Returns the rotation between two vectors, from and to are ASSUMED to be normalized
        /// </summary>
        public static quaternion FromToRotation(float3 from, float3 to)
        {
            return quaternion.AxisAngle(
                angle: math.acos(math.clamp(math.dot(from, to), -1f, 1f)),
                axis: math.normalize(math.cross(from, to))
            );
        }
    }
}