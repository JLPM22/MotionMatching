using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
public static class UtilitiesBurst
{
    public static readonly float MIN_INSIDE_ELLIPSE = 1e-6f;
    public static readonly float MAX_INSIDE_ELLIPSE = 1e-9f;

    // Source: https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    [BurstCompile]
    private static float GetRoot(in float r0, in float z0, in float z1, in float g, in int maxIterations = 149)
    {
        // maxIterations for float (32-bit) ideally should be 149
        float gi = g;
        float n0 = r0 * z0;
        float s0 = z1 - 1.0f;
        float s1 = gi < 0.0f ? 0.0f : math.length(new float2(n0, z1)) - 1.0f;
        float s = 0.0f;
        for (int i = 0; i < maxIterations; i++)
        {
            s = (s0 + s1) / 2.0f;
            if (math.abs(s - s0) < 1e-9 || math.abs(s - s1) < 1e-9)
            {
                break;
            }
            float ratio0 = n0 / (s + r0);
            float ratio1 = z1 / (s + 1.0f);
            gi = ratio0 * ratio0 + ratio1 * ratio1 - 1.0f;
            if (gi > 0.0f)
            {
                s0 = s;
            }
            else if (gi < 0.0f)
            {
                s1 = s;
            }
            else
            {
                break;
            }
        }
        return s;
    }

    // 'ellipse' are the extents of the ellipse axis, 'ellipse.x' >= 'ellipse.y' > 0
    // 'p' is the query point, 'p' >= 0
    // 'closest' is the ellipse point closest to the point 'p'
    // Source: https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    [BurstCompile]
    private static float PositiveQuarterDistanceToEllipse(in float2 ellipse, in float2 p, out float2 closest, in int maxIterations = 149)
    {
        Debug.Assert(ellipse.x >= ellipse.y);

        float distance = 0.0f;
        if (p.y > 0.0f)
        {
            if (p.x > 0.0f)
            {
                float2 z = p / ellipse;
                float g = z.x * z.x + z.y * z.y - 1.0f;
                if (math.abs(g) > 1e-9) // != 0
                {
                    float r0 = (ellipse.x / ellipse.y) * (ellipse.x / ellipse.y);
                    float sbar = GetRoot(r0, z.x, z.y, g, maxIterations);
                    closest.x = r0 * p.x / (sbar + r0);
                    closest.y = p.y / (sbar + 1.0f);
                    distance = math.sqrt((closest.x - p.x) * (closest.x - p.x) + (closest.y - p.y) * (closest.y - p.y));
                }
                else
                {
                    closest.x = p.x;
                    closest.y = p.y;
                    distance = 0.0f;
                }
            }
            else // p.x == 0.0f
            {
                closest.x = 0.0f;
                closest.y = ellipse.y;
                distance = math.abs(p.y - ellipse.y);
            }
        }
        else // p.y == 0.0f
        {
            float numer0 = ellipse.x * p.x;
            float denom0 = ellipse.x * ellipse.x - ellipse.y * ellipse.y;
            if (numer0 < denom0)
            {
                float xde0 = numer0 / denom0;
                closest.x = ellipse.x * xde0;
                closest.y = ellipse.y * math.sqrt(1.0f - xde0 * xde0);
                distance = math.sqrt((closest.x - p.x) * (closest.x - p.x) + closest.y * closest.y);
            }
            else
            {
                closest.x = ellipse.x;
                closest.y = 0.0f;
                distance = math.abs(p.x - ellipse.x);
            }
        }
        return distance;
    }

    // 'ellipse' are the extents of the ellipse axis
    // 'p' is the query point in world space
    // 'closest' is the ellipse point (in world space) closest to the point 'p'
    // Returns distance to the circumference of the ellipse, 0 when inside
    [BurstCompile]
    public static float DistancePointToEllipse(in float2 centerEllipse, in float2 primaryAxisUnit, in float2 secondaryAxisUnit,
                                               in float2 ellipse, in float2 query, out float2 closest,
                                               in int maxIterations = 149, bool ignoreCircleDebug=false)
    {
        Debug.Assert(ellipse.x > 0.0 && ellipse.y > 0.0);

        // UNCOMMENT THIS TO MAKE THIS CIRCLE CHECK ---
        //if (!ignoreCircleDebug)
        //{
        //    float circleRadius = math.max(ellipse.x, ellipse.y);
        //    float distanceCircle = math.distance(query, centerEllipse) - circleRadius;
        //    closest = centerEllipse + math.normalize(query - centerEllipse) * circleRadius;
        //    return distanceCircle;
        //}
        // END CIRCLE CHECK ---------------------------

        float2 p = query;
        float2 e = ellipse;
        float2 primary = primaryAxisUnit;
        float2 secondary = secondaryAxisUnit;
        if (e.y > e.x)
        {
            e.x = ellipse.y;
            e.y = ellipse.x;

            primary = secondaryAxisUnit;
            secondary = primaryAxisUnit;
        }

        // interpret the center of the sphere as the query point
        // change to ellipse coordinate frame
        p = p - centerEllipse;
        p = new(math.dot(math.normalize(primary), p),
                math.dot(math.normalize(secondary), p));
        float distance;
        if (p.x < 0.0f && p.y < 0.0f)
        {
            p = -p;
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest, maxIterations);
            closest = -closest;
        }
        else if (p.x < 0.0f)
        {
            p.x = -p.x;
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest, maxIterations);
            closest.x = -closest.x;
        }
        else if (p.y < 0.0f)
        {
            p.y = -p.y;
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest, maxIterations);
            closest.y = -closest.y;
        }
        else
        {
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest, maxIterations);
        }
        closest = centerEllipse + primary * closest.x + secondary * closest.y;

        if (p.x < e.x && p.y < e.y) // check if inside
        {
            distance = math.lerp(MIN_INSIDE_ELLIPSE, MAX_INSIDE_ELLIPSE, distance / e.x);
        }

        return distance;
    }

    /// <summary>
    /// Generates a point on the circumference of an ellipse at a specified angle in degrees.
    /// </summary>
    /// <param name="centerEllipse">The center of the ellipse in world space.</param>
    /// <param name="primaryAxisUnit">The normalized direction of the primary axis.</param>
    /// <param name="secondaryAxisUnit">The normalized direction of the secondary axis.</param>
    /// <param name="ellipseExtents">A float2 where x is the extent along the primary axis and y is the extent along the secondary axis.</param>
    /// <param name="angleInDegrees">The angle in degrees (0 to 360) at which to generate the point, measured from the primary axis.</param>
    /// <returns>The point on the ellipse circumference in world space at the specified angle.</returns>
    [BurstCompile]
    public static void GeneratePointOnEllipse(in float2 centerEllipse, in float2 primaryAxisUnit, in float2 secondaryAxisUnit,
                                              in float2 ellipseExtents, in float angleInDegrees, out float2 result)
    {
        // 1. Convert angle from degrees to radians
        float angleInRadians = angleInDegrees * Mathf.Deg2Rad;

        // 2. Calculate point on a hypothetical unrotated ellipse at the given angle
        // The parametric equations for an unrotated ellipse aligned with x and y axes are:
        // x = a * cos(theta)
        // y = b * sin(theta)
        // In our case, 'a' is ellipseExtents.x and 'b' is ellipseExtents.y
        float2 pointOnUnrotatedEllipse = new float2(
            ellipseExtents.x * math.cos(angleInRadians),
            ellipseExtents.y * math.sin(angleInRadians)
        );

        // 3. Rotate and translate the point based on the ellipse's orientation and center
        // We can use the primary and secondary axis units as basis vectors for rotation.
        // The point on the rotated ellipse is the center plus the contribution along
        // the primary and secondary axes based on the calculated unrotated point.
        float2 pointOnRotatedEllipse = centerEllipse +
                                       primaryAxisUnit * pointOnUnrotatedEllipse.x +
                                       secondaryAxisUnit * pointOnUnrotatedEllipse.y;

        result = pointOnRotatedEllipse;
    }

    // 'ellipse' are the extents of the ellipse axis
    // 'p' is the query point in world space
    // 'closest' is the ellipse point (in world space) closest to the point 'p'
    // Returns distance to the circumference of the ellipse, 0 when inside
    [BurstCompile]
    public static float FastDistancePointToEllipse(in float2 centerEllipse, in float2 primaryAxisUnit, in float2 secondaryAxisUnit,
                                                   in float2 ellipse, in float2 query, out float2 closest,
                                                   in float angle = 30.0f)
    {
        Debug.Assert(ellipse.x > 0.0 && ellipse.y > 0.0);

        float minDistance = float.MaxValue;
        float2 minV = float2.zero;
        closest = float2.zero;
        for (float a = 0.0f; a <= 360.0f; a += angle)
        {
            GeneratePointOnEllipse(centerEllipse, primaryAxisUnit, secondaryAxisUnit, ellipse, a, out float2 p);
            float2 v = p - query;
            float d = math.length(v);
            if (d < minDistance)
            {
                minDistance = d;
                closest = p;
                minV = v;
            }
        }
        if (math.dot(minV / minDistance, math.normalize(centerEllipse - query)) < 0.0f ||
            math.distance(centerEllipse, query) < minDistance) // check if inside
        {
            return math.lerp(MIN_INSIDE_ELLIPSE, MAX_INSIDE_ELLIPSE, minDistance / math.max(ellipse.x, ellipse.y));
        }
        return minDistance;
    }

    [BurstCompile]
    public static float DistanceEllipseToEllipse(in float2 centerEllipse1, in float2 primaryAxisUnit1, in float2 secondaryAxisUnit1, in float2 ellipse1,
                                             in float2 centerEllipse2, in float2 primaryAxisUnit2, in float2 secondaryAxisUnit2, in float2 ellipse2,
                                             out float2 closest1, out float2 closest2, in float angle = 30.0f, in int maxIterations = 149)
    {
        float minDistance = float.MaxValue;
        closest1 = float2.zero;
        closest2 = float2.zero;
        for (float a = 0.0f; a <= 360.0f; a += angle)
        {
            GeneratePointOnEllipse(centerEllipse2, primaryAxisUnit2, secondaryAxisUnit2, ellipse2, a, out float2 p);
            float d = DistancePointToEllipse(centerEllipse1, primaryAxisUnit1, secondaryAxisUnit1, ellipse1, p, out float2 candidateClosest, maxIterations: maxIterations);
            if (d < minDistance)
            {
                minDistance = d;
                closest1 = candidateClosest;
                closest2 = p;
            }
        }
        return minDistance;
    }

    [BurstCompile]
    public static float FastDistanceEllipseToEllipse(in float2 centerEllipse1, in float2 primaryAxisUnit1, in float2 secondaryAxisUnit1, in float2 ellipse1,
                                                     in float2 centerEllipse2, in float2 primaryAxisUnit2, in float2 secondaryAxisUnit2, in float2 ellipse2,
                                                     out float2 closest1, out float2 closest2, in float angle = 30.0f)
    {
        float minDistance = float.MaxValue;
        closest1 = float2.zero;
        closest2 = float2.zero;
        for (float a = 0.0f; a <= 360.0f; a += angle)
        {
            GeneratePointOnEllipse(centerEllipse2, primaryAxisUnit2, secondaryAxisUnit2, ellipse2, a, out float2 p);
            float d = FastDistancePointToEllipse(centerEllipse1, primaryAxisUnit1, secondaryAxisUnit1, ellipse1, p, out float2 candidateClosest, angle: angle);
            if (d < minDistance)
            {
                minDistance = d;
                closest1 = candidateClosest;
                closest2 = p;
            }
        }
        return minDistance;
    }

}
