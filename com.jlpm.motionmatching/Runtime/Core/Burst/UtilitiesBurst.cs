using Unity.Burst;
using Unity.Mathematics;
using UnityEngine;

[BurstCompile]
public static class UtilitiesBurst
{
    // Source: https://www.geometrictools.com/Documentation/DistancePointEllipseEllipsoid.pdf
    [BurstCompile]
    private static float GetRoot(in float r0, in float z0, in float z1, in float g)
    {
        const int maxIterations = 149; // for float (32-bit)
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
    private static float PositiveQuarterDistanceToEllipse(in float2 ellipse, in float2 p, out float2 closest)
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
                    float sbar = GetRoot(r0, z.x, z.y, g);
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
    public static float DistanceToEllipse(in float2 centerEllipse, in float2 primaryAxisUnit, in float2 secondaryAxisUnit, in float2 ellipse, in float2 query, out float2 closest)
    {
        Debug.Assert(ellipse.x > 0.0 && ellipse.y > 0.0);

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
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest);
            closest = -closest;
        }
        else if (p.x < 0.0f)
        {
            p.x = -p.x;
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest);
            closest.x = -closest.x;
        }
        else if (p.y < 0.0f)
        {
            p.y = -p.y;
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest);
            closest.y = -closest.y;
        }
        else
        {
            distance = PositiveQuarterDistanceToEllipse(e, p, out closest);
        }
        closest = centerEllipse + primary * closest.x + secondary * closest.y;

        if (p.x < e.x && p.y < e.y) // check if inside
        {
            distance = 1e-9f;
        }

        return distance;
    }

}
