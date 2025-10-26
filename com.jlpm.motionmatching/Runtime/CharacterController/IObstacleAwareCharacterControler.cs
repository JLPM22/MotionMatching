using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

public interface IObstacleAwareCharacterControler
{
    public (
        NativeArray<(float2, float, float2)>,
        NativeArray<int>,
        NativeArray<(float2, float2, float2)>,
        NativeArray<int>
    ) GetNearbyObstacles(Transform character, float obstacleDistanceThreshold);
}
