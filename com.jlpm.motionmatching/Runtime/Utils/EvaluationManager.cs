using MotionMatching;
using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.LightTransport;

[DefaultExecutionOrder(-1000)]
public class EvaluationManager : MonoBehaviour
{
    public static EvaluationManager Instance { get; private set; }

    public string SceneName;
    public string TestName;

    private readonly Dictionary<int, List<float>> AgentPerformance = new();
    private readonly Dictionary<int, HashSet<int>> AgentDiversity = new();
    private readonly Dictionary<int, List<float>> AgentTrajectoryError = new();
    private readonly Dictionary<int, (MotionMatchingController, List<float>)> AgentCollisionTime = new();
    private readonly Dictionary<int, float2> PreviousAgentPosition = new();
    private int NextAgentID = 0; 
    private readonly CultureInfo CultureInfo = new("en-US");

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
        }
        else
        {
            Debug.LogError("Multiple instances of EvaluationManager detected.");
        }
    }

    private void OnDisable()
    {
        // Write data to txt file
        string filePath = $"C:/Users/user/Desktop/Projects/CrowdMatching/Evaluation/{SceneName}_{TestName}.txt";
        using (System.IO.StreamWriter file = new(filePath))
        {
            foreach (var kvp in AgentPerformance)
            {
                int agentID = kvp.Key;
                List<float> performances = kvp.Value;
                string performancesString = string.Join("; ", performances.Select(p => p.ToString(CultureInfo)));
                string diversityString = string.Join("; ", AgentDiversity[agentID]);
                string trajectoryErrorString = string.Join("; ", AgentTrajectoryError[agentID].Select(te => te.ToString(CultureInfo)));
                string agentCollisionTimeString = string.Join("; ", AgentCollisionTime[agentID].Item2.Select(ct => ct.ToString(CultureInfo)));
                string line = $"{agentID}: {performancesString} # {diversityString} # {trajectoryErrorString} # {agentCollisionTimeString}";
                file.WriteLine(line);
            }
        }
    }

    public int GetAgentID()
    {
        int agentID = NextAgentID;
        NextAgentID++;
        return agentID;
    }

    private Dictionary<int, float3> CurrentEllipse = new();
    private Dictionary<int, float2> WorldPos = new();
    private Dictionary<int, float2> PrevDisplVector = new();
    private Dictionary<int, HashSet<Obstacle>> CollisionObstacles = new();
    private Dictionary<Obstacle, float> CollisionStartingTime = new();
    private void Update()
    {
        for (int a = 0; a < AgentCollisionTime.Count; a++)
        {
            var kvp = AgentCollisionTime.ElementAt(a);
            var agentID = kvp.Key;
            var (controller, _) = kvp.Value;
            // Compute agent ellipse
            WorldPos[agentID] = new(controller.transform.position.x, controller.transform.position.z);
            float2 prevPos = PreviousAgentPosition[agentID];
            float2 displVector = WorldPos[agentID] - prevPos;
            if (math.length(displVector) < 1e-2)
            {
                //float2 prevDisplVector = PrevDisplVector[agentID];
                displVector = new float2(controller.transform.forward.x, controller.transform.forward.z);
            }
            float2 primaryAxis = math.normalize(displVector);
            float2 secondaryAxis = new(-primaryAxis.y, primaryAxis.x);
            float maxPrimaryDistance = 0.0f;
            float maxSecondaryDistance = 0.0f;
            foreach (Transform joint in controller.GetComponentsInChildren<Transform>())
            {
                float2 jointWorldPos = new(joint.position.x, joint.position.z);
                float2 jointVector = jointWorldPos - WorldPos[agentID];
                float primaryProjection = math.dot(jointVector, primaryAxis);
                float secondaryProjection = math.dot(jointVector, secondaryAxis);
                maxPrimaryDistance = math.max(maxPrimaryDistance, math.abs(primaryProjection));
                maxSecondaryDistance = math.max(maxSecondaryDistance, math.abs(secondaryProjection));
            }
            CurrentEllipse[agentID] = new(maxPrimaryDistance * primaryAxis, maxSecondaryDistance);

            PreviousAgentPosition[agentID] = WorldPos[agentID];
            PrevDisplVector[agentID] = displVector;

            // Compute collision time

            // previous obstacles
            foreach (Obstacle obstacle in CollisionObstacles[agentID])
            {
                float distance = UtilitiesBurst.DistancePointToEllipse(
                    WorldPos[agentID],
                    primaryAxis,
                    secondaryAxis,
                    new float2(maxPrimaryDistance, maxSecondaryDistance),
                    new float2(obstacle.transform.position.x, obstacle.transform.position.z),
                    out _
                    );
                distance -= obstacle.Radius;
                if (distance > UtilitiesBurst.MIN_INSIDE_ELLIPSE)
                {
                    CollisionObstacles[agentID].Remove(obstacle);
                    float collisionTime = Time.time - CollisionStartingTime[obstacle];
                    List<float> collisionTimes = AgentCollisionTime[agentID].Item2;
                    collisionTimes.Add(collisionTime);
                    AgentCollisionTime[agentID] = (controller, collisionTimes);
                    //Debug.Log("Collision ended between agent " + agentID + " and obstacle " + obstacle.name + "for " + collisionTime + " seconds.");
                    break;
                }
            }

            // new obstacles
            List<Obstacle> obstacles = ObstacleManager.Instance.GetObstacles();
            for (int i = 0; i < obstacles.Count; i++)
            {
                Obstacle obstacle = obstacles[i];

                if (!obstacle.IsStatic)
                {
                    throw new System.Exception("For now the EvaluationManager only works with static obstacles.");
                }

                if (CollisionObstacles[agentID].Contains(obstacle))
                {
                    continue;
                }

                // fast check
                if (math.distance(new float2(obstacle.transform.position.x, obstacle.transform.position.z), WorldPos[agentID]) >
                    math.max(maxPrimaryDistance, maxSecondaryDistance) + obstacle.Radius)
                {
                    continue;
                }

                // accurate check
                float distance = UtilitiesBurst.DistancePointToEllipse(
                    WorldPos[agentID],
                    primaryAxis,
                    secondaryAxis,
                    new float2(maxPrimaryDistance, maxSecondaryDistance),
                    new float2(obstacle.transform.position.x, obstacle.transform.position.z),
                    out _
                    );
                distance -= obstacle.Radius;
                if (distance <= UtilitiesBurst.MIN_INSIDE_ELLIPSE)
                {
                    //Debug.Log("Collision detected between agent " + agentID + " and obstacle " + obstacle.name);
                    CollisionObstacles[agentID].Add(obstacle);
                    CollisionStartingTime[obstacle] = Time.time;
                }
            }
        }
    }

    public void RegisterAgentPerformance(int agentID, float performance, List<int> poses, float trajectoryError, MotionMatchingController mmController)
    {
        if (!AgentPerformance.ContainsKey(agentID))
        {
            AgentPerformance[agentID] = new List<float>();
            AgentDiversity[agentID] = new HashSet<int>();
            AgentTrajectoryError[agentID] = new List<float>();
            AgentCollisionTime[agentID] = (mmController, new List<float>());
            PreviousAgentPosition[agentID] = new(mmController.transform.position.x, mmController.transform.position.z);
            PrevDisplVector[agentID] = new(mmController.transform.forward.x, mmController.transform.forward.z);
            CollisionObstacles[agentID] = new HashSet<Obstacle>();
        }
        AgentPerformance[agentID].Add(performance);
        AgentDiversity[agentID].UnionWith(poses);
        AgentTrajectoryError[agentID].Add(trajectoryError);
    }

    private void OnDrawGizmos()
    {
        if (WorldPos.Count == 0 || CurrentEllipse.Count == 0)
        {
            return;
        }

        // HARDCODED: draw only first agent

        float3 posOffset = new(WorldPos[0].x, 0.02f, WorldPos[0].y);
        float ellipsePrimaryMagnitude = math.length(new float2(CurrentEllipse[0].x, CurrentEllipse[0].y));
        float2 primaryAxis = math.normalize(new float2(CurrentEllipse[0].x, CurrentEllipse[0].y));
        float2 secondaryAxis = new(-primaryAxis.y, primaryAxis.x);

        Gizmos.color = Color.yellow;
        GizmosExtensions.DrawWireEllipse(posOffset, primaryAxis * ellipsePrimaryMagnitude, secondaryAxis * CurrentEllipse[0].z, Quaternion.identity, segments: 20 * Mathf.FloorToInt(MotionMatchingController.GIZMOS_MULTIPLIER), thickness: 1.5f * MotionMatchingController.GIZMOS_MULTIPLIER);

    }
}
