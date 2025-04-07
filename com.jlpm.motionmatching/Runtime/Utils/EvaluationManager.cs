using System.Collections.Generic;
using System.Globalization;
using System.Linq;
using UnityEngine;

[DefaultExecutionOrder(-1000)]
public class EvaluationManager : MonoBehaviour
{
    public static EvaluationManager Instance { get; private set; }

    public string SceneName;
    public string TestName;

    private readonly Dictionary<int, List<float>> AgentPerformance = new();
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
                string line = $"{agentID}: {performancesString}";
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

    public void RegisterAgentPerformance(int agentID, float performance)
    {
        if (!AgentPerformance.ContainsKey(agentID))
        {
            AgentPerformance[agentID] = new List<float>();
        }
        AgentPerformance[agentID].Add(performance);
    }
}
