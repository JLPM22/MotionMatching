using System;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    [DefaultExecutionOrder(-1000)]
    public class ObstacleManager : MonoBehaviour
    {
        public static ObstacleManager Instance { get; private set; }

        public event Action<List<Obstacle>> OnObstaclesUpdated;

        private List<Obstacle> Obstacles = new();
        private bool ObstaclesUpdated = false;

        private void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
            }
            else
            {
                Debug.LogError("Multiple instances of CrowdObstacleManager detected.");
            }
        }

        public List<Obstacle> GetObstacles()
        {
            return Obstacles;
        }

        public void RegisterObstacle(Obstacle obstacle)
        {
            if (!Obstacles.Contains(obstacle))
            {
                Obstacles.Add(obstacle);
                ObstaclesUpdated = true;
            }
        }

        public void UnregisterObstacle(Obstacle obstacle)
        {
            if (Obstacles.Contains(obstacle))
            {
                Obstacles.Remove(obstacle);
                ObstaclesUpdated = true;
            }
        }

        private void LateUpdate()
        {
            if (ObstaclesUpdated)
            {
                OnObstaclesUpdated?.Invoke(Obstacles);
                ObstaclesUpdated = false;
            }
        }
    }
}