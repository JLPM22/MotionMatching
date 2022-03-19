using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

    public abstract class MotionMatchingCharacterController : MonoBehaviour
    {
        public event Action<float> OnUpdated;
        public event Action OnInputChangedQuickly;

        public MotionMatchingController SimulationBone; // MotionMatchingController's transform is the SimulationBone of the character

        // Accumulated Delta Time
        public bool LockFPS { get { return SimulationBone.LockFPS; } }
        public float AveragedDeltaTime { get; private set; }
        private Queue<float> LastDeltaTime = new Queue<float>();
        private float SumDeltaTime;

        private void Update()
        {
            if (LockFPS)
            {
                AveragedDeltaTime = SimulationBone.FrameTime;
            }
            else
            {
                // Average DeltaTime (for prediction... it is better to have a stable frame rate)
                AveragedDeltaTime = GetAveragedDeltaTime();
            }
            // Update the character
            OnUpdate();
            // Update other components depending on the character controller
            if (OnUpdated != null) OnUpdated.Invoke(Time.deltaTime);
        }

        protected void NotifyInputChangedQuickly()
        {
            if (OnInputChangedQuickly != null) OnInputChangedQuickly.Invoke();
        }

        protected abstract void OnUpdate();

        public abstract float3 GetWorldInitPosition();
        public abstract float3 GetWorldInitDirection();
        public abstract float3 GetCurrentPosition();
        public abstract quaternion GetCurrentRotation();
        /// <summary>
        /// Get the prediction in world space of the feature.
        /// e.g. the feature is the position of the character, and it has frames = { 20, 40, 60}
        /// if index=1 it will return the position of the character at frame 40
        /// </summary>
        public abstract float3 GetWorldSpacePrediction(TrajectoryFeature feature, int index);

        private float GetAveragedDeltaTime()
        {
            const int nAverageDeltaTime = 20;
            SumDeltaTime += Time.deltaTime;
            LastDeltaTime.Enqueue(Time.deltaTime);
            if (LastDeltaTime.Count == nAverageDeltaTime + 1) SumDeltaTime -= LastDeltaTime.Dequeue();
            return SumDeltaTime / nAverageDeltaTime;
        }
    }
}