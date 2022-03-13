using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    public abstract class MotionMatchingCharacterController : MonoBehaviour
    {
        public event Action<float> OnUpdated;
        public event Action OnInputChangedQuickly;

        public MotionMatchingController SimulationBone; // MotionMatchingController's transform is the SimulationBone of the character
        public int NumberPrediction = 3;

        // Accumulated Delta Time
        public float AveragedDeltaTime { get; private set; }
        private Queue<float> LastDeltaTime = new Queue<float>();
        private float SumDeltaTime;

        private void Update()
        {
            // Average DeltaTime (for prediction... it is better to have a stable frame rate)
            AveragedDeltaTime = GetAveragedDeltaTime();
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
        public abstract float2 GetWorldPredictedPosition(int index);
        public abstract float2 GetWorldPredictedDirection(int index);

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