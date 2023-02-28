using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

    public abstract class MotionMatchingCharacterController : MonoBehaviour
    {
		// TODO: Create a OnValidate() (other name because it will collide with Unity's
		//       that validates if the current MMData has the necessary trajectories requeried
		//       by the current controller (eg. simulation bone pos + dir, or HMD + L/R controllers pos + dir)
		
        public event Action<float> OnUpdated;
        public event Action OnInputChangedQuickly;

        public MotionMatchingController SimulationBone; // MotionMatchingController's transform is the SimulationBone of the character

        public bool LockFPS { get { return SimulationBone.LockFPS; } }
        // Accumulated Delta Time
        public float AveragedDeltaTime { get; private set; }
        private Queue<float> LastDeltaTime = new Queue<float>();
        private float SumDeltaTime;
        // ---

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

        /// <summary>
        /// Call this method to notify Motion Matching that a large change in the input has been made.
        /// Therefore, an immediate Motion Matching search should be performed.
        /// </summary>
        protected void NotifyInputChangedQuickly()
        {
            if (OnInputChangedQuickly != null) OnInputChangedQuickly.Invoke();
        }

        /// <summary>
        /// Use this intead of Unity's Update() method.
        /// </summary>
        protected abstract void OnUpdate();

        /// <summary>
        /// Return the initial world position of the character.
        /// </summary>
        public abstract float3 GetWorldInitPosition();
        /// <summary>
        /// Return the initial world direction of the character.
        /// </summary>
        public abstract float3 GetWorldInitDirection();

        /// <summary>
        /// Get the prediction in world space of a trajectory feature.
        /// e.g. suppose that the feature is the projected position of the character at frames 20, 40 and 60 in the future.
        /// Then, since the projected position is 2D (2 floats), thus, output[0] and output[1] should be filled with the X and Z coordinates.
        /// If index==1, it should return the position of the character at frame 40.
        /// </summary>
        public abstract void GetTrajectoryFeature(TrajectoryFeature feature, int index, Transform character, NativeArray<float> output);

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