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

        public MotionMatchingController MotionMatching; // MotionMatchingController's transform is the SimulationBone of the character

        public float DatabaseDeltaTime { get; private set; }

        private void LateUpdate()
        {
            DatabaseDeltaTime = MotionMatching.DatabaseFrameTime;
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
        /// Get the prediction in character space of a trajectory feature.
        /// e.g., suppose that the feature is the projected position of the character at frames 20, 40 and 60 in the future:
        ///       then, since the projected position is 2D (2 floats), thus, output[0] and output[1] should be filled with the X and Z coordinates.
        ///       e.g., when index==1, it should return the position of the character at frame 40.
        /// </summary>
        public abstract void GetTrajectoryFeature(TrajectoryFeature feature, int index, Transform character, NativeArray<float> output);
    }
}