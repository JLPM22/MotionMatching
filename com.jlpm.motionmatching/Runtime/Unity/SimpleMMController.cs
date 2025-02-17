using UnityEngine;
using UnityEngine.TextCore.Text;

namespace MotionMatching
{
    /// <summary>
    /// This class provides a simple interface to control a character with Motion Matching.
    /// Use the SetVelocity function to set the desired velocity of the character.
    /// The character will move in the direction of the velocity. 
    /// The magnitude is used to set the speed of the character. 
    /// </summary>
    [DefaultExecutionOrder(-100)]
    public class SimpleMMController : MonoBehaviour
    {
        public MotionMatchingData MotionMatchingData;

        private SpringCharacterController CharacterController;
        private MotionMatchingController MMController;
        private MotionMatchingSkinnedMeshRenderer SkinnedMeshRenderer;

        private void Awake()
        {
            CharacterController = GetComponentInChildren<SpringCharacterController>();
            Debug.Assert(CharacterController != null, "SimpleMMController requires a SpringCharacterController in a children GameObject");
            MMController = GetComponentInChildren<MotionMatchingController>();
            Debug.Assert(MMController != null, "SimpleMMController requires a MotionMatchingController in a children GameObject");
            SkinnedMeshRenderer = GetComponentInChildren<MotionMatchingSkinnedMeshRenderer>();
            Debug.Assert(SkinnedMeshRenderer != null, "SimpleMMController requires an avatar with a MotionMatchingSkinnedMeshRenderer in a children GameObject");

            MMController.MMData = MotionMatchingData;
            MMController.CharacterController = CharacterController;
            CharacterController.MotionMatching = MMController;
            SkinnedMeshRenderer.MotionMatching = MMController;
        }

        /// <summary>
        /// Use this function to set the desired velocity of the character.
        /// </summary>
        /// <param name="velocity">The desired direction and speed of the character.</param>
        public void SetVelocity(Vector2 velocity)
        {
            CharacterController.SetMovementDirection(velocity);
        }
    }
}
