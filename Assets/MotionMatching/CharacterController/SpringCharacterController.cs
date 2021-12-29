using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

namespace MotionMatching
{
    public class SpringCharacterController : MonoBehaviour
    {
        public void Movement(InputAction.CallbackContext value)
        {
            Vector2 input = value.ReadValue<Vector2>();
            Debug.Log(input + "Magnitude: " + input.magnitude);
        }

        /* https://theorangeduck.com/page/code-vs-data-driven-displacement */
        // Springs
        /* https://theorangeduck.com/page/spring-roll-call#controllers */
    }
}