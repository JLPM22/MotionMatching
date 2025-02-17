using MotionMatching;
using UnityEngine;
using UnityEngine.InputSystem;

public class SimpleKeyboardInput : MonoBehaviour
{
    public SimpleMMController MMController;

    private void Update()
    {
        float horizontal = Keyboard.current.aKey.isPressed ? -1 : Keyboard.current.dKey.isPressed ? 1 : 0;
        float vertical = Keyboard.current.sKey.isPressed ? -1 : Keyboard.current.wKey.isPressed ? 1 : 0;

        MMController.SetVelocity(new Vector2(horizontal, vertical));
    }
}
