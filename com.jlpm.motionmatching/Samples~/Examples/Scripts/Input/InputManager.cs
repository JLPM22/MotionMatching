using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Class responsible for wrapping the input from the generated C# class from the new Unity's Input System.
/// </summary>
public class InputManager : MonoBehaviour
{
    public event Action<Vector2> OnMovementDirectionChanged; // Vector2 with the direction of movement
    public event Action OnFixOrientationSwap; // Called when the orientation is fixed/unfixed

    private PlayerInput Input;

    private void Awake()
    {
        Input = new PlayerInput();
    }

    private void OnEnable()
    {
        Input.Player.Enable();
        Input.Player.Movement.performed += OnMovementPerformed;
        Input.Player.FixOrientation.performed += OnFixOrientationSwapPerformed;
    }

    private void OnDisable()
    {
        Input.Player.Disable();
        Input.Player.Movement.performed -= OnMovementPerformed;
        Input.Player.FixOrientation.performed -= OnFixOrientationSwapPerformed;
    }

    private void OnMovementPerformed(InputAction.CallbackContext context)
    {
        Vector2 direction = context.ReadValue<Vector2>();
        if (OnMovementDirectionChanged != null) OnMovementDirectionChanged(direction);
    }

    private void OnFixOrientationSwapPerformed(InputAction.CallbackContext context)
    {
        if (OnFixOrientationSwap != null) OnFixOrientationSwap();
    }
}
