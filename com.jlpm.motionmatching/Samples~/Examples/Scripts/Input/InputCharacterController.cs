using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotionMatching;

[RequireComponent(typeof(InputManager))]
[RequireComponent(typeof(SpringCharacterController))]
/// <summary>
/// This class listents to InputManager and input the desired information to SpringCharacterController.
/// </summary>
public class InputCharacterController : MonoBehaviour
{
    private InputManager Input;
    private SpringCharacterController CharacterController;
    private TagSwitchHelper TagHelper;

    private void Awake()
    {
        Input = GetComponent<InputManager>();
        CharacterController = GetComponent<SpringCharacterController>();
        TagHelper = GetComponent<TagSwitchHelper>();
    }

    private void OnEnable()
    {
        Input.OnMovementDirectionChanged += OnMovementDirectionChanged;
        Input.OnFixOrientationSwap += OnFixOrientationSwap;
        Input.OnSwitchTag += OnSwitchTag;
    }

    private void OnDisable()
    {
        Input.OnMovementDirectionChanged -= OnMovementDirectionChanged;
        Input.OnFixOrientationSwap -= OnFixOrientationSwap;
        Input.OnSwitchTag -= OnSwitchTag;
    }

    public void OnMovementDirectionChanged(Vector2 movementDirection)
    {
        CharacterController.SetMovementDirection(movementDirection);
    }

    public void OnFixOrientationSwap()
    {
        CharacterController.SwapFixOrientation();
    }

    public void OnSwitchTag()
    {
        if (TagHelper != null)
        {
            if (TagHelper.IsQuerySet)
            {
                TagHelper.DisableQuery();
            }
            else
            {
                TagHelper.UpdateQuery();
            }
        }
    }
}