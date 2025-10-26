using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotionMatching;

[RequireComponent(typeof(InputManager))]
/// <summary>
/// This class listents to InputManager and input the desired information to SpringCharacterController.
/// </summary>
public class InputCharacterController : MonoBehaviour
{
    private InputManager Input;
    private IPlayerInputCharacterController CharacterController;
    private TagSwitchHelper TagHelper;

    private void Awake()
    {
        Input = GetComponent<InputManager>();
        CharacterController = GetComponent<IPlayerInputCharacterController>();
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
        if (CharacterController != null)
        {
            CharacterController.SetMovementDirection(movementDirection);
        }
    }

    public void OnFixOrientationSwap()
    {
        if (CharacterController != null)
        {
            CharacterController.SwapFixOrientation();
        }
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