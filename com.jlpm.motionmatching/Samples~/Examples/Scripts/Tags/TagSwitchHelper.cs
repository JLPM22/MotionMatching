using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MotionMatching;

[RequireComponent(typeof(MotionMatchingCharacterController))]
public class TagSwitchHelper : MonoBehaviour
{
    [SerializeField] [Delayed] private string QueryExpression;

    private MotionMatchingCharacterController CharacterController;
    private QueryTag CurrentQueryTag;
    
    public bool IsQuerySet { get; private set; }

    private void Awake()
    {
        CharacterController = GetComponent<MotionMatchingCharacterController>();
    }

    private void Start()
    {
        if (QueryExpression != null)
        {
            UpdateQuery();
        }
    }

    public void UpdateQuery()
    {
        Dispose();
        if (QueryTag.Parse(QueryExpression, out CurrentQueryTag))
        {
            CharacterController.MotionMatching.SetQueryTag(CurrentQueryTag);
            IsQuerySet = true;
        }
        else
        {
            DisableQuery();
            IsQuerySet = false;
        }
    }

    public void DisableQuery()
    {
        CharacterController.MotionMatching.DisableQueryTag();
        IsQuerySet = false;
    }

    public void SetExpression(string expression)
    {
        QueryExpression = expression;
        UpdateQuery();
    }

    private void Dispose()
    {
        if (CurrentQueryTag != null) CurrentQueryTag.Dispose();
    }

    private void OnDestroy()
    {
        Dispose();
    }


#if UNITY_EDITOR
    private void OnValidate()
    {
        if (Application.isPlaying && CharacterController != null)
        {
            UpdateQuery();
        }
    }
#endif
}
