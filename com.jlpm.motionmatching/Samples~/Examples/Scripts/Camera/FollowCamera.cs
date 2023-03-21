using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform Target;
    public Vector3 Offset;
    public float Smooth = 1;

    private Vector3 SmoothVelocity;

    private void Update()
    {
        if (Target != null)
        {
            transform.position = Vector3.SmoothDamp(transform.position, Target.position + Offset, ref SmoothVelocity, Smooth * Time.deltaTime);
        }
    }
}
