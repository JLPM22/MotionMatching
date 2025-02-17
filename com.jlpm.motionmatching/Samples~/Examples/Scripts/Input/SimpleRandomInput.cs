using MotionMatching;
using UnityEngine;

public class SimpleRandomInput : MonoBehaviour
{
    public SimpleMMController MMController;

    private float Timer = 3.0f;

    private void Update()
    {
        Timer += Time.deltaTime;

        // Every 3 seconds, generate a new random velocity
        if (Timer > 3.0f)
        {
            float magnitude = Random.Range(0.3f, 1.2f);
            MMController.SetVelocity(new Vector2(Random.Range(-1.0f, 1.0f), Random.Range(-1.0f, 1.0f)) * magnitude);
            Timer = 0.0f;
        }
    }
}
