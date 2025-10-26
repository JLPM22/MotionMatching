using Unity.Mathematics;

public interface IPlayerInputCharacterController
{
    public void SetMovementDirection(float2 movementDirection);
    public void SwapFixOrientation();
}
