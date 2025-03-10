using System;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;


    public class CrowdCharacterController : MotionMatchingCharacterController
    {
        public Transform[] Obstacles;
        // Features ----------------------------------------------------------
        [Header("Features")]
        public string TrajectoryPositionFeatureName = "FuturePosition";
        public string TrajectoryDirectionFeatureName = "FutureDirection";
        // General ----------------------------------------------------------
        [Header("General")]
        public float MaxSpeed = 1.0f;
        [Range(0.0f, 1.0f)] public float ResponsivenessPositions = 0.75f;
        [Range(0.0f, 1.0f)] public float ResponsivenessDirections = 0.75f;
        public float MinimumVelocityClamp = 0.01f;
        [Tooltip("Controls when to consider that the input has suddenly changed. Used to recompute MotionMatching. -1.0f: Never. 1.0f: Always")]
        [Range(-1.0f, 1.0f)] public float InputBigChangeThreshold = 0.5f;
        // Adjustment & Clamping --------------------------------------------
        [Header("Adjustment")] // Move Simulation Bone towards the Simulation Object (motion matching towards character controller)
        public bool DoAdjustment = true;
        [Range(0.0f, 2.0f)] public float PositionAdjustmentHalflife = 0.1f; // Time needed to move half of the distance between MotionMatching and the CharacterController
        [Range(0.0f, 2.0f)] public float RotationAdjustmentHalflife = 0.1f;
        [Range(0.0f, 2.0f)] public float PosMaximumAdjustmentRatio = 0.1f; // Ratio between the adjustment and the character's velocity to clamp the adjustment
        [Range(0.0f, 2.0f)] public float RotMaximumAdjustmentRatio = 0.1f; // Ratio between the adjustment and the character's velocity to clamp the adjustment
        public bool DoClamping = true;
        [Range(0.0f, 2.0f)] public float MaxDistanceMMAndCharacterController = 0.1f; // Max distance between MotionMatching and the CharacterController
        [Header("DEBUG")]
        public bool DebugCurrent = true;
        public bool DebugPrediction = true;
        public bool DebugClamping = true;
        // --------------------------------------------------------------------------

        // PRIVATE ------------------------------------------------------------------
        // Input --------------------------------------------------------------------
        private float2 InputMovement;
        private bool OrientationFixed;
        // Rotation and Predicted Rotation ------------------------------------------
        private quaternion DesiredRotation; // Desired Rotation/Direction
        private quaternion[] PredictedRotations;
        private float3 AngularVelocity;
        private float3[] PredictedAngularVelocities;
        // Position and Predicted Position ------------------------------------------
        private float2[] PredictedPosition;
        private float2 Velocity;
        private float2[] PredictedVelocity;
        private float2 Acceleration;
        private float2[] PredictedAcceleration;
        // Features -----------------------------------------------------------------
        private int TrajectoryPosFeatureIndex;
        private int TrajectoryRotFeatureIndex;
        private int[] TrajectoryPosPredictionFrames;
        private int[] TrajectoryRotPredictionFrames;
        private int NumberPredictionPos { get { return TrajectoryPosPredictionFrames.Length; } }
        private int NumberPredictionRot { get { return TrajectoryRotPredictionFrames.Length; } }
        // Crowds ------------------------------------------------------------------
        private NativeArray<(float2, float)> ObstaclesArray;
        // --------------------------------------------------------------------------

        // FUNCTIONS ---------------------------------------------------------------
        private void Start()
        {
            // Get the feature indices
            TrajectoryPosFeatureIndex = -1;
            TrajectoryRotFeatureIndex = -1;
            for (int i = 0; i < MotionMatching.MMData.TrajectoryFeatures.Count; ++i)
            {
                if (MotionMatching.MMData.TrajectoryFeatures[i].Name == TrajectoryPositionFeatureName) TrajectoryPosFeatureIndex = i;
                if (MotionMatching.MMData.TrajectoryFeatures[i].Name == TrajectoryDirectionFeatureName) TrajectoryRotFeatureIndex = i;
            }
            Debug.Assert(TrajectoryPosFeatureIndex != -1, "Trajectory Position Feature not found");
            Debug.Assert(TrajectoryRotFeatureIndex != -1, "Trajectory Direction Feature not found");

            TrajectoryPosPredictionFrames = MotionMatching.MMData.TrajectoryFeatures[TrajectoryPosFeatureIndex].FramesPrediction;
            TrajectoryRotPredictionFrames = MotionMatching.MMData.TrajectoryFeatures[TrajectoryRotFeatureIndex].FramesPrediction;
            // TODO: generalize this... allow different number of prediction frames for different features
            Debug.Assert(TrajectoryPosPredictionFrames.Length == TrajectoryRotPredictionFrames.Length, "Trajectory Position and Trajectory Direction Prediction Frames must be the same for SpringCharacterController");
            for (int i = 0; i < TrajectoryPosPredictionFrames.Length; ++i)
            {
                Debug.Assert(TrajectoryPosPredictionFrames[i] == TrajectoryRotPredictionFrames[i], "Trajectory Position and Trajectory Direction Prediction Frames must be the same for SpringCharacterController");
            }

            PredictedPosition = new float2[NumberPredictionPos];
            PredictedVelocity = new float2[NumberPredictionPos];
            PredictedAcceleration = new float2[NumberPredictionPos];
            DesiredRotation = quaternion.LookRotation(transform.forward, transform.up);
            PredictedRotations = new quaternion[NumberPredictionRot];
            PredictedAngularVelocities = new float3[NumberPredictionRot];
            // Crowds
            ObstaclesArray = new NativeArray<(float2, float)>(Obstacles.Length, Allocator.Persistent);
        }

        // Input a change in the movement direction
        public void SetMovementDirection(float2 movementDirection)
        {
            float2 prevInputMovement = InputMovement;
            InputMovement = movementDirection;
            // Desired Rotation
            if (!OrientationFixed && math.length(movementDirection) > 0.0001f)
            {
                float2 desiredDirection = math.normalize(movementDirection);
                DesiredRotation = quaternion.LookRotation(new float3(desiredDirection.x, 0.0f, desiredDirection.y), transform.up);
            }
            // Input Changed Quickly
            if (math.dot(prevInputMovement, InputMovement) < InputBigChangeThreshold)
            {
                NotifyInputChangedQuickly();
            }
        }

        public void SwapFixOrientation()
        {
            OrientationFixed = !OrientationFixed;
        }

        protected override void OnUpdate()
        {
            // Rotations
            quaternion currentRotation = transform.rotation;
            PredictRotations(currentRotation, DatabaseDeltaTime);
            // Update Current Rotation
            quaternion newRot = ComputeNewRot(currentRotation);

            // Positions
            float2 desiredSpeed = InputMovement * MaxSpeed;
            float2 currentPos = new float2(MotionMatching.transform.position.x, MotionMatching.transform.position.z);
            // Predict
            PredictPositions(currentPos, desiredSpeed, DatabaseDeltaTime);
            // Update Current Position
            float2 newPos = ComputeNewPos(currentPos, desiredSpeed);

            // Update Character Controller
            //if (math.lengthsq(Velocity) > MinimumVelocityClamp * MinimumVelocityClamp)
            //{
            //    // Update Transform
            //    transform.position = new float3(newPos.x, transform.position.y, newPos.y);
            //    transform.rotation = newRot;
            //}
            transform.position = new float3(currentPos.x, transform.position.y, currentPos.y);
            transform.rotation = newRot;

            // Adjust MotionMatching to pull the Character towards the Character Controller
            if (DoAdjustment) AdjustMotionMatching();
            if (DoClamping) ClampMotionMatching();
        }

        private void PredictRotations(quaternion currentRotation, float averagedDeltaTime)
        {
            for (int i = 0; i < NumberPredictionRot; i++)
            {
                // Init Predicted values
                PredictedRotations[i] = currentRotation;
                PredictedAngularVelocities[i] = AngularVelocity;
                // Predict
                Spring.SimpleSpringDamperImplicit(ref PredictedRotations[i], ref PredictedAngularVelocities[i],
                                                  DesiredRotation, 1.0f - ResponsivenessDirections, TrajectoryRotPredictionFrames[i] * averagedDeltaTime);
            }
        }

        /* https://theorangeduck.com/page/spring-roll-call#controllers */
        private void PredictPositions(float2 currentPos, float2 desiredSpeed, float averagedDeltaTime)
        {
            int lastPredictionFrames = 0;
            for (int i = 0; i < NumberPredictionPos; ++i)
            {
                if (i == 0)
                {
                    PredictedPosition[i] = currentPos;
                    PredictedVelocity[i] = Velocity;
                    PredictedAcceleration[i] = Acceleration;
                }
                else
                {
                    PredictedPosition[i] = PredictedPosition[i - 1];
                    PredictedVelocity[i] = PredictedVelocity[i - 1];
                    PredictedAcceleration[i] = PredictedAcceleration[i - 1];
                }
                int diffPredictionFrames = TrajectoryPosPredictionFrames[i] - lastPredictionFrames;
                lastPredictionFrames = TrajectoryPosPredictionFrames[i];
                Spring.CharacterPositionUpdate(ref PredictedPosition[i], ref PredictedVelocity[i], ref PredictedAcceleration[i],
                                               desiredSpeed, 1.0f - ResponsivenessPositions, diffPredictionFrames * averagedDeltaTime);
            }
        }

        private quaternion ComputeNewRot(quaternion currentRotation)
        {
            quaternion newRotation = currentRotation;
            Spring.SimpleSpringDamperImplicit(ref newRotation, ref AngularVelocity, DesiredRotation, 1.0f - ResponsivenessDirections, Time.deltaTime);
            return newRotation;
        }

        private float2 ComputeNewPos(float2 currentPos, float2 desiredSpeed)
        {
            float2 newPos = currentPos;
            Spring.CharacterPositionUpdate(ref newPos, ref Velocity, ref Acceleration, desiredSpeed, 1.0f - ResponsivenessPositions, Time.deltaTime);
            return newPos;
        }

        private void AdjustMotionMatching()
        {
            AdjustCharacterPosition();
            AdjustCharacterRotation();
        }

        private void ClampMotionMatching()
        {
            // Clamp Position
            float3 characterController = transform.position;
            float3 motionMatching = MotionMatching.transform.position;
            if (math.distance(characterController, motionMatching) > MaxDistanceMMAndCharacterController)
            {
                float3 newMotionMatchingPos = MaxDistanceMMAndCharacterController * math.normalize(motionMatching - characterController) + characterController;
                MotionMatching.SetPosAdjustment(newMotionMatchingPos - motionMatching);
            }
        }

        private void AdjustCharacterPosition()
        {
            float3 characterController = transform.position;
            float3 motionMatching = MotionMatching.transform.position;
            float3 differencePosition = characterController - motionMatching;
            // Damp the difference using the adjustment halflife and dt
            float3 adjustmentPosition = Spring.DampAdjustmentImplicit(differencePosition, PositionAdjustmentHalflife, Time.deltaTime);
            // Clamp adjustment if the length is greater than the character velocity
            // multiplied by the ratio
            float maxLength = PosMaximumAdjustmentRatio * math.length(MotionMatching.Velocity) * Time.deltaTime;
            if (math.length(adjustmentPosition) > maxLength)
            {
                adjustmentPosition = maxLength * math.normalize(adjustmentPosition);
            }
            // Move the simulation bone towards the simulation object
            MotionMatching.SetPosAdjustment(adjustmentPosition);
        }

        private void AdjustCharacterRotation()
        {
            quaternion characterController = transform.rotation;
            quaternion motionMatching = MotionMatching.transform.rotation;
            // Find the difference in rotation (from character to simulation object)
            // Note: if numerically unstable, try quaternion.Normalize(quaternion.Inverse(characterController) * motionMatching)
            quaternion differenceRotation = math.mul(math.inverse(motionMatching), characterController);
            // Damp the difference using the adjustment halflife and dt
            quaternion adjustmentRotation = Spring.DampAdjustmentImplicit(differenceRotation, RotationAdjustmentHalflife, Time.deltaTime);
            // Clamp adjustment if the length is greater than the character angular velocity
            // multiplied by the ratio
            float maxLength = RotMaximumAdjustmentRatio * math.length(MotionMatching.AngularVelocity) * Time.deltaTime;
            if (math.length(MathExtensions.QuaternionToScaledAngleAxis(adjustmentRotation)) > maxLength)
            {
                adjustmentRotation = MathExtensions.QuaternionFromScaledAngleAxis(maxLength * math.normalize(MathExtensions.QuaternionToScaledAngleAxis(adjustmentRotation)));
            }
            // Rotate the simulation bone towards the simulation object
            MotionMatching.SetRotAdjustment(adjustmentRotation);
        }

        public float3 GetCurrentPosition()
        {
            return transform.position;
        }
        public quaternion GetCurrentRotation()
        {
            return transform.rotation;
        }

        public override void GetTrajectoryFeature(TrajectoryFeature feature, int index, Transform character, NativeArray<float> output)
        {
            if (!feature.SimulationBone) Debug.Assert(false, "Trajectory should be computed using the SimulationBone");
            switch (feature.FeatureType)
            {
                case TrajectoryFeature.Type.Position:
                    float2 world = PredictedPosition[index];
                    float3 local = character.InverseTransformPoint(new float3(world.x, 0.0f, world.y));
                    output[0] = local.x;
                    output[1] = local.z;
                    break;
                case TrajectoryFeature.Type.Direction:
                    float2 dirProjected = GetWorldSpaceDirectionPrediction(index);
                    float3 localDir = character.InverseTransformDirection(new Vector3(dirProjected.x, 0.0f, dirProjected.y));
                    output[0] = localDir.x;
                    output[1] = localDir.z;
                    break;
                default:
                    Debug.Assert(false, "Unknown feature type: " + feature.FeatureType);
                    break;
            }
        }

        // TODO: maybe get dynamic feature could return something more generic that allows to send custom data like the obstacle arrays
        public override void GetDynamicFeature(TrajectoryFeature feature, int index, Transform character, NativeArray<float> output)
        {
            if (feature.Name == "FutureEllipse")
            {
                output[0] = 0.0f;
                output[1] = 0.0f;
                output[2] = 0.0f;
                output[3] = 0.0f;
                for (int i = 0; i < Obstacles.Length; i++)
                {
                    float3 world = new(Obstacles[i].position.x, 0.0f, Obstacles[i].position.z);
                    float3 localPos = character.InverseTransformPoint(world);
                    // HARDCODED: circle radius
                    ObstaclesArray[i] = (new float2(localPos.x, localPos.z), Obstacles[i].localScale.x / 2.0f);
                }
                MotionMatching.SetObstacle(ObstaclesArray);
            }
            else
            {
                Debug.Assert(false, "Unknown feature name: " + feature.Name);
            }
        }

        private float2 GetWorldSpaceDirectionPrediction(int index)
        {
            float3 dir = math.mul(PredictedRotations[index], new float3(0, 0, 1));
            return math.normalize(new float2(dir.x, dir.z));
        }

        public override float3 GetWorldInitPosition()
        {
            return transform.position;
        }
        public override float3 GetWorldInitDirection()
        {
            return transform.forward;
        }

#if UNITY_EDITOR
        private void OnDrawGizmos()
        {
            const float radius = 0.05f;
            const float vectorReduction = 0.5f;
            const float verticalOffset = 0.05f;
            Vector3 transformPos = (Vector3)GetCurrentPosition() + Vector3.up * verticalOffset;
            if (DebugCurrent)
            {
                // Draw Current Position & Velocity
                Gizmos.color = new Color(1.0f, 0.3f, 0.1f, 1.0f);
                Gizmos.DrawSphere(transformPos, radius);
                GizmosExtensions.DrawLine(transformPos, transformPos + ((Quaternion)GetCurrentRotation() * Vector3.forward) * vectorReduction, 3);
            }

            if (PredictedPosition == null || PredictedRotations == null) return;

            if (DebugPrediction)
            {
                // Draw Predicted Position & Velocity
                Gizmos.color = new Color(0.6f, 0.3f, 0.8f, 1.0f);
                for (int i = 0; i < PredictedPosition.Length; ++i)
                {
                    float3 predictedPos = new float3(PredictedPosition[i].x, verticalOffset, PredictedPosition[i].y);
                    float2 predictedDir = GetWorldSpaceDirectionPrediction(i);
                    float3 predictedDir3D = new float3(predictedDir.x, 0.0f, predictedDir.y);
                    Gizmos.DrawSphere(predictedPos, radius);
                    GizmosExtensions.DrawLine(predictedPos, predictedPos + predictedDir3D * vectorReduction, 3);
                }
            }

            if (DebugClamping)
            {
                // Draw Clamp Circle
                if (DoClamping)
                {
                    Gizmos.color = new Color(0.1f, 1.0f, 0.1f, 1.0f);
                    GizmosExtensions.DrawWireCircle(transformPos, MaxDistanceMMAndCharacterController, quaternion.identity);
                }
            }
        }
#endif
    }
}