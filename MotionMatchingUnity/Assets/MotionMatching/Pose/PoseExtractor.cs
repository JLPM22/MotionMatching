using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Jobs;
using Unity.Collections;
using Python.Runtime;
using System.IO;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;

    /// <summary>
    /// Extracts full pose for Motion Matching from BVHAnimation
    /// </summary>
    public class PoseExtractor
    {
        /// <summary>
        /// Extract the poses from bvhAnimation and store it in poseSet
        /// poseSet is not cleared, it will add bvhAnimation the the existing poses
        /// Returns true if the bvhAnimation was added to the poseSet, false otherwise
        /// </summary>
        public bool Extract(BVHAnimation bvhAnimation, PoseSet poseSet, MotionMatchingData mmData)
        {
            // Set Poses
            int nFrames = bvhAnimation.Frames.Length;
            PoseVector[] poses = new PoseVector[nFrames];
            for (int i = 0; i < nFrames; i++)
            {
                poses[i] = ExtractPose(bvhAnimation, i, mmData, poseSet, poses);
            }
            if (mmData.SmoothSimulationBone)
            {
                SmoothSimulationBone(poses, poseSet);
            }
            return poseSet.AddClip(poses, bvhAnimation.FrameTime);
        }

        private void SmoothSimulationBone(PoseVector[] poses, PoseSet poseSet)
        {
            // Save Hips world position and rotation before smoothing Simulation Bone (Root)
            float3[] hipsWorldPositions = new float3[poses.Length];
            quaternion[] hipsWorldRotations = new quaternion[poses.Length];
            if (!poseSet.Skeleton.Find(HumanBodyBones.Hips, out Joint hipsJoint))
            {
                Debug.LogError("Hips Joint not found");
            }
            for (int i = 0; i < poses.Length; i++)
            {
                hipsWorldPositions[i] = FeatureSet.GetWorldPosition(poseSet.Skeleton, poses[i], hipsJoint);
                hipsWorldRotations[i] = FeatureSet.GetWorldRotation(poseSet.Skeleton, poses[i], hipsJoint);
            }
            // Prepare simulation bone lists for python
            float[] sbPosX = new float[poses.Length];
            float[] sbPosY = new float[poses.Length];
            float[] sbPosZ = new float[poses.Length];
            float[] sbDirX = new float[poses.Length];
            float[] sbDirY = new float[poses.Length];
            float[] sbDirZ = new float[poses.Length];
            for (int i = 0; i < poses.Length; i++)
            {
                sbPosX[i] = poses[i].JointLocalPositions[0].x;
                sbPosY[i] = poses[i].JointLocalPositions[0].y;
                sbPosZ[i] = poses[i].JointLocalPositions[0].z;
                quaternion rot = poses[i].JointLocalRotations[0];
                float3 dir = math.mul(rot, new float3(0, 0, 1));
                sbDirX[i] = dir.x;
                sbDirY[i] = dir.y;
                sbDirZ[i] = dir.z;
            }
            // Call Python
            if (PythonEngine.IsInitialized) PythonEngine.Shutdown();
            Runtime.PythonDLL = Application.dataPath + "/StreamingAssets/Python/python-3.9.11-embed-amd64/python39.dll";
            PythonEngine.Initialize();
            var script = File.ReadAllText(Application.dataPath + "/StreamingAssets/Python/scripts/savgol_filter.py");
            using (Py.GIL())
            {
                using (PyModule scope = Py.CreateScope())
                {
                    PyList pyPosX = new PyList();
                    PyList pyPosY = new PyList();
                    PyList pyPosZ = new PyList();
                    PyList pyDirX = new PyList();
                    PyList pyDirY = new PyList();
                    PyList pyDirZ = new PyList();
                    for (int i = 0; i < poses.Length; i++)
                    {
                        pyPosX.Append(new PyFloat(sbPosX[i]));
                        pyPosY.Append(new PyFloat(sbPosY[i]));
                        pyPosZ.Append(new PyFloat(sbPosZ[i]));
                        pyDirX.Append(new PyFloat(sbDirX[i]));
                        pyDirY.Append(new PyFloat(sbDirY[i]));
                        pyDirZ.Append(new PyFloat(sbDirZ[i]));
                    }
                    scope.Set("pos_x", pyPosX);
                    scope.Set("pos_y", pyPosY);
                    scope.Set("pos_z", pyPosZ);
                    scope.Set("dir_x", pyDirX);
                    scope.Set("dir_y", pyDirY);
                    scope.Set("dir_z", pyDirZ);

                    scope.Exec(script);

                    PyList pyPosX_out = scope.Get<PyList>("pos_f_x");
                    float[] posX_out = pyPosX_out.As<float[]>();
                    PyList pyPosY_out = scope.Get<PyList>("pos_f_y");
                    float[] posY_out = pyPosY_out.As<float[]>();
                    PyList pyPosZ_out = scope.Get<PyList>("pos_f_z");
                    float[] posZ_out = pyPosZ_out.As<float[]>();
                    PyList pyDirX_out = scope.Get<PyList>("dir_f_x");
                    float[] dirX_out = pyDirX_out.As<float[]>();
                    PyList pyDirY_out = scope.Get<PyList>("dir_f_y");
                    float[] dirY_out = pyDirY_out.As<float[]>();
                    PyList pyDirZ_out = scope.Get<PyList>("dir_f_z");
                    float[] dirZ_out = pyDirZ_out.As<float[]>();
                    for (int i = 0; i < poses.Length; i++)
                    {
                        sbPosX[i] = posX_out[i];
                        sbPosY[i] = posY_out[i];
                        sbPosZ[i] = posZ_out[i];
                        sbDirX[i] = dirX_out[i];
                        sbDirY[i] = dirY_out[i];
                        sbDirZ[i] = dirZ_out[i];
                    }
                }
            }
            if (PythonEngine.IsInitialized) PythonEngine.Shutdown();
            // Set new Simulation Bone positions and rotations
            for (int i = 0; i < poses.Length; i++)
            {
                poses[i].JointLocalPositions[0] = new float3(sbPosX[i], sbPosY[i], sbPosZ[i]);
                float3 dir = new float3(sbDirX[i], sbDirY[i], sbDirZ[i]);
                dir = math.normalize(dir);
                poses[i].JointLocalRotations[0] = math.normalize(quaternion.LookRotation(dir, math.up()));
            }
            // Set new relative Hips position and rotation to the Simulation Bone
            for (int i = 0; i < poses.Length; i++)
            {
                float3 sbPos = poses[i].JointLocalPositions[0];
                quaternion inverseSBRot = math.inverse(poses[i].JointLocalRotations[0]);
                float3 newHipsPos = math.mul(inverseSBRot, hipsWorldPositions[i] - sbPos);
                quaternion newHipsRot = math.mul(inverseSBRot, hipsWorldRotations[i]);
                poses[i].JointLocalPositions[1] = newHipsPos;
                poses[i].JointLocalRotations[1] = newHipsRot;
            }
        }

        private PoseVector ExtractPose(BVHAnimation bvhAnimation, int frameIndex, MotionMatchingData mmData, PoseSet poseSet, PoseVector[] poses)
        {
            BVHAnimation.Frame frame = bvhAnimation.Frames[frameIndex];

            int nJoints = bvhAnimation.Skeleton.Joints.Count;
            int outNJoint = nJoints + 1; // +1 for SimulationBone

            // Native Arrays used by Burst for Output
            NativeArray<float3> jointLocalPositions = new NativeArray<float3>(outNJoint, Allocator.TempJob);
            NativeArray<quaternion> jointLocalRotations = new NativeArray<quaternion>(outNJoint, Allocator.TempJob);
            NativeArray<float3> jointLocalVelocities = new NativeArray<float3>(outNJoint, Allocator.TempJob);
            NativeArray<float3> jointLocalAngularVelocities = new NativeArray<float3>(outNJoint, Allocator.TempJob);

            // Native Arrays used by Burst for Input
            NativeArray<float3> jointOffsets = new NativeArray<float3>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) jointOffsets[i] = bvhAnimation.Skeleton.Joints[i].LocalOffset;
            NativeArray<quaternion> frameLocalRotations = new NativeArray<quaternion>(nJoints, Allocator.TempJob);
            for (int i = 0; i < nJoints; i++) frameLocalRotations[i] = frame.LocalRotations[i];

            // Use Burst (way faster than doing it in regular C#)
            var jobSB = new AddSimulationBoneBurst
            {
                // Input
                Njoints = nJoints,
                JointOffsets = jointOffsets,
                FrameLocalRotations = frameLocalRotations,
                FrameRootMotion = frame.RootMotion,
                HipsForwardLocalVector = mmData.HipsForwardLocalVector,
                // Output
                OutJointLocalPositions = jointLocalPositions,
                OutJointLocalRotations = jointLocalRotations,
            };
            jobSB.Schedule().Complete();

            // Dispose
            jointOffsets.Dispose();
            frameLocalRotations.Dispose();

            if (frameIndex == 0)
            {
                for (int i = 0; i < jointLocalVelocities.Length; i++) jointLocalVelocities[i] = float3.zero;
                for (int i = 0; i < jointLocalAngularVelocities.Length; i++) jointLocalAngularVelocities[i] = float3.zero;
            }
            else
            {
                PoseVector prevPos = poses[frameIndex - 1];

                // Native Arrays used by Burst for Input
                NativeArray<int> jointParents = new NativeArray<int>(outNJoint, Allocator.TempJob);
                for (int i = 0; i < outNJoint; i++) jointParents[i] = poseSet.Skeleton.Joints[i].ParentIndex; // poseSet skeleton already includes SimulationBone
                NativeArray<float3> prevLocalPositions = new NativeArray<float3>(outNJoint, Allocator.TempJob);
                for (int i = 0; i < outNJoint; i++) prevLocalPositions[i] = prevPos.JointLocalPositions[i];
                NativeArray<quaternion> prevLocalRotations = new NativeArray<quaternion>(outNJoint, Allocator.TempJob);
                for (int i = 0; i < outNJoint; i++) prevLocalRotations[i] = prevPos.JointLocalRotations[i];

                var jobVelocities = new ExtractVelocitiesBurst
                {
                    // Input
                    Njoints = outNJoint,
                    FrameTime = bvhAnimation.FrameTime,
                    JointParents = jointParents,
                    LocalPositions = jointLocalPositions,
                    PrevLocalPositions = prevLocalPositions,
                    LocalRotations = jointLocalRotations,
                    PrevLocalRotations = prevLocalRotations,
                    // Output
                    OutJointLocalVelocities = jointLocalVelocities,
                    OutJointLocalAngularVelocities = jointLocalAngularVelocities,
                };
                jobVelocities.Schedule().Complete();

                // Dispose
                jointParents.Dispose();
                prevLocalPositions.Dispose();
                prevLocalRotations.Dispose();
            }

            // Copy to managed arrays
            float3[] _jointLocalPositions = new float3[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointLocalPositions[i] = jointLocalPositions[i];
            quaternion[] _jointLocalRotations = new quaternion[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointLocalRotations[i] = jointLocalRotations[i];
            float3[] _jointVelocities = new float3[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointVelocities[i] = jointLocalVelocities[i];
            float3[] _jointAngularVelocities = new float3[outNJoint];
            for (int i = 0; i < outNJoint; i++) _jointAngularVelocities[i] = jointLocalAngularVelocities[i];

            // Dispose Output Native Arrays
            jointLocalPositions.Dispose();
            jointLocalRotations.Dispose();
            jointLocalVelocities.Dispose();
            jointLocalAngularVelocities.Dispose();

            // Result
            return new PoseVector(_jointLocalPositions, _jointLocalRotations,
                                  _jointVelocities, _jointAngularVelocities);
        }

        [BurstCompile]
        public struct AddSimulationBoneBurst : IJob
        {
            // Input arrays are of size nJoints (not including SimulationBone)
            [ReadOnly] public int Njoints;
            [ReadOnly] public NativeArray<float3> JointOffsets;
            [ReadOnly] public NativeArray<quaternion> FrameLocalRotations;
            [ReadOnly] public float3 FrameRootMotion;
            [ReadOnly] public float3 HipsForwardLocalVector;

            // Output arrays are of size nJoints + 1 (including SimulationBone)
            [WriteOnly] public NativeArray<float3> OutJointLocalPositions;
            [WriteOnly] public NativeArray<quaternion> OutJointLocalRotations;

            public void Execute()
            {
                // Joints
                for (int i = 0; i < Njoints; i++)
                {
                    OutJointLocalPositions[i + 1] = JointOffsets[i];
                    OutJointLocalRotations[i + 1] = FrameLocalRotations[i];
                }
                // SimulationBone
                // position and direction are hips projected on the ground
                float3 sbPos = new float3(FrameRootMotion.x, 0.0f, FrameRootMotion.z);
                float3 hipsForwardDir = math.mul(FrameLocalRotations[0], HipsForwardLocalVector);
                hipsForwardDir.y = 0;
                hipsForwardDir = math.normalize(hipsForwardDir);
                quaternion sbRot = quaternion.LookRotation(hipsForwardDir, math.up());
                OutJointLocalPositions[0] = sbPos;
                OutJointLocalRotations[0] = sbRot;
                // make first joint (hips) position and direction relative to the simulation bone
                OutJointLocalPositions[1] = math.mul(math.inverse(sbRot), FrameRootMotion - sbPos);
                OutJointLocalRotations[1] = math.mul(math.inverse(sbRot), FrameLocalRotations[0]);
            }
        }

        [BurstCompile]
        public struct ExtractVelocitiesBurst : IJob
        {
            // Input arrays include SimulationBone
            [ReadOnly] public int Njoints;
            [ReadOnly] public float FrameTime;
            [ReadOnly] public NativeArray<int> JointParents;
            [ReadOnly] public NativeArray<float3> LocalPositions;
            [ReadOnly] public NativeArray<float3> PrevLocalPositions;
            [ReadOnly] public NativeArray<quaternion> LocalRotations;
            [ReadOnly] public NativeArray<quaternion> PrevLocalRotations;

            // Output arrays include SimulationBone
            [WriteOnly] public NativeArray<float3> OutJointLocalVelocities;
            [WriteOnly] public NativeArray<float3> OutJointLocalAngularVelocities;

            public void Execute()
            {
                // Velocities
                for (int joint = 0; joint < Njoints; joint++)
                {
                    // Velocity
                    float3 pos = LocalPositions[joint];
                    float3 prevPos = PrevLocalPositions[joint];
                    float3 vel = (pos - prevPos) / FrameTime;
                    OutJointLocalVelocities[joint] = vel;
                    // Angular velocity
                    quaternion rot = LocalRotations[joint];
                    quaternion prevRot = PrevLocalRotations[joint];
                    float3 angularVel = MathExtensions.AngularVelocity(prevRot, rot, FrameTime);
                    OutJointLocalAngularVelocities[joint] = angularVel;
                }
            }
        }
    }
}