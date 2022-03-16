using System.Collections;
using System.Collections.Generic;
using Unity.Barracuda;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
    public class Decompressor
    {
        private Model Model;
        private IWorker Worker;

        private float[] Input;

        public Decompressor(NNModel modelSource, int inputSize)
        {
            Model = ModelLoader.Load(modelSource);
            Input = new float[inputSize];
            Worker = WorkerFactory.CreateWorker(WorkerFactory.Type.CSharp, Model);
        }

        public void Decompress(NativeArray<float> feature, ref PoseVector pose)
        {
            Debug.Assert(feature.Length == Input.Length, "Input size mismatch");
            // Copy feature to input
            for (int i = 0; i < Input.Length; ++i) Input[i] = feature[i];
            // Tensor
            Tensor input = new Tensor(1, 1, 1, Input.Length, Input);
            // Run
            Worker.Execute(input);
            // Copy output
            Tensor output = Worker.PeekOutput();
            int count = 0;
            for (int i = 0; i < 23; ++i)
            {
                int index = i * 3 + count;
                pose.JointLocalPositions[i] = new float3(output[0, 0, 0, index], output[0, 0, 0, index + 1], output[0, 0, 0, index + 2]);
            }
            count += 23 * 3;
            for (int i = 0; i < 23; ++i)
            {
                int index = i * 4 + count;
                pose.JointLocalRotations[i] = new quaternion(output[0, 0, 0, index], output[0, 0, 0, index + 1], output[0, 0, 0, index + 2], output[0, 0, 0, index + 3]);
            }
            count += 23 * 4;
            for (int i = 0; i < 23; ++i)
            {
                int index = i * 3 + count;
                pose.JointVelocities[i] = new float3(output[0, 0, 0, index], output[0, 0, 0, index + 1], output[0, 0, 0, index + 2]);
            }
            count += 23 * 3;
            for (int i = 0; i < 23; ++i)
            {
                int index = i * 3 + count;
                pose.JointAngularVelocities[i] = new float3(output[0, 0, 0, index], output[0, 0, 0, index + 1], output[0, 0, 0, index + 2]);
            }
            count += 23 * 3;
            pose.RootDisplacement = new float3(output[0, 0, 0, count], output[0, 0, 0, count + 1], output[0, 0, 0, count + 2]);
            count += 3;
            pose.RootRotDisplacement = new quaternion(output[0, 0, 0, count], output[0, 0, 0, count + 1], output[0, 0, 0, count + 2], output[0, 0, 0, count + 3]);
            count += 4;
            pose.RootRotAngularVelocity = new float3(output[0, 0, 0, count], output[0, 0, 0, count + 1], output[0, 0, 0, count + 2]);
            count += 3;
            pose.RootWorld = new float3(output[0, 0, 0, count], output[0, 0, 0, count + 1], output[0, 0, 0, count + 2]);
            count += 3;
            pose.RootWorldRot = new quaternion(output[0, 0, 0, count], output[0, 0, 0, count + 1], output[0, 0, 0, count + 2], output[0, 0, 0, count + 3]);
            count += 4;
            // Dispose
            input.Dispose();
            output.Dispose();
        }
    }
}