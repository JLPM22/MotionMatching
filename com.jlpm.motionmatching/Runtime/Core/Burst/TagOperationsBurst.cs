using System.Collections;
using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;

namespace MotionMatching
{
    [BurstCompile]
    public struct DisableTagBurst : IJob
    {
        [WriteOnly] public NativeArray<bool> TagMask;

        public void Execute()
        {
            for (int i = 0; i < TagMask.Length; i++)
            {
                TagMask[i] = true;
            }
        }
    }

    [BurstCompile]
    public struct SetTagBurst : IJob
    {
        [ReadOnly] public int MaximumFramesPrediction; // Number of prediction frames of the longest trajectory feature
        [ReadOnly] public NativeArray<int> StartRanges;
        [ReadOnly] public NativeArray<int> EndRanges;
        [WriteOnly] public NativeArray<bool> TagMask;

        public void Execute()
        {
            for (int i = 0; i < TagMask.Length; i++)
            {
                TagMask[i] = false;
            }
            for (int i = 0; i < StartRanges.Length; i++)
            {
                int start = StartRanges[i];
                int end = EndRanges[i];
                for (int j = start; j < end - MaximumFramesPrediction; j++)
                {
                    TagMask[j] = true;
                }
            }
        }
    }
}