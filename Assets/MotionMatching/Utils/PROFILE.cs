// Uncomment this line to enable profiling for Motion Matching
#define PROFILE_MOTION_MATCHING

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
#if PROFILE_MOTION_MATCHING
using System.Diagnostics;
using Debug = UnityEngine.Debug;
#endif

namespace MotionMatching
{
    public static class PROFILE
    {
#if PROFILE_MOTION_MATCHING
        private static Dictionary<string, Stopwatch> Stopwatches = new Dictionary<string, Stopwatch>();
        // We need a separated ProfileSamples (instead of using Stopwatches) because we don't know when the ms/ticks will be queried
        // it may happen just after a stopwatch has been reseted
        private static Dictionary<string, DATA> ProfileDatas = new Dictionary<string, DATA>();
        private static Dictionary<string, long> StartMemory = new Dictionary<string, long>();
        private static Dictionary<string, long> EndMemory = new Dictionary<string, long>();
#endif
        // Functions must have empty body if PROFILE_MOTION_MATCHING is not defined
        // Compiler will remove them from the build
        public static void BEGIN_MEMORY_PROFILING(string tag)
        {
#if PROFILE_MOTION_MATCHING
            StartMemory[tag] = System.GC.GetTotalMemory(true);
#endif
        }
        public static void END_MEMORY_PROFILING(string tag)
        {
#if PROFILE_MOTION_MATCHING
            EndMemory[tag] = System.GC.GetTotalMemory(true);
#endif
        }
        public static void BEGIN_SAMPLE_PROFILING(string tag, bool sampleMemory = false)
        {
#if PROFILE_MOTION_MATCHING
            if (!Stopwatches.TryGetValue(tag, out Stopwatch stopwatch))
            {
                stopwatch = new Stopwatch();
                Stopwatches.Add(tag, stopwatch);
            }
            stopwatch.Reset();
            if (sampleMemory) BEGIN_MEMORY_PROFILING(tag);
            stopwatch.Start();
#endif
        }
        public static void END_SAMPLE_PROFILING(string tag, bool sampleMemory = false)
        {
#if PROFILE_MOTION_MATCHING
            Stopwatch stopwatch = Stopwatches[tag];
            stopwatch.Stop();
            if (sampleMemory) END_MEMORY_PROFILING(tag);
            if (!ProfileDatas.TryGetValue(tag, out DATA data))
            {
                data = new DATA();
                ProfileDatas.Add(tag, data);
            }
            data.AddSample(stopwatch.ElapsedMilliseconds, stopwatch.ElapsedTicks);
#endif
        }
        public static void END_AND_PRINT_SAMPLE_PROFILING(string tag, bool sampleMemory = false)
        {
#if PROFILE_MOTION_MATCHING
            END_SAMPLE_PROFILING(tag, sampleMemory);
            Stopwatch stopwatch = Stopwatches[tag];
            Debug.Log("[PROFILER]" + tag + ": " + stopwatch.ElapsedMilliseconds + "ms" + "(" + stopwatch.ElapsedTicks + " ticks)");
#endif
        }
        public static DATA GET_DATA(string tag)
        {
#if PROFILE_MOTION_MATCHING
            if (ProfileDatas.ContainsKey(tag))
            {
                return ProfileDatas[tag];
            }
            return null;
#endif
        }
        /// <summary>
        /// Memory returned in bytes
        /// </summary>
        public static long GET_MEMORY(string tag)
        {
#if PROFILE_MOTION_MATCHING
            if (StartMemory.ContainsKey(tag) && EndMemory.ContainsKey(tag))
            {
                return EndMemory[tag] - StartMemory[tag];
            }
            return -1;
#endif
        }
        public static void PRINT_MEMORY(string tag)
        {
#if PROFILE_MOTION_MATCHING
            if (StartMemory.ContainsKey(tag) && EndMemory.ContainsKey(tag))
            {
                Debug.Log(tag + ": " + ((EndMemory[tag] - StartMemory[tag]) / (1024 * 1024)) + " MiB");
            }
#endif
        }
        /// <summary>
        /// Use it only for Editor functions, otherwise is better to use a compiler directive such as #define
        /// </summary>
        public static bool IS_PROFILING_ENABLED()
        {
#if PROFILE_MOTION_MATCHING
            return true;
#endif
#pragma warning disable 162
            return false;
        }

#if PROFILE_MOTION_MATCHING
        public class DATA
        {
            private static readonly int NumberSamplesToAverage = 600;

            public float MinTicks, MinMs;
            public float MaxTicks, MaxMs;
            private float AverageTicks, AverageMs;

            private float[] SamplesToAverageMs;
            private float[] SamplesToAverageTicks;
            private int CurrentSample;
            private int AbsoluteCurrentSample;

            public DATA()
            {
                MinTicks = float.MaxValue;
                MinMs = float.MaxValue;
                MaxTicks = float.MinValue;
                MaxMs = float.MinValue;
                AverageTicks = 0;
                AverageMs = 0;
                SamplesToAverageTicks = new float[NumberSamplesToAverage];
                SamplesToAverageMs = new float[NumberSamplesToAverage];
                CurrentSample = 0;
            }

            public void AddSample(float sampleMs, float sampleTicks)
            {
                MinTicks = Mathf.Min(MinTicks, sampleTicks);
                MinMs = Mathf.Min(MinMs, sampleMs);
                MaxTicks = Mathf.Max(MaxTicks, sampleTicks);
                MaxMs = Mathf.Max(MaxMs, sampleMs);

                SamplesToAverageTicks[CurrentSample] = sampleTicks;
                SamplesToAverageMs[CurrentSample] = sampleMs;
                AbsoluteCurrentSample += 1;
                CurrentSample = AbsoluteCurrentSample % NumberSamplesToAverage;

                AverageTicks = 0;
                AverageMs = 0;
                int count = Mathf.Min(AbsoluteCurrentSample, NumberSamplesToAverage);
                for (int i = 0; i < count; i++)
                {
                    AverageTicks += SamplesToAverageTicks[i];
                    AverageMs += SamplesToAverageMs[i];
                }
                AverageTicks /= count;
                AverageMs /= count;
            }

            public float GetAverageTicks()
            {
                return AverageTicks;
            }
            public float GetAverageMs()
            {
                return AverageMs;
            }
        }
#endif
    }
}