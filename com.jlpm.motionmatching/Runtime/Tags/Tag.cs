using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace MotionMatching
{
    [System.Serializable]
    public struct Tag
    {
        public string Name;
        public int[] Start; // Each element with index i, where, 0 <= i <= Start.Length == End.Length
        public int[] End;   // represents a range. That is, for an arbitrary i -> [Start[i], End[i]]
    }
}