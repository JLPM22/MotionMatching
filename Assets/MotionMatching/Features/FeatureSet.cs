using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    /// <summary>
    /// Stores all features vectors of all poses for Motion Matching
    /// </summary>
    public class FeatureSet
    {
        public FeatureVector[] Features { get; private set; }

        public FeatureSet(FeatureVector[] features)
        {
            Features = features;
        }
    }
}