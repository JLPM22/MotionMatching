using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Text.RegularExpressions;
using UnityEngine;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;
    using EndSite = BVHAnimation.EndSite;
    using Frame = BVHAnimation.Frame;

    /// <summary>
    /// Imports a BVH file and stores the animation data in Unity format (BVHAnimation).
    /// </summary>
    public class BVHImporter
    {
        public BVHAnimation Import(TextAsset bvh, float scale = 1.0f, bool onlyFirstFrame = false)
        {
            List<AxisOrder> channels = new List<AxisOrder>();
            BVHAnimation animation = new BVHAnimation();

            Stack<int> parentIndexStack = new Stack<int>();
            char[] whitespace = new char[] { ' ', '\t', '\r', '\n' };
            string[] words = bvh.text.Split(whitespace, System.StringSplitOptions.RemoveEmptyEntries);
            // string[] words = Regex.Split(bvh.text, "[\\s+|\\r*\\n+]+");
            int w = 0;
            // ROOT
            if (words[w++] != "HIERARCHY") Debug.LogError("[BVHImporter] HIERARCHY not found");
            if (words[w++] != "ROOT") Debug.LogError("[BVHImporter] ROOT not found");
            Joint root = new Joint(words[w++], 0, 0, Vector3.zero);
            ReadLeftBracket(words, ref w);
            root.LocalOffset = ReadOffset(words, ref w) * scale;
            root.LocalOffset = Vector3.zero; // even if we read the offset, it is not used... it should always be 0...
            ReadChannels(channels, words, ref w, true);
            animation.AddJoint(root);
            // JOINTS
            int brackets = 1;
            int parent = 0;
            int jointIndex = 1;
            int it = 100000;
            if (ReadRightBracket(words, ref w)) brackets -= 1;
            while (brackets > 0 && --it > 0)
            {
                if (words[w++] != "JOINT") Debug.LogError("[BVHImporter] JOINT not found");
                Joint joint = new Joint(words[w++], jointIndex++, parent, Vector3.zero);
                ReadLeftBracket(words, ref w);
                parentIndexStack.Push(parent);
                parent = jointIndex - 1;
                brackets += 1;
                joint.LocalOffset = ReadOffset(words, ref w) * scale;
                ReadChannels(channels, words, ref w);
                animation.AddJoint(joint);
                if (words[w] == "End")
                {
                    w += 1;
                    if (words[w++] != "Site") Debug.LogError("[BVHImporter] End Site not found");
                    ReadLeftBracket(words, ref w);
                    Vector3 offset = ReadOffset(words, ref w) * scale;
                    EndSite endSite = new EndSite(parent, offset);
                    animation.AddEndSite(endSite);
                    if (!ReadRightBracket(words, ref w)) Debug.LogError("[BVHImporter] End Site right bracket not found");
                }
                while (ReadRightBracket(words, ref w))
                {
                    brackets -= 1;
                    if (parentIndexStack.Count > 0) parent = parentIndexStack.Pop();
                }
            }
            if (it <= 0) Debug.LogError("[BVHImporter] Infinite loop detected, Left and Right brackets does not match");

            // MOTION
            if (words[w++] != "MOTION") Debug.LogError("[BVHImporter] MOTION not found");
            if (words[w++] != "Frames:") Debug.LogError("[BVHImporter] Frames: not found");
            int numberFrames = int.Parse(words[w++]);
            animation.InitFrames(numberFrames);
            if (words[w++] != "Frame") Debug.LogError("[BVHImporter] Frame not found");
            if (words[w++] != "Time:") Debug.LogError("[BVHImporter] Time: not found");
            float frameTime = float.Parse(words[w++], CultureInfo.InvariantCulture);
            animation.SetFrameTime(frameTime);
            // Frames
            int numberChannels = channels.Count;
            numberFrames = onlyFirstFrame ? Mathf.Min(1, numberFrames) : numberFrames;
            for (int i = 0; i < numberFrames; i++)
            {
                Vector3 rootMotion = new Vector3();
                Quaternion[] localRotations = new Quaternion[numberChannels - 1];
                for (int j = 0; j < numberChannels; ++j)
                {
                    float v1 = float.Parse(words[w++], CultureInfo.InvariantCulture);
                    float v2 = float.Parse(words[w++], CultureInfo.InvariantCulture);
                    float v3 = float.Parse(words[w++], CultureInfo.InvariantCulture);
                    AxisOrder axisOrder = channels[j];
                    if (j == 0)
                    {
                        rootMotion = BVHToUnityTranslation(v1, v2, v3, axisOrder) * scale;
                    }
                    else
                    {
                        localRotations[j - 1] = BVHToUnityRotation(v1, v2, v3, axisOrder);
                    }
                }
                Frame frame = new Frame(rootMotion, localRotations);
                animation.AddFrame(i, frame);
            }
            return animation;
        }

        private void ReadLeftBracket(string[] words, ref int w)
        {
            if (words[w++] != "{") Debug.LogError("[BVHImporter] { not found");
        }

        private bool ReadRightBracket(string[] words, ref int w)
        {
            bool isRightBracket = words[w] == "}";
            if (isRightBracket)
            {
                w += 1;
            }
            return isRightBracket;
        }

        private Vector3 ReadOffset(string[] words, ref int w)
        {
            Vector3 offset = Vector3.zero;
            if (words[w++] != "OFFSET") Debug.LogError("[BVHImporter] OFFSET not found");
            offset.x = float.Parse(words[w++], CultureInfo.InvariantCulture);
            offset.y = float.Parse(words[w++], CultureInfo.InvariantCulture);
            offset.z = -float.Parse(words[w++], CultureInfo.InvariantCulture); // Unity is left-handed and BVH is right-handed (Z is opposite sign)
            return offset;
        }

        private void ReadChannels(List<AxisOrder> channels, string[] words, ref int w, bool root = false)
        {
            if (words[w++] != "CHANNELS") Debug.LogError("[BVHImporter] CHANNELS not found");
            if (root)
            {
                if (int.Parse(words[w++]) != 6) Debug.LogError("[BVHImporter] root must have 6 channels");
                channels.Add(ReadChannelPosition(words, ref w));
            }
            else
            {
                if (int.Parse(words[w++]) != 3) Debug.LogError("[BVHImporter] all joints must have 3 channels");
            }
            channels.Add(ReadChannelRotation(words, ref w));
        }

        private AxisOrder ReadChannelPosition(string[] words, ref int w)
        {
            string order1 = words[w++];
            string order2 = words[w++];
            string order3 = words[w++];
            if (order1 != "Xposition" && order1 != "Yposition" && order1 != "Zposition") Debug.LogError("[BVHImporter] root position channels must be Xposition, Yposition or Zposition");
            if (order2 != "Xposition" && order2 != "Yposition" && order2 != "Zposition") Debug.LogError("[BVHImporter] root position channels must be Xposition, Yposition or Zposition");
            if (order3 != "Xposition" && order3 != "Yposition" && order3 != "Zposition") Debug.LogError("[BVHImporter] root position channels must be Xposition, Yposition or Zposition");
            if (order1 == "Xposition")
            {
                if (order2 == "Yposition")
                {
                    if (order3 == "Zposition") return AxisOrder.XYZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
                else
                {
                    if (order3 == "Yposition") return AxisOrder.XZY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
            }
            else if (order1 == "Yposition")
            {
                if (order2 == "Xposition")
                {
                    if (order3 == "Zposition") return AxisOrder.YXZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
                else
                {
                    if (order3 == "Xposition") return AxisOrder.YZX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
            }
            else
            {
                if (order2 == "Xposition")
                {
                    if (order3 == "Yposition") return AxisOrder.ZXY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
                else
                {
                    if (order3 == "Xposition") return AxisOrder.ZYX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xposition, Yposition and Zposition");
                }
            }
            return AxisOrder.None;
        }

        private AxisOrder ReadChannelRotation(string[] words, ref int w)
        {
            string order1 = words[w++];
            string order2 = words[w++];
            string order3 = words[w++];
            if (order1 != "Xrotation" && order1 != "Yrotation" && order1 != "Zrotation") Debug.LogError("[BVHImporter] root or joint rotation channels must be Xrotation, Yrotation or Zrotation");
            if (order2 != "Xrotation" && order2 != "Yrotation" && order2 != "Zrotation") Debug.LogError("[BVHImporter] root or joint rotation channels must be Xrotation, Yrotation or Zrotation");
            if (order3 != "Xrotation" && order3 != "Yrotation" && order3 != "Zrotation") Debug.LogError("[BVHImporter] root or joint rotation channels must be Xrotation, Yrotation or Zrotation");
            if (order1 == "Xrotation")
            {
                if (order2 == "Yrotation")
                {
                    if (order3 == "Zrotation") return AxisOrder.XYZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
                else
                {
                    if (order3 == "Yrotation") return AxisOrder.XZY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
            }
            else if (order1 == "Yrotation")
            {
                if (order2 == "Xrotation")
                {
                    if (order3 == "Zrotation") return AxisOrder.YXZ;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
                else
                {
                    if (order3 == "Xrotation") return AxisOrder.YZX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
            }
            else
            {
                if (order2 == "Xrotation")
                {
                    if (order3 == "Yrotation") return AxisOrder.ZXY;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
                else
                {
                    if (order3 == "Xrotation") return AxisOrder.ZYX;
                    else Debug.LogError("[BVHImporter] root position channels must contain Xrotation, Yrotation and Zrotation");
                }
            }
            return AxisOrder.None;
        }

        private Quaternion BVHToUnityRotation(float v1, float v2, float v3, AxisOrder rotationOrder)
        {
            if (rotationOrder == AxisOrder.None) Debug.LogError("[BVHImporter] rotationOrder is None. There was an error while reading the channels");

            switch (rotationOrder)
            {
                // Why some are negative? Because Unity is left-handed and BVH is right-handed. See: https://stackoverflow.com/questions/31191752/right-handed-euler-angles-xyz-to-left-handed-euler-angles-xyz
                case AxisOrder.XYZ: return Quaternion.AngleAxis(-v1, Vector3.right) * Quaternion.AngleAxis(-v2, Vector3.up) * Quaternion.AngleAxis(v3, Vector3.forward); // XYZ
                case AxisOrder.XZY: return Quaternion.AngleAxis(-v1, Vector3.right) * Quaternion.AngleAxis(v2, Vector3.forward) * Quaternion.AngleAxis(-v3, Vector3.up); // XZY
                case AxisOrder.YXZ: return Quaternion.AngleAxis(-v1, Vector3.up) * Quaternion.AngleAxis(-v2, Vector3.right) * Quaternion.AngleAxis(v3, Vector3.forward); // YXZ
                case AxisOrder.YZX: return Quaternion.AngleAxis(-v1, Vector3.up) * Quaternion.AngleAxis(v2, Vector3.forward) * Quaternion.AngleAxis(-v3, Vector3.right); // YZX
                case AxisOrder.ZXY: return Quaternion.AngleAxis(v1, Vector3.forward) * Quaternion.AngleAxis(-v2, Vector3.right) * Quaternion.AngleAxis(-v3, Vector3.up); // ZXY
                case AxisOrder.ZYX: return Quaternion.AngleAxis(v1, Vector3.forward) * Quaternion.AngleAxis(-v2, Vector3.up) * Quaternion.AngleAxis(-v3, Vector3.right); // ZYX
            }

            return Quaternion.identity;
        }

        private Vector3 BVHToUnityTranslation(float v1, float v2, float v3, AxisOrder translationOrder)
        {
            if (translationOrder == AxisOrder.None) Debug.LogError("[BVHImporter] translationOrder is None. There was an error while reading the channels");

            // BVH's z+ axis is Unity's left (z-) (Unity is left-handed BVH is right-handed)
            switch (translationOrder)
            {
                case AxisOrder.XYZ: return new Vector3(v1, v2, -v3); // XYZ
                case AxisOrder.XZY: return new Vector3(v1, v3, -v2); // XZY
                case AxisOrder.YXZ: return new Vector3(v2, v1, -v3); // YXZ
                case AxisOrder.YZX: return new Vector3(v3, v1, -v2); // YZX
                case AxisOrder.ZXY: return new Vector3(v2, v3, -v1); // ZXY
                case AxisOrder.ZYX: return new Vector3(v3, v2, -v1); // ZYX
            }
            return Vector3.zero;
        }

        private enum AxisOrder
        {
            XYZ, XZY, YXZ, YZX, ZXY, ZYX, None
        }

    }
}