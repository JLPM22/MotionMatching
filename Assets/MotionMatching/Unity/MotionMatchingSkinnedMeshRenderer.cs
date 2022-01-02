using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace MotionMatching
{
    public class MotionMatchingSkinnedMeshRenderer : MonoBehaviour
    {
        public GameObject MotionMatching;

        private void Start()
        {
            Transform[] bones = MotionMatching.transform.GetChild(0).GetComponentsInChildren<Transform>();
            Dictionary<string, Transform> boneDict = new Dictionary<string, Transform>();
            foreach (Transform bone in bones)
            {
                boneDict.Add(bone.name, bone);
            }

            SkinnedMeshRenderer[] skinnedMeshRenderers = GetComponentsInChildren<SkinnedMeshRenderer>();
            foreach (SkinnedMeshRenderer skinnedMeshRenderer in skinnedMeshRenderers)
            {
                Transform[] bonesRenderer = new Transform[skinnedMeshRenderer.bones.Length];
                for (int i = 0; i < skinnedMeshRenderer.bones.Length; i++)
                {
                    if (boneDict.ContainsKey(skinnedMeshRenderer.bones[i].name))
                    {
                        bonesRenderer[i] = boneDict[skinnedMeshRenderer.bones[i].name];
                    }
                    else
                    {
                        Transform parent = skinnedMeshRenderer.bones[i].parent;
                        Transform newBone = new GameObject(skinnedMeshRenderer.bones[i].name).transform;
                        newBone.SetParent(boneDict[parent.name], false);
                        newBone.localPosition = skinnedMeshRenderer.bones[i].localPosition;
                        newBone.localRotation = skinnedMeshRenderer.bones[i].localRotation;
                        newBone.localScale = skinnedMeshRenderer.bones[i].localScale;
                        boneDict.Add(newBone.name, newBone);
                        bonesRenderer[i] = newBone;
                    }
                }
                skinnedMeshRenderer.bones = bonesRenderer;
                skinnedMeshRenderer.rootBone = bonesRenderer[0];
            }
        }
    }
}