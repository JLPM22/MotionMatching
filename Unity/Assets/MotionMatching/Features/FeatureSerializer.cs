using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace MotionMatching
{
    using static BinarySerializerExtensions;

    public class FeatureSerializer
    {
        /// <summary>
        /// Stores the features for Motion Matching in a binary format
        /// in the specified path with name filename and extension .mmfeatures
        /// </summary>
        public void Serialize(FeatureSet featureSet, string path, string fileName)
        {
            Directory.CreateDirectory(path); // create directory and parent directories if they don't exist

            // Write Features
            using (var stream = File.Open(Path.Combine(path, fileName + ".mmfeatures"), FileMode.Create))
            {
                using (var writer = new BinaryWriter(stream))
                {
                    // Serialize Number Feature Vectors
                    writer.Write((uint)featureSet.NumberFeatureVectors);
                    // Serialize Number Features Dimension
                    // HARDCODED: allow for variable dimension
                    const uint featuresDimension = 27;
                    writer.Write(featuresDimension);
                    // Serialize Number Features
                    // HARDCODED: allow for variable number of features
                    const uint numberFeatures = 7;
                    writer.Write(numberFeatures);

                    // Serialize Mean & StandardDeviation
                    for (int i = 0; i < featuresDimension; ++i)
                    {
                        writer.Write(featureSet.GetMean(i));
                        writer.Write(featureSet.GetStandardDeviation(i));
                    }

                    // Serialize Features
                    // HARDCODED: allow for variable number of features
                    writer.Write("FutureTrajectoryLocalPosition");
                    writer.Write((uint)2);
                    writer.Write((uint)3);

                    writer.Write("FutureTrajectoryLocalDirection");
                    writer.Write((uint)2);
                    writer.Write((uint)3);

                    writer.Write("LeftFootLocalPosition");
                    writer.Write((uint)3);
                    writer.Write((uint)1);

                    writer.Write("RightFootLocalPosition");
                    writer.Write((uint)3);
                    writer.Write((uint)1);

                    writer.Write("LeftFootLocalVelocity");
                    writer.Write((uint)3);
                    writer.Write((uint)1);

                    writer.Write("RightFootLocalVelocity");
                    writer.Write((uint)3);
                    writer.Write((uint)1);

                    writer.Write("HipsLocalVelocity");
                    writer.Write((uint)3);
                    writer.Write((uint)1);

                    // Serialize Feature Vectors
                    for (int i = 0; i < featureSet.NumberFeatureVectors; ++i)
                    {
                        FeatureVector featureVector = featureSet.GetFeature(i);
                        writer.Write((uint)(featureVector.IsValid ? 1 : 0));
                        // HARDCODED: allow for variable number of features
                        for (int j = 0; j < 3; ++j)
                        {
                            WriteFloat2(writer, featureVector.GetFutureTrajectoryLocalPosition(j));
                        }
                        for (int j = 0; j < 3; ++j)
                        {
                            WriteFloat2(writer, featureVector.GetFutureTrajectoryLocalDirection(j));
                        }
                        WriteFloat3(writer, featureVector.LeftFootLocalPosition);
                        WriteFloat3(writer, featureVector.RightFootLocalPosition);
                        WriteFloat3(writer, featureVector.LeftFootLocalVelocity);
                        WriteFloat3(writer, featureVector.RightFootLocalVelocity);
                        WriteFloat3(writer, featureVector.HipsLocalVelocity);
                    }
                }
            }
        }

        /// <summary>
        /// Reads the features for Motion Matching from a binary format
        /// from the specified path with name filename and extension .mmfeatures
        /// Returns true if featureSet was successfully deserialized, false otherwise
        /// </summary>
        public bool Deserialize(string path, string fileName, out FeatureSet featureSet)
        {
            featureSet = null;

            // Read Features
            string featuresPath = Path.Combine(path, fileName + ".mmfeatures");
            if (File.Exists(featuresPath))
            {
                using (var stream = File.Open(featuresPath, FileMode.Open))
                {
                    using (var reader = new BinaryReader(stream))
                    {
                        // Deserialize Number Feature Vectors
                        uint numberFeatureVectors = reader.ReadUInt32();
                        // Deserialize Number Features Dimension
                        uint featuresDimension = reader.ReadUInt32();
                        // Deserialize Number Features
                        uint numberFeatures = reader.ReadUInt32();

                        // Deserialize Mean & StandardDeviation
                        float[] mean = new float[featuresDimension];
                        float[] standardDeviation = new float[featuresDimension];
                        for (int i = 0; i < featuresDimension; ++i)
                        {
                            mean[i] = reader.ReadSingle();
                            standardDeviation[i] = reader.ReadSingle();
                        }

                        // Deserialize Features
                        // HARDCODED: allow for variable number of features
                        for (int i = 0; i < numberFeatures; ++i)
                        {
                            reader.ReadString();
                            reader.ReadUInt32();
                            reader.ReadUInt32();
                        }

                        // Deserialize Feature Vectors
                        FeatureVector[] featureVectors = new FeatureVector[numberFeatureVectors];
                        for (int i = 0; i < numberFeatureVectors; ++i)
                        {
                            FeatureVector featureVector = new FeatureVector();
                            featureVector.IsValid = reader.ReadUInt32() == 1;
                            // HARDCODED: allow for variable number of features
                            for (int j = 0; j < 3; ++j)
                            {
                                featureVector.SetFutureTrajectoryLocalPosition(j, ReadFloat2(reader));
                            }
                            for (int j = 0; j < 3; ++j)
                            {
                                featureVector.SetFutureTrajectoryLocalDirection(j, ReadFloat2(reader));
                            }
                            featureVector.LeftFootLocalPosition = ReadFloat3(reader);
                            featureVector.RightFootLocalPosition = ReadFloat3(reader);
                            featureVector.LeftFootLocalVelocity = ReadFloat3(reader);
                            featureVector.RightFootLocalVelocity = ReadFloat3(reader);
                            featureVector.HipsLocalVelocity = ReadFloat3(reader);
                            featureVectors[i] = featureVector;
                        }

                        featureSet = new FeatureSet(featureVectors);
                        featureSet.SetMean(mean);
                        featureSet.SetStandardDeviation(standardDeviation);
                    }
                }
            }
            else return false;

            return true;
        }
    }
}