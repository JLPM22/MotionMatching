using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Linq.Expressions;
using System.Text;
using Unity.Collections;
using UnityEngine;

namespace MotionMatching
{
    /// <summary>
    /// Class used to store the operational graph of a tag query (union, intersection and difference)
    /// Note that the query is not actually evaluated until it is passed to the MotionMatchingController
    /// </summary>
    public class QueryTag : IDisposable
    {
        private enum Operation
        {
            Or = -1,
            And = -2,
            Diff = -3,
        }

        private int[] Graph; // negative int are operations and positive int are tags (indices of Tags)
        private string[] Tags;
        private bool Dirty;

        private NativeArray<int> StartRanges;
        private NativeArray<int> EndRanges;

        public QueryTag() { Dirty = true; }
        
        public QueryTag(string tag)
        {
            Tags = new string[1];
            Tags[0] = tag;
            Graph = new int[1];
            Graph[0] = 0;
            Dirty = true;
        }

        public string[] GetTags() { return Tags; }
        public NativeArray<int> GetStartRanges() 
        {
            Debug.Assert(!Dirty, "Call ComputeRanges(...) before accesing GetStartRanges() when the QueryTag is marked as Dirty");
            return StartRanges; 
        }
        public NativeArray<int> GetEndRanges()
        {
            Debug.Assert(!Dirty, "Call ComputeRanges(...) before accesing GetStartRanges() when the QueryTag is marked as Dirty");
            return EndRanges;
        }

        public void ComputeRanges(PoseSet poseSet, List<MotionMatching.AnimationData.Tag> tags=null, bool force=false)
        {
            if (!Dirty && !force) return;

            Dispose();

            List<NativeArray<int>> startRanges = new(); // auxiliary ranges computed from operations
            List<NativeArray<int>> endRanges = new();   // indices are represented in Graph as Tags.Length + i (where i is an index of startRanges or endRanges).
            List<NativeArray<int>> toDelete = new();

            bool GetRangesFromGraphIndex(int g, out NativeArray<int> start, out NativeArray<int> end)
            {
                if (g < Tags.Length) // single tag
                {
                    if (poseSet == null)
                    {
                        Debug.Assert(tags != null);
                        foreach (var tag in tags)
                        {
                            if (tag.Name == Tags[g] && tag.Start != null)
                            {
                                start = new NativeArray<int>(tag.Start, Allocator.Temp);
                                end = new NativeArray<int>(tag.End, Allocator.Temp);
                                toDelete.Add(start);
                                toDelete.Add(end);
                                return true;
                            }
                        }
                        start = new NativeArray<int>();
                        end = new NativeArray<int>();
                        return false;
                    }
                    else
                    {
                        PoseSet.Tag tag = poseSet.GetTag(Tags[g]);
                        start = tag.GetStartRanges();
                        end = tag.GetEndRanges();
                    }
                }
                else // at least an operation was performed
                {
                    int index = g - Tags.Length;
                    start = startRanges[index];
                    end = endRanges[index];
                }
                return true;
            }

            Queue<int> graph = new Queue<int>(); // queue used to process Graph
            foreach (int g in Graph)
            {
                graph.Enqueue(g);
            }

            while (graph.Count > 1)
            {
                int v = graph.Dequeue();
                if (v < 0) // operation
                {
                    if (graph.Peek() < 0) // another op
                    {
                        // enqueue current op and process the next op
                        graph.Enqueue(v);
                    }
                    else // first tag
                    {
                        int t1 = graph.Dequeue();
                        if (graph.Peek() < 0) // another op
                        {
                            // enqueue both current op and tag, and process next op
                            graph.Enqueue(v);
                            graph.Enqueue(t1);
                        }
                        else // second tag
                        {
                            int t2 = graph.Dequeue();
                            GetRangesFromGraphIndex(t1, out NativeArray<int> start1, out NativeArray<int> end1);
                            GetRangesFromGraphIndex(t2, out NativeArray<int> start2, out NativeArray<int> end2);
                            NativeArray<int> startResult;
                            NativeArray<int> endResult;
                            if ((Operation)v == Operation.Or)
                            {
                                Or(start1, end1, start2, end2, out startResult, out endResult);
                            }
                            else if ((Operation)v == Operation.And)
                            {
                                And(start1, end1, start2, end2, out startResult, out endResult);
                            }
                            else // if ((Operation)v == Operation.Diff)
                            {
                                Diff(start1, end1, start2, end2, out startResult, out endResult);
                            }
                            // Enqueue results
                            startRanges.Add(startResult);
                            endRanges.Add(endResult);
                            graph.Enqueue(Tags.Length + startRanges.Count - 1);
                        }
                    }
                }
                else
                {
                    graph.Enqueue(v);
                }
            }

            // Convert to NativeArray
            Debug.Assert(graph.Count == 1 || graph.Count == 0);
            if (graph.Count > 0) // not empty query
            {
                int g = graph.Dequeue();
                if (GetRangesFromGraphIndex(g, out NativeArray<int> start, out NativeArray<int> end))
                {
                    StartRanges = new NativeArray<int>(start, Allocator.Persistent);
                    EndRanges = new NativeArray<int>(end, Allocator.Persistent);
                }
            }
            
            // Dispose auxiliary ranges
            foreach (NativeArray<int> array in startRanges)
            {
                array.Dispose();
            }
            foreach (NativeArray<int> array in endRanges)
            {
                array.Dispose();
            }
            foreach (NativeArray<int> array in toDelete)
            {
                array.Dispose();
            }

            Dirty = false;
        }

        public void Dispose()
        {
            if (StartRanges != null && StartRanges.IsCreated) StartRanges.Dispose();
            if (EndRanges != null && EndRanges.IsCreated) EndRanges.Dispose();
        }

        private void Or(NativeArray<int> start1, NativeArray<int> end1,
                        NativeArray<int> start2, NativeArray<int> end2,
                        out NativeArray<int> r1, out NativeArray<int> r2)
        {
            // Create lists to store the merged ranges
            List<int> starts = new List<int>();
            List<int> ends = new List<int>();

            // Initialize pointers for both the input range sets
            int i = 0, j = 0;

            // Traverse both range sets
            while (i < start1.Length && j < start2.Length)
            {
                // If current range from the first set is smaller or equal
                if (start1[i] <= start2[j])
                {
                    // If there's overlap with the last range in the merged set
                    if (starts.Count > 0 && start1[i] <= ends[ends.Count - 1])
                    {
                        // Extend the end of the last range in the merged set
                        ends[ends.Count - 1] = Math.Max(ends[ends.Count - 1], end1[i]);
                    }
                    else
                    {
                        // Otherwise, add the current range to the merged set
                        starts.Add(start1[i]);
                        ends.Add(end1[i]);
                    }

                    // Move to the next range in the first set
                    i++;
                }
                else
                {
                    // If current range from the second set is smaller
                    // And there's overlap with the last range in the merged set
                    if (starts.Count > 0 && start2[j] <= ends[ends.Count - 1])
                    {
                        // Extend the end of the last range in the merged set
                        ends[ends.Count - 1] = Math.Max(ends[ends.Count - 1], end2[j]);
                    }
                    else
                    {
                        // Otherwise, add the current range to the merged set
                        starts.Add(start2[j]);
                        ends.Add(end2[j]);
                    }

                    // Move to the next range in the second set
                    j++;
                }
            }

            // If there are remaining ranges in the first set
            while (i < start1.Length)
            {
                // If there's overlap with the last range in the merged set
                if (starts.Count > 0 && start1[i] <= ends[ends.Count - 1])
                {
                    // Extend the end of the last range in the merged set
                    ends[ends.Count - 1] = Math.Max(ends[ends.Count - 1], end1[i]);
                }
                else
                {
                    // Otherwise, add the current range to the merged set
                    starts.Add(start1[i]);
                    ends.Add(end1[i]);
                }

                // Move to the next range in the first set
                i++;
            }

            // If there are remaining ranges in the second set
            while (j < start2.Length)
            {
                // If there's overlap with the last range in the merged set
                if (starts.Count > 0 && start2[j] <= ends[ends.Count - 1])
                {
                    // Extend the end of the last range in the merged set
                    ends[ends.Count - 1] = Math.Max(ends[ends.Count - 1], end2[j]);
                }
                else
                {
                    // Otherwise, add the current range to the merged set
                    starts.Add(start2[j]);
                    ends.Add(end2[j]);
                }

                // Move to the next range in the second set
                j++;
            }

            // Convert the lists to NativeArray and set as output
            r1 = new NativeArray<int>(starts.ToArray(), Allocator.Temp);
            r2 = new NativeArray<int>(ends.ToArray(), Allocator.Temp);
        }
        private void And(NativeArray<int> start1, NativeArray<int> end1,
                         NativeArray<int> start2, NativeArray<int> end2,
                         out NativeArray<int> r1, out NativeArray<int> r2)
        {
            // Create lists to store the intersected ranges
            List<int> starts = new List<int>();
            List<int> ends = new List<int>();

            // Initialize pointers for both the input range sets
            int i = 0, j = 0;

            // Traverse both range sets
            while (i < start1.Length && j < start2.Length)
            {
                // Find overlap
                int overlapStart = Math.Max(start1[i], start2[j]);
                int overlapEnd = Math.Min(end1[i], end2[j]);

                // If overlap exists, add it to result
                if (overlapStart <= overlapEnd)
                {
                    starts.Add(overlapStart);
                    ends.Add(overlapEnd);
                }

                // Move to the next range in the set which ends earlier
                if (end1[i] < end2[j])
                    i++;
                else
                    j++;
            }

            // Convert the lists to NativeArray and set as output
            r1 = new NativeArray<int>(starts.ToArray(), Allocator.Temp);
            r2 = new NativeArray<int>(ends.ToArray(), Allocator.Temp);
        }
        private void Diff(NativeArray<int> start1, NativeArray<int> end1,
                          NativeArray<int> start2, NativeArray<int> end2,
                          out NativeArray<int> r1, out NativeArray<int> r2)
        {
            List<int> starts = new List<int>();
            List<int> ends = new List<int>();
            int i = 0, j = 0;
            NativeArray<int> start1Copy = new NativeArray<int>(start1, Allocator.Temp);

            while (i < start1Copy.Length && j < start2.Length)
            {
                if (end1[i] < start2[j])
                {
                    // If the current range from first set does not overlap with the current range from second set
                    starts.Add(start1Copy[i]);
                    ends.Add(end1[i]);
                    i++;
                }
                else if (start1Copy[i] > end2[j])
                {
                    // If the current range from second set does not overlap with the current range from first set
                    j++;
                }
                else
                {
                    // Overlap exists
                    if (start1Copy[i] < start2[j])
                    {
                        // If the current range from the first set starts before the current range from the second set
                        starts.Add(start1Copy[i]);
                        ends.Add(start2[j] - 1);
                    }
                    if (end1[i] > end2[j])
                    {
                        // If the current range from the first set ends after the current range from the second set
                        start1Copy[i] = end2[j] + 1;
                        j++;
                    }
                    else
                    {
                        i++;
                    }
                }
            }

            // Add any remaining ranges from the first set
            while (i < start1Copy.Length)
            {
                starts.Add(start1Copy[i]);
                ends.Add(end1[i]);
                i++;
            }

            start1Copy.Dispose();  // Dispose the copy array here

            r1 = new NativeArray<int>(starts.ToArray(), Allocator.Temp);
            r2 = new NativeArray<int>(ends.ToArray(), Allocator.Temp);
        }

        public static QueryTag Tag(string tag)
        {
            return new QueryTag(tag);
        }

        public static QueryTag Or(QueryTag t1, QueryTag t2)
        {
            return Op(t1, t2, Operation.Or);
        }
        public static QueryTag Or(QueryTag t1, string t2)
        {
            return Op(t1, t2, Operation.Or);
        }
        public static QueryTag Or(string t1, QueryTag t2)
        {
            return Op(t1, t2, Operation.Or);
        }
        public static QueryTag Or(string t1, string t2)
        {
            return Op(t1, t2, Operation.Or);
        }

        public static QueryTag And(QueryTag t1, QueryTag t2)
        {
            return Op(t1, t2, Operation.And);
        }
        public static QueryTag And(QueryTag t1, string t2)
        {
            return Op(t1, t2, Operation.And);
        }
        public static QueryTag And(string t1, QueryTag t2)
        {
            return Op(t1, t2, Operation.And);
        }
        public static QueryTag And(string t1, string t2)
        {
            return Op(t1, t2, Operation.And);
        }

        public static QueryTag Diff(QueryTag t1, QueryTag t2)
        {
            return Op(t1, t2, Operation.Diff);
        }
        public static QueryTag Diff(QueryTag t1, string t2)
        {
            return Op(t1, t2, Operation.Diff);
        }
        public static QueryTag Diff(string t1, QueryTag t2)
        {
            return Op(t1, t2, Operation.Diff);
        }
        public static QueryTag Diff(string t1, string t2)
        {
            return Op(t1, t2, Operation.Diff);
        }

        private static QueryTag Op(QueryTag t1, QueryTag t2, Operation op)
        {
            QueryTag r = new QueryTag
            {
                Tags = new string[t1.Tags.Length + t2.Tags.Length],
                Graph = new int[t1.Graph.Length + t2.Graph.Length + 1]
            };
            // Tags
            int t = 0;
            for (int i = 0; i < t1.Tags.Length; ++i, ++t)
            {
                r.Tags[t] = t1.Tags[i];
            }
            for (int i = 0; i < t2.Tags.Length; ++i, ++t)
            {
                r.Tags[t] = t2.Tags[i];
            }
            // Graph
            r.Graph[0] = (int)op;
            int g = 1;
            for (int i = 0; i < t1.Graph.Length; ++i, ++g)
            {
                r.Graph[g] = t1.Graph[i];
            }
            for (int i = 0; i < t2.Graph.Length; ++i, ++g)
            {
                int v = t2.Graph[i];
                r.Graph[g] = v < 0 ? v : v + t1.Tags.Length;
            }
            return r;
        }
        private static QueryTag Op(QueryTag t1, string t2, Operation op)
        {
            QueryTag r = new QueryTag
            {
                Tags = new string[t1.Tags.Length + 1],
                Graph = new int[t1.Graph.Length + 1]
            };
            // Tags
            int t = 0;
            for (int i = 0; i < t1.Tags.Length; ++i, ++t)
            {
                r.Tags[t] = t1.Tags[i];
            }
            r.Tags[t] = t2;
            // Graph
            r.Graph[0] = (int)op;
            int g = 1;
            for (int i = 0; i < t1.Graph.Length; ++i, ++g)
            {
                r.Graph[g] = t1.Graph[i];
            }
            r.Graph[g] = r.Tags.Length - 1;
            return r;
        }
        private static QueryTag Op(string t1, QueryTag t2, Operation op)
        {
            QueryTag r = new QueryTag
            {
                Tags = new string[t2.Tags.Length + 1],
                Graph = new int[t2.Graph.Length + 1]
            };
            // Tags
            r.Tags[0] = t1;
            int t = 1;
            for (int i = 0; i < t2.Tags.Length; ++i, ++t)
            {
                r.Tags[t] = t2.Tags[i];
            }
            // Graph
            r.Graph[0] = (int)op;
            r.Graph[1] = 0;
            int g = 2;
            for (int i = 0; i < t2.Graph.Length; ++i, ++g)
            {
                int v = t2.Graph[i];
                r.Graph[g] = v < 0 ? v : v + 1;
            }
            return r;
        }
        private static QueryTag Op(string t1, string t2, Operation op)
        {
            QueryTag r = new QueryTag
            {
                Tags = new string[2],
                Graph = new int[3]
            };
            // Tags
            r.Tags[0] = t1;
            r.Tags[1] = t2;
            // Graph
            r.Graph[0] = (int)op;
            r.Graph[1] = 0;
            r.Graph[2] = 1;
            return r;
        }

        /// <summary>
        /// Returns a QueryTag given a string expression such as ((walk | run) & male) - sad
        /// | is the OR operator
        /// & is the AND operator
        /// - is the DIFF operator
        /// () can be used to prioritize operations
        /// Returns true if it was correctly parsed, false otherwise
        /// </summary>
        public static bool Parse(string expression, out QueryTag queryTag)
        {
            if (!ExtractTokens(expression, out List<string> tokens))
            {
                queryTag = null;
                return false;
            }
            return ParseTokens(tokens, 0, out queryTag);
        }

        private static bool ExtractTokens(string expression, out List<string> tokens)
        {
            tokens = new List<string>();
            StringBuilder sb = new StringBuilder();

            for (int i = 0; i < expression.Length; ++i)
            {
                char c = expression[i];
                switch (c)
                {
                    case ' ':
                        if (sb.Length == 0)
                            continue;

                        sb.Append(c);
                        break;

                    case '(':
                    case ')':
                        if (sb.Length > 0)
                        {
                            tokens.Add(sb.ToString());
                            sb.Clear();
                        }

                        tokens.Add(c.ToString(CultureInfo.InvariantCulture));
                        break;

                    case '&':
                    case '|':
                    case '-':
                        if (sb.Length > 0)
                        {
                            tokens.Add(sb.ToString().Trim());
                            sb.Clear();
                        }
                        tokens.Add(c.ToString());
                        break;

                    default:
                        sb.Append(c);
                        break;
                }
            }

            if (sb.Length > 0)
            {
                tokens.Add(sb.ToString().Trim());
            }

            return true;
        }

        // tokens are provided in the follow way: "(male & (run | walk)) - sad"
        // this example should be converted to:
        //      graph = [(int)Operation.Diff, (int)Operation.And, 0, (int)Operation.Or, 1, 2, 3]
        //      tags  = [male, run, walk, sad]
        private static bool ParseTokens(List<string> tokens, int left, out QueryTag queryTag)
        {
            return ParseOperations(tokens, ref left, out queryTag);
        }
        private static bool ParseOperations(List<string> tokens, ref int left, out QueryTag queryTag)
        {
            // Parse left hand side
            bool res = ParseLeaf(tokens, ref left, out QueryTag leftQueryTag);
            if (!res)
            {
                if (leftQueryTag != null) leftQueryTag.Dispose();
                queryTag = null;
                return false;
            }

            while (true)
            {
                if (left >= tokens.Count)
                {
                    // No operation
                    queryTag = leftQueryTag;
                    return true;
                }

                Operation op;

                // Operator
                if (tokens[left] == "|")
                {
                    op = Operation.Or;
                }
                else if (tokens[left] == "&")
                {
                    op = Operation.And;
                }
                else if (tokens[left] == "-")
                {
                    op = Operation.Diff;
                }
                else
                {
                    // op not found
                    queryTag = leftQueryTag;
                    return true;
                }

                left += 1; // skip operator

                // Parse right hand side
                res = ParseLeaf(tokens, ref left, out QueryTag rightQueryTag);
                if (!res)
                {
                    if (leftQueryTag != null) leftQueryTag.Dispose();
                    if (rightQueryTag != null) rightQueryTag.Dispose();
                    queryTag = null;
                    return false;
                }

                // Create new QueryTag based on op
                QueryTag tmpLeftQueryTag = QueryTag.Op(leftQueryTag, rightQueryTag, op);
                if (leftQueryTag != null) leftQueryTag.Dispose();
                if (rightQueryTag != null) rightQueryTag.Dispose();
                leftQueryTag = tmpLeftQueryTag;
            }
        }
        private static bool ParseLeaf(List<string> tokens, ref int left, out QueryTag queryTag)
        {
            if (left >= tokens.Count)
            {
                queryTag = null;
                return false;
            }

            // Is it a tag?
            if (tokens[left] != "|" && tokens[left] != "&" && tokens[left] != "-" && tokens[left] != "(" && tokens[left] != ")")
            {
                queryTag = new QueryTag(tokens[left]);
                left += 1;
                return true;
            }

            // Is it a Parenthesis?
            if (tokens[left] == "(")
            {
                left += 1; // skip "("

                // Parse a top-level expression
                bool res = ParseOperations(tokens, ref left, out queryTag);

                // Check and skip ")"
                if (!res || left >= tokens.Count || tokens[left] != ")")
                {
                    if (queryTag != null) queryTag.Dispose();
                    // Missing close parenthesis
                    return false;
                }
                left += 1;

                return true;
            }

            queryTag = null;
            return false;
        }
    }
}
