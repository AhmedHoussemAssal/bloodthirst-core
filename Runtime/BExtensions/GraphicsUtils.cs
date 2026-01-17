using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Pool;
using UnityEngine.Rendering;


#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Bloodthirst.Core.Utils
{
    public struct ConvexShape
    {
        public Vector2[] points;
        public Vector2 center;
    }
    public struct TriangleIndicies
    {
        public int v1;
        public int v2;
        public int v3;
    }

    public struct EdgeIndicies
    {
        public int e1;
        public int e2;
    }

    public sealed class TextureIsland
    {
        public Vector2Int[] pixelCoordsArray;
        public RectInt bounds;
        public Vector2Int textureSize;
    }


    public sealed class RasterizedIsland
    {
        public struct Segment
        {
            public int x;
            public int width;
        }

        /// <summary>
        /// <para>Contains the populated rows of the Island in terms of filled segments in rows</para>
        /// <para>The rows are expressed in the local space of the Rect</para>
        /// <para>Example : Row.Segement[0].Firs() correponds to the first segment in the bottom row</para>
        /// <para>Example : Row.Segement[bounds.height].Last() correponds to the last segment in the top row</para>
        /// <para>NOTE : Row.Segement's values are in int increments</para>
        /// <para>Row.x is always between [0 , bounds.width]</para>
        /// <para>Row.width is always between [0 , bounds.width]</para>
        /// <para></para>
        /// </summary>
        public Segment[][] segmentsInRow;
        public RectInt bounds;
        public Vector2Int textureSize;
    }

    public sealed class ScaledRasterizedIsland
    {
        public struct Segment
        {
            public float x;
            public float width;
        }

        public Rect bounds;
        public Segment[][] segmentsPerRow;
    }

    /// <summary>
    /// an extension class containing helper methods for types / reflection
    /// </summary>
    public static class GraphicsUtils
    {
        public static ScaledRasterizedIsland ScaleTextureIsland(RasterizedIsland island, Rect targetSpace)
        {
            Vector2Int textureSize = island.textureSize;

            Rect scaledRect = default;
            scaledRect.x = MathUtils.Remap(island.bounds.x, 0, textureSize.x, targetSpace.xMin, targetSpace.xMax);
            scaledRect.y = MathUtils.Remap(island.bounds.y, 0, textureSize.y, targetSpace.yMin, targetSpace.yMax);
            scaledRect.width = MathUtils.Remap(island.bounds.width, 0, textureSize.x, 0 , targetSpace.width);
            scaledRect.height = MathUtils.Remap(island.bounds.height, 0, textureSize.y, 0, targetSpace.height);

            ScaledRasterizedIsland.Segment[][] scaledRows = new ScaledRasterizedIsland.Segment[island.segmentsInRow.Length][];

            for (int i = 0; i < island.segmentsInRow.Length; i++)
            {
                RasterizedIsland.Segment[] segments = island.segmentsInRow[i];
                ScaledRasterizedIsland.Segment[] scaledSegements = new ScaledRasterizedIsland.Segment[segments.Length];

                for (int segIdx = 0; segIdx < segments.Length; ++segIdx)
                {
                    RasterizedIsland.Segment currSeg = segments[segIdx];
                    ScaledRasterizedIsland.Segment scaledSeg = new ScaledRasterizedIsland.Segment()
                    {
                        x = MathUtils.Remap(currSeg.x, 0, island.bounds.width, scaledRect.xMin, scaledRect.xMax),
                        width = MathUtils.Remap(currSeg.width, 0, island.bounds.width, 0, scaledRect.width),
                    };

                    scaledSegements[segIdx] = scaledSeg;
                }

                scaledRows[i] = scaledSegements;
            }

            ScaledRasterizedIsland scaled = new ScaledRasterizedIsland()
            {
                bounds = scaledRect,
                segmentsPerRow = scaledRows
            };

            return scaled;
        }

        public static RasterizedIsland RasterizeTextureIsland(TextureIsland island)
        {
            RectInt rectf = new RectInt(island.bounds.x, island.bounds.y, island.bounds.width, island.bounds.height);

            RasterizedIsland rasterized = new RasterizedIsland()
            {
                textureSize = island.textureSize,
                bounds = rectf,
                segmentsInRow = new RasterizedIsland.Segment[island.bounds.height + 1][]
            };

            using (HashSetPool<Vector2Int>.Get(out var pixels))
            using (HashSetPool<RasterizedIsland.Segment>.Get(out HashSet<RasterizedIsland.Segment> rows))
            {
                foreach (Vector2Int p in island.pixelCoordsArray)
                {
                    pixels.Add(p);
                }

                for (int y = island.bounds.yMin; y <= island.bounds.yMax; ++y)
                {
                    rows.Clear();
                    int x = island.bounds.xMin;

                    while (x <= island.bounds.xMax)
                    {
                        Vector2Int? start = null;

                        while (x <= island.bounds.xMax)
                        {
                            Vector2Int p = new Vector2Int(x, y);

                            if (!pixels.Contains(p))
                            {
                                x++;
                                continue;
                            }

                            start = p;
                            break;
                        }

                        if (!start.HasValue) { break; }

                        Vector2Int? end = null;

                        while (x <= island.bounds.xMax)
                        {
                            Vector2Int p = new Vector2Int(x, y);

                            if (!pixels.Contains(p))
                            {
                                break;
                            }

                            x++;
                        }

                        end = new Vector2Int(x, y);

                        var seg = new RasterizedIsland.Segment()
                        {
                            x = start.Value.x - island.bounds.xMin,
                            width = end.Value.x - start.Value.x
                        };

                        rows.Add(seg);
                    }

                    int idx = y - island.bounds.yMin;
                    rasterized.segmentsInRow[idx] = rows.ToArray();
                }

            }
            return rasterized;
        }

        public static void GetIslandsFromTexture(Texture2D texture, List<TextureIsland> islands, float minMaskValue)
        {
            Vector2Int textureSize = new Vector2Int(texture.width, texture.height);

            Stack<Vector2Int> currentStack = new Stack<Vector2Int>();


            Assert.IsTrue(minMaskValue > 0);

            using (ListPool<Color>.Get(out List<Color> texColors))
            using (ListPool<Vector2Int>.Get(out List<Vector2Int> freePoints))
            using (HashSetPool<Vector2Int>.Get(out HashSet<Vector2Int> maskPoints))
            using (HashSetPool<Vector2Int>.Get(out HashSet<Vector2Int> usedPoints))
            {
                texColors.Capacity = textureSize.x * textureSize.y;
                freePoints.Capacity = textureSize.x * textureSize.y;

                texColors.AddRange(texture.GetPixels());

                for (int x = 0; x < textureSize.x; x++)
                {
                    for (int y = 0; y < textureSize.y; y++)
                    {
                        freePoints.Add(new Vector2Int(x, y));
                    }
                }

                while(freePoints.Count != 0)
                {
                    // keep iterating over the points
                    // until we find the first one that has a heatmap value
                    {
                        Vector2Int currPoint = freePoints[freePoints.Count - 1];
                        freePoints.RemoveAt(freePoints.Count - 1);

                        if (!usedPoints.Add(currPoint)) { continue; }

                        int idx = currPoint.x + (textureSize.x * currPoint.y);
                        Color val = texColors[idx];

                        if (val.r < minMaskValue) { continue; }

                        maskPoints.Clear();
                        currentStack.Clear();
                        currentStack.Push(currPoint);
                    }

                    // we keep iterating and exploring the neighbors that have a value
                    // until we run out
                    while(currentStack.Count != 0)
                    {
                        Vector2Int currPoint = currentStack.Pop();

                        int idx = currPoint.x + (textureSize.x * currPoint.y);
                        Color val = texColors[idx];

                        if (val.r < minMaskValue) { continue; }

                        maskPoints.Add(currPoint);

                        Vector2Int t = currPoint + Vector2Int.up;
                        Vector2Int b = currPoint + Vector2Int.down;
                        Vector2Int l = currPoint + Vector2Int.left;
                        Vector2Int r = currPoint + Vector2Int.right;

                        bool tOOB = t.x < 0 || t.x > (textureSize.x - 1) || t.y < 0 || t.y > (textureSize.y - 1);
                        bool bOOB = b.x < 0 || b.x > (textureSize.x - 1) || b.y < 0 || b.y > (textureSize.y - 1);
                        bool lOOB = l.x < 0 || l.x > (textureSize.x - 1) || l.y < 0 || l.y > (textureSize.y - 1);
                        bool rOOB = r.x < 0 || r.x > (textureSize.x - 1) || r.y < 0 || r.y > (textureSize.y - 1);

                        if (!tOOB && usedPoints.Add(t)) { currentStack.Push(t); }
                        if (!bOOB && usedPoints.Add(b)) { currentStack.Push(b); }
                        if (!rOOB && usedPoints.Add(r)) { currentStack.Push(r); }
                        if (!lOOB && usedPoints.Add(l)) { currentStack.Push(l); }
                    }

                    if(maskPoints.Count == 0) { continue; }

                    Vector2Int[] maskPixels = maskPoints.ToArray();
                    int minX = maskPixels.Min(p => p.x);
                    int maxX = maskPixels.Max(p => p.x);
                    int minY = maskPixels.Min(p => p.y);
                    int maxY = maskPixels.Max(p => p.y);

                    TextureIsland island = new TextureIsland()
                    {
                        pixelCoordsArray = maskPixels,
                        bounds = new RectInt(minX, minY, maxX - minX, maxY - minY),
                        textureSize = textureSize
                    };

                    islands.Add(island);
                }
            }
        }

        public static TriangleIndicies AlignTriangleNormal(IReadOnlyList<Vector2> points, TriangleIndicies tri, Vector3 up)
        {
            Vector2 v1 = points[tri.v1];
            Vector2 v2 = points[tri.v2];
            Vector2 v3 = points[tri.v3];

            Vector3 normal = Vector3.Cross(v3 - v1, v2 - v1);

            TriangleIndicies corretTri = tri;

            if (Vector3.Dot(normal, up) < 0)
            {
                corretTri.v1 = tri.v2;
                corretTri.v2 = tri.v1;
            }

            return corretTri;
        }

        public static Rect CreateRect(Vector2 center, Vector2 size)
        {
            return new Rect(center - (size * 0.5f), size);
        }

        private struct TriangleOutlinePairs
        {
            public TriangleIndicies tri;
            public EdgeIndicies edge;
        }

        /// <summary>
        /// TODO : remake this in a way that accounts for a single tri being associated with MORE than one edge
        /// </summary>
        /// <param name="points"></param>
        /// <param name="triangles"></param>
        /// <param name="outlineEdges"></param>
        /// <param name="outlineMesh"></param>
        /// <param name="width"></param>
        public static void GenerateOutlineMesh(IReadOnlyList<Vector2> points, IReadOnlyList<TriangleIndicies> triangles, IReadOnlyList<EdgeIndicies> outlineEdges, float width, List<Vector2> outlineVerts, List<int> outlineIndicies)
        {
            using (HashSetPool<Vector2>.Get(out HashSet<Vector2> dupCheck))
            {
                foreach (Vector2 p in points)
                {
                    dupCheck.Add(p);
                }

                Assert.IsTrue(dupCheck.Count == points.Count, "Duplicate points detected");
            }

            using (ListPool<TriangleOutlinePairs>.Get(out List<TriangleOutlinePairs> triangleOutlines))
            using (ListPool<int>.Get(out List<int> indiciesTmp))
            using (ListPool<EdgeIndicies>.Get(out List<EdgeIndicies> outlineEdgesCopy))
            using (ListPool<int>.Get(out var edgeFilter))
            using (HashSetPool<int>.Get(out var edgeIndicies))
            using (DictionaryPool<Vector2, int>.Get(out var vertToIndexLookup))
            using (DictionaryPool<EdgeIndicies, Vector2>.Get(out var edgeToNormal))
            using (DictionaryPool<int, Vector2>.Get(out var indexToAvgNormal))
            using (DictionaryPool<EdgeIndicies, TriangleIndicies>.Get(out var edgeToTriangle))
            using (ListPool<TriangleIndicies>.Get(out List<TriangleIndicies> outlineMeshTris))
            using (ListPool<Vector2>.Get(out List<Vector2> outlineMeshVerticies2D))
            using (ListPool<int>.Get(out List<int> outlineMeshIndicies))
            {
                outlineEdgesCopy.AddRange(outlineEdges);

                foreach (TriangleIndicies t in triangles)
                {
                    indiciesTmp.Clear();
                    indiciesTmp.Add(t.v1);
                    indiciesTmp.Add(t.v2);
                    indiciesTmp.Add(t.v3);

                    for (int i = outlineEdgesCopy.Count - 1; i >= 0; i--)
                    {
                        EdgeIndicies e = outlineEdgesCopy[i];

                        if (indiciesTmp.Contains(e.e1) && indiciesTmp.Contains(e.e2))
                        {
                            edgeToTriangle.Add(e, t);

                            triangleOutlines.Add(new TriangleOutlinePairs()
                            {
                                edge = e,
                                tri = t
                            });

                            outlineEdgesCopy.RemoveAt(i);
                        }
                    }
                }

                foreach (EdgeIndicies e in outlineEdges)
                {
                    edgeFilter.Clear();
                    edgeFilter.Add(e.e1);
                    edgeFilter.Add(e.e2);

                    int freeVert = -1;

                    TriangleIndicies t = edgeToTriangle[e];

                    if (!edgeFilter.Contains(t.v1))
                    {
                        freeVert = t.v1;
                    }

                    if (!edgeFilter.Contains(t.v2))
                    {
                        freeVert = t.v2;
                    }

                    if (!edgeFilter.Contains(t.v3))
                    {
                        freeVert = t.v3;
                    }

                    Assert.IsTrue(freeVert != -1);

                    int v1 = edgeFilter[0];
                    int v2 = edgeFilter[1];

                    edgeIndicies.Add(v1);
                    edgeIndicies.Add(v2);

                    Vector2 freeVertPos = points[freeVert];
                    Vector2 v1Pos = points[v1];
                    Vector2 v2Pos = points[v2];

                    Vector3 normal = Vector3.Cross(v2Pos - v1Pos, freeVertPos - v1Pos).normalized;
                    Vector2 outlineVec = Quaternion.AngleAxis(90, normal) * (v2Pos - v1Pos);
                    outlineVec.Normalize();

                    if (Vector2.Dot(outlineVec, freeVertPos - v1Pos) < 0)
                    {
                        outlineVec *= -1;
                    }

#if UNITY_EDITOR
                    if (!Application.isPlaying)
                    {
                        Handles.color = Color.white;
                        Handles.ArrowHandleCap(-1, Vector3.Lerp(v1Pos, v2Pos, 0.5f), Quaternion.LookRotation(outlineVec, Vector3.forward), 1, EventType.Repaint);
                        Handles.color = Color.black;
                        Handles.ArrowHandleCap(-1, Vector3.Lerp(v1Pos, v2Pos, 0.5f), Quaternion.LookRotation(outlineVec, Vector3.forward), width, EventType.Repaint);
                    }
#endif
                    edgeToNormal.Add(e, outlineVec);
                }

                foreach (int i in edgeIndicies)
                {
                    Vector2 sum = default;
                    int cnt = 0;

                    foreach (KeyValuePair<EdgeIndicies, Vector2> e in edgeToNormal)
                    {
                        if (e.Key.e1 == i || e.Key.e2 == i)
                        {
                            sum += e.Value;
                            cnt++;
                        }
                    }

                    Assert.IsTrue(cnt == 2);

                    Vector2 avgNormal = (sum / cnt).normalized;
                    indexToAvgNormal.Add(i, avgNormal);


#if UNITY_EDITOR
                    if (!Application.isPlaying)
                    {
                        Vector2 p = points[i];
                        Handles.color = Color.cyan;
                        Handles.ArrowHandleCap(-1, p, Quaternion.LookRotation(avgNormal, Vector3.forward), 1, EventType.Repaint);
                    }
#endif
                }

                float outlineWith = width;

                foreach (var (e, n) in edgeToNormal)
                {
                    EdgeIndicies currEdge = e;

                    var e1AvgNormal = indexToAvgNormal[currEdge.e1];
                    var e2AvgNormal = indexToAvgNormal[currEdge.e2];

                    Vector2 v1Pos = points[e.e1];
                    Vector2 v2Pos = points[e.e2];

                    var proj1 = Vector2.Dot(n, e1AvgNormal);
                    var proj2 = Vector2.Dot(n, e2AvgNormal);

                    Vector2 v3Pos = v1Pos + (e1AvgNormal / proj1 * outlineWith);
                    Vector2 v4Pos = v2Pos + (e2AvgNormal / proj2 * outlineWith);

#if UNITY_EDITOR
                    if (!Application.isPlaying)
                    {
                        Handles.color = Color.yellow;
                        Handles.ArrowHandleCap(-1, v1Pos, Quaternion.LookRotation(e1AvgNormal, Vector3.forward), proj1, EventType.Repaint);
                        Handles.ArrowHandleCap(-1, v2Pos, Quaternion.LookRotation(e2AvgNormal, Vector3.forward), proj2, EventType.Repaint);
                    }
#endif
                    if (!vertToIndexLookup.TryGetValue(v1Pos, out var v1Idx)) { v1Idx = vertToIndexLookup.Count; vertToIndexLookup.Add(v1Pos, v1Idx); }
                    if (!vertToIndexLookup.TryGetValue(v2Pos, out var v2Idx)) { v2Idx = vertToIndexLookup.Count; vertToIndexLookup.Add(v2Pos, v2Idx); }
                    if (!vertToIndexLookup.TryGetValue(v3Pos, out var v3Idx)) { v3Idx = vertToIndexLookup.Count; vertToIndexLookup.Add(v3Pos, v3Idx); }
                    if (!vertToIndexLookup.TryGetValue(v4Pos, out var v4Idx)) { v4Idx = vertToIndexLookup.Count; vertToIndexLookup.Add(v4Pos, v4Idx); }

                    TriangleIndicies t1 = new TriangleIndicies() { v1 = v1Idx, v2 = v3Idx, v3 = v4Idx };
                    TriangleIndicies t2 = new TriangleIndicies() { v1 = v4Idx, v2 = v2Idx, v3 = v1Idx };

                    outlineMeshTris.Add(t1);
                    outlineMeshTris.Add(t2);
                }

                // add point to the outline vertex list
                outlineMeshVerticies2D.ExpandeSize(vertToIndexLookup.Count);
                foreach (KeyValuePair<Vector2, int> kv in vertToIndexLookup)
                {
                    outlineMeshVerticies2D[kv.Value] = kv.Key;
                }

                for (int i = 0; i < outlineMeshTris.Count; i++)
                {
                    TriangleIndicies t = outlineMeshTris[i];
                    t = GraphicsUtils.AlignTriangleNormal(outlineMeshVerticies2D, t, Vector3.forward);
                    outlineMeshTris[i] = t;

                    outlineMeshIndicies.Add(t.v1);
                    outlineMeshIndicies.Add(t.v2);
                    outlineMeshIndicies.Add(t.v3);
                }

                outlineIndicies.AddRange(outlineMeshIndicies);
                outlineVerts.AddRange(outlineMeshVerticies2D);
            }
        }

        public static void GetMeshOutline(IReadOnlyList<TriangleIndicies> triangles, List<EdgeIndicies> outlineEdges)
        {
            using (DictionaryPool<EdgeIndicies, int>.Get(out Dictionary<EdgeIndicies, int> edges))
            {
                foreach (TriangleIndicies t in triangles)
                {
                    EdgeIndicies e1 = new EdgeIndicies() { e1 = Mathf.Min(t.v1, t.v2), e2 = Mathf.Max(t.v1, t.v2) };
                    EdgeIndicies e2 = new EdgeIndicies() { e1 = Mathf.Min(t.v2, t.v3), e2 = Mathf.Max(t.v2, t.v3) };
                    EdgeIndicies e3 = new EdgeIndicies() { e1 = Mathf.Min(t.v3, t.v1), e2 = Mathf.Max(t.v3, t.v1) };

                    if (!edges.TryGetValue(e1, out var cnt1)) { edges.Add(e1, 0); }
                    if (!edges.TryGetValue(e2, out var cnt2)) { edges.Add(e2, 0); }
                    if (!edges.TryGetValue(e3, out var cnt3)) { edges.Add(e3, 0); }

                    edges[e1] = cnt1 + 1;
                    edges[e2] = cnt2 + 1;
                    edges[e3] = cnt3 + 1;
                }

                foreach (KeyValuePair<EdgeIndicies, int> e in edges)
                {
                    if (e.Value != 1) { continue; }

                    outlineEdges.Add(e.Key);
                }
            }
        }

        public static ConvexShape PointsToConvexShapes(IReadOnlyList<Vector2> points)
        {
            using (ListPool<Vector2>.Get(out var orderedPoints))
            using (ListPool<Vector2>.Get(out var convexPoints))
            {
                Vector2 middlePoint = default;

                foreach (Vector2 p in points)
                {
                    middlePoint += p;
                }

                middlePoint /= points.Count;

                orderedPoints.AddRange(points.OrderBy(a =>
                {
                    Vector2 vec = a - middlePoint;
                    float angle = Mathf.Atan2(vec.y, vec.x) * Mathf.Rad2Deg;
                    float remapped = angle >= 0 ? angle : 360 + angle;
                    return remapped;
                }));

                float correctSign = 1;

                for (int i = 0; i < orderedPoints.Count; ++i)
                {
                    Vector2 curr = orderedPoints[i];
                    Vector2 prev = orderedPoints[(i - 1 + orderedPoints.Count) % orderedPoints.Count];
                    Vector2 next = orderedPoints[(i + 1 + orderedPoints.Count) % orderedPoints.Count];

                    float angle = Vector2.SignedAngle(curr - prev, next - curr);
                    float sign = angle < 0 ? -1 : 1;

                    if (sign == correctSign)
                    {
                        convexPoints.Add(curr);
                        continue;
                    }
                }

                return new ConvexShape() { points = convexPoints.ToArray(), center = middlePoint };
            }
        }

        public static bool FindIntersections(Vector2 center1, float radius1, Vector2 center2, float radius2,
                                         out Vector2 intersection1, out Vector2 intersection2)
        {
            intersection1 = Vector2.zero;
            intersection2 = Vector2.zero;

            float d = Vector2.Distance(center1, center2);

            // No solution: circles are too far apart or one is contained inside the other
            if (d > radius1 + radius2 || d < Mathf.Abs(radius1 - radius2))
            {
                return false;
            }

            // Circles are coincident (infinite solutions)
            if (d == 0 && radius1 == radius2)
            {
                return false;
            }

            // Distance from center1 to the midpoint between intersections
            float a = (radius1 * radius1 - radius2 * radius2 + d * d) / (2 * d);

            // Height from midpoint to intersection points
            float h = Mathf.Sqrt(radius1 * radius1 - a * a);

            // Midpoint between intersections
            Vector2 p = center1 + a * (center2 - center1) / d;

            // Intersection points
            intersection1 = new Vector2(
                p.x + h * (center2.y - center1.y) / d,
                p.y - h * (center2.x - center1.x) / d
            );

            intersection2 = new Vector2(
                p.x - h * (center2.y - center1.y) / d,
                p.y + h * (center2.x - center1.x) / d
            );

            return true;
        }

        public static bool TwoCircleIntersection(Vector3 a, Vector3 b, int resolution, out Vector2 inter1, out Vector2 inter2, List<Vector2> points)
        {
            Vector2 aCenter = new Vector2(a.x, a.y);
            float aRad = a.z;

            Vector2 bCenter = new Vector2(b.x, b.y);
            float bRad = b.z;

            Vector2 fromAtoB = bCenter - aCenter;

            bool exists = FindIntersections(aCenter, aRad, bCenter, bRad, out inter1, out inter2);

            if (!exists)
            {
                return false;
            }

            Vector2 midPoint = Vector2.Lerp(inter1, inter2, 0.5f);

            float aAnglePerSlice = default;
            float bAnglePerSlice = default;

            // B
            {
                float bFullAngle = Vector2.SignedAngle(inter1 - bCenter, aCenter - bCenter) * 2;
                bAnglePerSlice = bFullAngle / resolution;
            }

            // A
            {
                float aFullAngle = Vector2.SignedAngle(inter1 - aCenter, bCenter - aCenter) * 2;
                aAnglePerSlice = aFullAngle / resolution;
            }

            points.Add(midPoint);

            for (int i = 1; i < resolution; i++)
            {
                Vector2 p1Inter = aCenter + (Vector2)(Quaternion.AngleAxis(aAnglePerSlice * i, Vector3.forward) * (inter1 - aCenter));
                points.Add(p1Inter);
            }

            for (int i = 1; i < resolution; i++)
            {
                Vector2 p2Inter = bCenter + (Vector2)(Quaternion.AngleAxis(bAnglePerSlice * i, Vector3.forward) * (inter1 - bCenter));
                points.Add(p2Inter);
            }

            return true;
        }

        public static Rect ComputeScreenSize(GameObject gameObject, Camera cam, Vector3 worldPos)
        {
            Bounds bounds = GraphicsUtils.GetBounds(gameObject);

            using (ListPool<Vector3>.Get(out List<Vector3> cornersWorld))
            using (ListPool<Vector3>.Get(out List<Vector3> cornersScreen))
            {
                CollectionsUtils.ExpandeSize(cornersWorld, 8);
                GraphicsUtils.GetCorners(bounds, cornersWorld);

                for (int i = 0; i < cornersWorld.Count; i++)
                {
                    Vector3 c = cornersWorld[i];
                    c += worldPos;
                    Vector3 screenPos = cam.WorldToScreenPoint(c);
                    cornersScreen.Add(screenPos);
                }

                Vector2 xMinMax = new Vector2(cornersScreen[0].x, float.NegativeInfinity);
                Vector2 yMinMax = new Vector2(float.PositiveInfinity, float.NegativeInfinity);

                for (int i = 0; i < cornersScreen.Count; i++)
                {
                    Vector3 curr = cornersScreen[i];
                    xMinMax.x = Mathf.Min(curr.x, xMinMax.x);
                    xMinMax.y = Mathf.Max(curr.x, xMinMax.y);

                    yMinMax.x = Mathf.Min(curr.y, yMinMax.x);
                    yMinMax.y = Mathf.Max(curr.y, yMinMax.y);
                }

                Vector2 size = new Vector2(xMinMax.y - xMinMax.x, yMinMax.y - yMinMax.x);
                Rect rect = new Rect(xMinMax.x, yMinMax.x, size.x, size.y);

                return rect;
            }
        }

        public static Bounds GetBounds(GameObject obj)
        {
            Bounds bounds = new Bounds();
            using (ListPool<Renderer>.Get(out List<Renderer> renderers))
            {
                obj.GetComponentsInChildren<Renderer>(renderers);

                //Encapsulate for all renderers
                foreach (Renderer renderer in renderers)
                {
                    if (renderer.enabled)
                    {
                        bounds.Encapsulate(renderer.bounds);
                    }
                }

                return bounds;
            }
        }

        public static void GetCorners(this Bounds bounds, IList<Vector3> points)
        {
            Assert.IsTrue(points.Count == 8);
            Vector3 min = bounds.min;
            Vector3 max = bounds.max;

            // bottom Y square
            Vector3 boundPoint1 = new Vector3(min.x, min.y, min.z);
            Vector3 boundPoint2 = new Vector3(max.x, min.y, max.z);
            Vector3 boundPoint3 = new Vector3(min.x, min.y, max.z);
            Vector3 boundPoint4 = new Vector3(max.x, min.y, min.z);

            // top Y square
            Vector3 boundPoint5 = new Vector3(min.x, max.y, min.z);
            Vector3 boundPoint6 = new Vector3(max.x, max.y, max.z);
            Vector3 boundPoint7 = new Vector3(min.x, max.y, max.z);
            Vector3 boundPoint8 = new Vector3(max.x, max.y, min.z);

            points[0] = boundPoint1;
            points[1] = boundPoint2;
            points[2] = boundPoint3;
            points[3] = boundPoint4;
            points[4] = boundPoint5;
            points[5] = boundPoint6;
            points[6] = boundPoint7;
            points[7] = boundPoint8;
        }

        public static bool IsTotallyOutsideFrustum(Vector3[] boundsPoints, Camera cam)
        {
            for (int i = 0; i < 8; i++)
            {
                if (CanSeePoint(boundsPoints[i], cam))
                    return false;
            }

            return true;
        }

        public static bool CanSeePoint(Vector3 p, Camera cam)
        {
            Vector3 vp = cam.WorldToViewportPoint(p);
            if (vp.x < 0)
                return false;
            if (vp.x > 1)
                return false;
            if (vp.y < 0)
                return false;
            if (vp.y > 1)
                return false;
            if (vp.z < 0)
                return false;

            return true;
        }
    }
}