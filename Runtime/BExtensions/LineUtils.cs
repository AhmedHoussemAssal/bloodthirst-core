using Bloodthirst.Utils;
using System;
using System.Collections.Generic;
using UnityEngine;

namespace Bloodthirst.Scripts.Utils
{
    public static class LineUtils
    {
        private struct LineQuad
        {
            public Vector2 LeftUpPos { get; set; }
            public Vector2 LeftDownPos { get; set; }
            public Vector2 RightUpPos { get; set; }
            public Vector2 RightDownPos { get; set; }
            public Vector2 Normal { get; set; }
        }

        private enum QUAD_POINT
        {
            LEFT_DOWN = 0,
            LEFT_UP = 1,
            RIGHT_UP = 2,
            RIGHT_DOWN = 3
        }

        private static int GetQuadIndex(int quadIndex, QUAD_POINT quadPoint, CurveSettings curveSettings)
        {
            return (quadIndex * (4 + (curveSettings.CornerSmoothing - 1))) + (int)quadPoint;
        }

        private static ref T GetQuadPoint<T>(T[] array , int quadIndex , QUAD_POINT quadPoint , CurveSettings curveSettings)
        {
            int index = GetQuadIndex(quadIndex, quadPoint, curveSettings);
            return ref array[index];
        }

        private static Vector3 RotatePointAroundPivot(Vector3 point, Vector3 pivot, Vector3 angles)
        {
            Vector3 dir = point - pivot; // get point direction relative to pivot
            dir = Quaternion.Euler(angles) * dir; // rotate it
            point = dir + pivot; // calculate rotated point
            return point; // return it
        }

        public struct CurveSettings
        {
            [SerializeField]
            private UVSmoothingType uVSmoothing;

            [SerializeField]
            private float uVSmoothLerp;

            [SerializeField]
            private int cornerSmoothing;

            [SerializeField]
            private float lineThikness;

            [SerializeField]
            private float handlesLength;

            [SerializeField]
            private int resolution;

            [SerializeField]
            private bool normalizeHandles;

            [SerializeField]
            private bool invertHandles;

            public UVSmoothingType UVSmoothing { get => uVSmoothing; set => uVSmoothing = value; }
            public float UVSmoothLerp { get => uVSmoothLerp; set => uVSmoothLerp = value; }
            public int CornerSmoothing { get => cornerSmoothing; set => cornerSmoothing = value; }
            public float LineThikness { get => lineThikness; set => lineThikness = value; }
            public float HandlesLength { get => handlesLength; set => handlesLength = value; }
            public int Resolution { get => resolution; set => resolution = value; }
            public bool NormalizeHandles { get => normalizeHandles; set => normalizeHandles = value; }
            public bool InvertHandles { get => invertHandles; set => invertHandles = value; }
        }

        public static void PointsToCurve(IReadOnlyList<Vector2> InterpolatedPoints, CurveSettings curveSettings, out float lineLength, ref List<UIVertex> tris, ref List<int> indices)
        {
            int segmentCount = (InterpolatedPoints.Count - 1);
            int gapsCount = segmentCount - 1;
            int totalVerticesCount = 
                // vertices for quads
                (segmentCount * 4) +

                // vertices for corners
                (gapsCount * (curveSettings.CornerSmoothing - 1) );

            Span<LineQuad> quads = stackalloc LineQuad[segmentCount];
            Span<UIVertex> vertices = stackalloc UIVertex[totalVerticesCount];

            // true means up
            // false means down
            bool[] gapDirection = new bool[gapsCount];

            // angles of gaps
            float[] angles = new float[gapsCount];

            // uv at vertex
            Vector2[] uvPerVertex = new Vector2[totalVerticesCount];

            lineLength = 0;

            float halfThicnessPlus = curveSettings.LineThikness / 2;
            float halfThicnessMinus = -curveSettings.LineThikness / 2;

            // create the quads that define the line width
            for (int i = 1; i < InterpolatedPoints.Count; i++)
            {
                Vector2 prev = InterpolatedPoints[i - 1];
                Vector2 cur = InterpolatedPoints[i];


                float angle = Mathf.Atan2(cur.y - prev.y, cur.x - prev.x) * Mathf.Rad2Deg;

                float currentLength = (cur - prev).magnitude;

                // form the thickness vertices
                // at this point they are not yet rotated to have the same direction as the segmenet
                Vector2 prevBottom = prev + new Vector2(0, halfThicnessMinus);
                Vector2 prevUp = prev + new Vector2(0, halfThicnessPlus);

                Vector2 currentBottom = cur + new Vector2(0, halfThicnessMinus);
                Vector2 currentUp = cur + new Vector2(0, halfThicnessPlus);

                // here we rotate them
                Vector3 rotateBy = new Vector3(0, 0, angle);
                prevBottom = RotatePointAroundPivot(prevBottom, prev, rotateBy);
                prevUp = RotatePointAroundPivot(prevUp, prev, rotateBy);
                currentUp = RotatePointAroundPivot(currentUp, cur, rotateBy);
                currentBottom = RotatePointAroundPivot(currentBottom, cur, rotateBy);

                LineQuad quad = new LineQuad()
                {
                    RightDownPos = currentBottom,
                    RightUpPos = currentUp,
                    LeftDownPos = prevBottom,
                    LeftUpPos = prevUp
                };

                quads[i - 1] = quad;
                lineLength += currentLength;
            }


            ///// sow the middle parts 
            // 0 : prev bottom
            // 1 : prev up
            // 2 : current up
            // 3 : current bottom


            // create the corner triangles
            {
                ref LineQuad prevQuad = ref quads[0];
                ref LineQuad nextQuad = ref quads[0];

                // here we treat the overlap between the quads
                // by pushing them enough to create a gap where we can insert a clean triangle
                for (int i = 1; i < quads.Length; i++)
                {
                    prevQuad = ref quads[i - 1];
                    nextQuad = ref quads[i];

                    Vector2 middleIntersectopBothQuads = Vector2.Lerp(prevQuad.RightDownPos, prevQuad.RightUpPos, 0.5f);

                    Vector2 middleOfFreeSpace = Vector2.Lerp(prevQuad.RightUpPos, nextQuad.LeftUpPos, 0.5f);

                    Vector2 middleLine = (middleOfFreeSpace - middleIntersectopBothQuads).normalized;
                    Vector2 middleToNext = (nextQuad.LeftUpPos - middleIntersectopBothQuads).normalized;

                    // this is angle going from the middle line towards the next quad (if we're going from left to right , it would be the one of the right)
                    float angleInDegrees = Vector2.SignedAngle(middleLine, middleToNext);

                    // if angle is more than than 0
                    // that means that the gap is on the bottom , and the intersection is at the top
                    // so we need to push the verts at the top first
                    if (angleInDegrees > 0)
                    {
                        gapDirection[i - 1] = false;
                        angles[i - 1] = angleInDegrees;

                        float angleInRads = (angleInDegrees) * Mathf.Deg2Rad;

                        // hyp = adj / cos(angle)
                        float hyp = (halfThicnessPlus) / Mathf.Cos(angleInRads);

                        Vector2 topCuttingPointForBothQuads = middleIntersectopBothQuads + (middleLine * hyp);

                        Vector2 inter = topCuttingPointForBothQuads;

                        // calculate new segment lengths
                        Vector2 prevPush = inter - prevQuad.RightUpPos;
                        Vector2 nextPush = inter - nextQuad.LeftUpPos;


                        // replace with new intersection point
                        // push prev
                        prevQuad.RightUpPos = inter;
                        prevQuad.RightDownPos += prevPush;

                        // replace with new intersection point
                        // push next
                        nextQuad.LeftUpPos = inter;
                        nextQuad.LeftDownPos += nextPush;

                    }

                    // if angle is less than than 0
                    // that means that the gap is one top , and the intersection is at the bottom
                    // so we need to push the verts at the bottom first
                    else
                    {
                        gapDirection[i - 1] = true;
                        angles[i - 1] = angleInDegrees;

                        float angleInRads = (angleInDegrees) * Mathf.Deg2Rad;

                        // hyp = adj / cos(angle)
                        float hyp = (halfThicnessPlus) / Mathf.Cos(angleInRads);

                        Vector2 bottomCuttingPointForBothQuads = middleIntersectopBothQuads - (middleLine * hyp);

                        Vector2 inter = bottomCuttingPointForBothQuads;

                        // calculate new segment lengths
                        Vector2 prevPush = inter - prevQuad.RightDownPos;
                        Vector2 nextPush = inter - nextQuad.LeftDownPos;


                        // replace with new intersection point
                        // push prev
                        prevQuad.RightDownPos = inter;
                        prevQuad.RightUpPos += prevPush;

                        // replace with new intersection point
                        // push next
                        nextQuad.LeftDownPos = inter;
                        nextQuad.LeftUpPos += nextPush;
                    }
                }

            }


            // push verts and indicies
            {
                ref LineQuad prevQuad = ref quads[0];
                ref LineQuad nextQuad = ref quads[0];

                // add first

                vertices[0] = new UIVertex() { position = prevQuad.LeftDownPos };
                vertices[1] = new UIVertex() { position = prevQuad.LeftUpPos };
                vertices[2] = new UIVertex() { position = prevQuad.RightUpPos };
                vertices[3] = new UIVertex() { position = prevQuad.RightDownPos };

                indices.Add(0);
                indices.Add(1);
                indices.Add(2);

                indices.Add(0);
                indices.Add(2);
                indices.Add(3);



                for (int i = 1; i < quads.Length; i++)
                {
                    int prevIndexOffset = (i - 1)   * (4 + (curveSettings.CornerSmoothing - 1));
                    int nextIndexOffset = i         * (4 + (curveSettings.CornerSmoothing - 1));

                    prevQuad = ref quads[i - 1];
                    nextQuad = ref quads[i];

                    float ang = angles[i - 1];

                    // gap bottom
                    if (gapDirection[i - 1] == false)
                    {
                        int gapIntersection = GetQuadIndex(i - 1 , QUAD_POINT.RIGHT_UP , curveSettings);

                        int currentVertToTieIn = GetQuadIndex(i - 1, QUAD_POINT.RIGHT_DOWN, curveSettings);

                        Vector2 rotatingPivot = prevQuad.RightUpPos;
                        Vector2 rotatingPoint = prevQuad.RightDownPos;

                        for (int c = 1; c < curveSettings.CornerSmoothing; c++)
                        {
                            float t = c / (float)curveSettings.CornerSmoothing;
                            Quaternion rotationNeeded = Quaternion.AngleAxis(t * ang * 2, Vector3.forward);
                            Vector2 nextPoint = (Vector2)(rotationNeeded * (rotatingPoint - rotatingPivot)) + rotatingPivot;

                            int nextIndex = prevIndexOffset + 3 + c;

                            vertices[nextIndex] = new UIVertex() { position = nextPoint };

                            indices.Add(currentVertToTieIn);
                            indices.Add(gapIntersection);
                            indices.Add(nextIndex);

                            currentVertToTieIn = nextIndex;

                        }

                        indices.Add(currentVertToTieIn);
                        indices.Add(gapIntersection);

                        // lastIndexToTieInto
                        int lastPoint = GetQuadIndex(i, QUAD_POINT.LEFT_DOWN, curveSettings);

                        indices.Add(lastPoint);
                    }

                    // gap top
                    else
                    {

                        int gapIntersection = GetQuadIndex(i - 1 , QUAD_POINT.RIGHT_DOWN , curveSettings);

                        int currentVertToTieIn = GetQuadIndex(i -1 , QUAD_POINT.RIGHT_UP, curveSettings);

                        Vector2 rotatingPivot = prevQuad.RightDownPos;
                        Vector2 rotatingPoint = prevQuad.RightUpPos;

                        for (int c = 1; c < curveSettings.CornerSmoothing; c++)
                        {
                            float t = c / (float)curveSettings.CornerSmoothing;
                            var rotationNeeded = Quaternion.AngleAxis(t * ang * 2, Vector3.forward);
                            Vector2 nextPoint = (Vector2)(rotationNeeded * (rotatingPoint - rotatingPivot)) + rotatingPivot;

                            int nextIndex = prevIndexOffset + 3 + c;

                            vertices[nextIndex] = new UIVertex() { position = nextPoint };


                            indices.Add(gapIntersection);
                            indices.Add(currentVertToTieIn);
                            indices.Add(nextIndex);


                            currentVertToTieIn = nextIndex;
                        }



                        indices.Add(gapIntersection);
                        indices.Add(currentVertToTieIn);

                        // lastIndexToTieInto
                        int lastPoint = GetQuadIndex(i, QUAD_POINT.LEFT_UP, curveSettings);

                        indices.Add(lastPoint);

                    }


                    vertices[nextIndexOffset + 0] = new UIVertex() { position = nextQuad.LeftDownPos };
                    vertices[nextIndexOffset + 1] = new UIVertex() { position = nextQuad.LeftUpPos };
                    vertices[nextIndexOffset + 2] = new UIVertex() { position = nextQuad.RightUpPos };
                    vertices[nextIndexOffset + 3] = new UIVertex() { position = nextQuad.RightDownPos };

                    indices.Add(nextIndexOffset + 0);
                    indices.Add(nextIndexOffset + 1);
                    indices.Add(nextIndexOffset + 2);

                    indices.Add(nextIndexOffset + 0);
                    indices.Add(nextIndexOffset + 2);
                    indices.Add(nextIndexOffset + 3);

                }


            }


            // start assigning the UV points

            float topLength = 0;
            float bottomLength = 0;

            // get the length per side
            {


                for (int i = 0; i < quads.Length - 1; i++)
                {
                    int prevIndexOffset = i;
                    int nextIndexOffset = i + 1;

                    LineQuad prevQuad = quads[prevIndexOffset];
                    LineQuad nextQuad = quads[nextIndexOffset];

                    int lu = GetQuadIndex(i, QUAD_POINT.LEFT_UP, curveSettings);
                    int ld = GetQuadIndex(i, QUAD_POINT.LEFT_DOWN, curveSettings);


                    uvPerVertex[lu].x = topLength;
                    uvPerVertex[lu].y = 1;

                    uvPerVertex[ld].x = bottomLength;
                    uvPerVertex[ld].y = 0;


                    // note : we can use the same length for both sides
                    float length = (prevQuad.RightUpPos - prevQuad.LeftUpPos).magnitude;

                    topLength += length;
                    bottomLength += length;

                    int ru = GetQuadIndex(i, QUAD_POINT.RIGHT_UP, curveSettings);
                    int rd = GetQuadIndex(i, QUAD_POINT.RIGHT_DOWN, curveSettings);

                    uvPerVertex[ru].x = topLength;
                    uvPerVertex[ru].y = 1;

                    uvPerVertex[rd].x = bottomLength;
                    uvPerVertex[rd].y = 0;

                    float ang = angles[i];

                    // gap up
                    if (gapDirection[i] == true)
                    {
                        Vector2 rotatingPoint = prevQuad.RightUpPos;

                        for (int c = 1; c < curveSettings.CornerSmoothing; c++)
                        {
                            int nextIndex = GetQuadIndex(i, QUAD_POINT.RIGHT_DOWN, curveSettings) + c;
                            Vector2 nextVert = vertices[nextIndex].position;

                            topLength += (rotatingPoint - nextVert).magnitude;

                            uvPerVertex[rd + c].x = topLength;
                            uvPerVertex[rd + c].y = 1;

                            rotatingPoint = nextVert;
                        }

                        Vector2 lastPoint = nextQuad.LeftUpPos;
                        topLength += (rotatingPoint - lastPoint).magnitude;
                    }

                    // gap down
                    else
                    {
                        Vector2 rotatingPoint = prevQuad.RightDownPos;

                        for (int c = 1; c < curveSettings.CornerSmoothing; c++)
                        {
                            int nextIndex = GetQuadIndex(i , QUAD_POINT.RIGHT_DOWN , curveSettings) + c;
                            Vector2 nextVert = vertices[nextIndex].position;

                            float distToAdd = (rotatingPoint - nextVert).magnitude;
                            bottomLength += distToAdd;
                            
                            uvPerVertex[rd + c].x = bottomLength;
                            uvPerVertex[rd + c].y = 0;

                            rotatingPoint = nextVert;
                        }

                        Vector2 lastPoint = nextQuad.LeftDownPos;
                        float lengthToAdd = (rotatingPoint - lastPoint).magnitude;
                        bottomLength += lengthToAdd;
                    }

                }

                //last quard
                {
                    int lastIndex = quads.Length - 1;
                    LineQuad last = quads[lastIndex];

                    int lu = GetQuadIndex(lastIndex, QUAD_POINT.LEFT_UP, curveSettings);
                    int ld = GetQuadIndex(lastIndex, QUAD_POINT.LEFT_DOWN, curveSettings);

                    uvPerVertex[lu].x = topLength;
                    uvPerVertex[lu].y = 1;

                    uvPerVertex[ld].x = bottomLength;
                    uvPerVertex[ld].y = 0;

                    // note : we can use the same length for both sides
                    float length = (last.RightUpPos - last.LeftUpPos).magnitude;

                    topLength += length;
                    bottomLength += length;

                    int ru = GetQuadIndex(lastIndex, QUAD_POINT.RIGHT_UP, curveSettings);
                    int rd = GetQuadIndex(lastIndex, QUAD_POINT.RIGHT_DOWN, curveSettings);

                    uvPerVertex[ru].x = topLength;
                    uvPerVertex[ru].y = 1;

                    uvPerVertex[rd].x = bottomLength;
                    uvPerVertex[rd].y = 0;

                }
            }

            // divide by length to get the uv value
            {
                for (int i = 0; i < quads.Length - 1; i++)
                {
                    int prevIndexOffset = i;
                    int nextIndexOffset = i + 1;

                    LineQuad prevQuad = quads[prevIndexOffset];
                    LineQuad nextQuad = quads[nextIndexOffset];

                    int lu = GetQuadIndex(i, QUAD_POINT.LEFT_UP, curveSettings);
                    int ld = GetQuadIndex(i, QUAD_POINT.LEFT_DOWN, curveSettings);
                    int ru = GetQuadIndex(i, QUAD_POINT.RIGHT_UP, curveSettings);
                    int rd = GetQuadIndex(i, QUAD_POINT.RIGHT_DOWN, curveSettings);

                    uvPerVertex[lu].x /= topLength;
                    uvPerVertex[ru].x /= topLength;

                    uvPerVertex[ld].x /= bottomLength;
                    uvPerVertex[rd].x /= bottomLength;

                    float ang = angles[i];

                    // gap up
                    if (gapDirection[i] == true)
                    {
                        for (int c = 1; c < curveSettings.CornerSmoothing; c++)
                        {
                            int nextIndex = GetQuadIndex(i, QUAD_POINT.RIGHT_DOWN, curveSettings) + c;
                            uvPerVertex[nextIndex].x /= topLength;
                        }
                    }

                    // gap down
                    else
                    {
                        for (int c = 1; c < curveSettings.CornerSmoothing; c++)
                        {
                            int nextIndex = GetQuadIndex(i, QUAD_POINT.RIGHT_DOWN, curveSettings) + c;   
                            uvPerVertex[nextIndex].x /= bottomLength;
                        }
                    }

                }

                //last quard
                {
                    int lastIndex = quads.Length - 1;
                    LineQuad last = quads[lastIndex];

                    int lu = GetQuadIndex(lastIndex, QUAD_POINT.LEFT_UP, curveSettings);
                    int ld = GetQuadIndex(lastIndex, QUAD_POINT.LEFT_DOWN, curveSettings);
                    int ru = GetQuadIndex(lastIndex, QUAD_POINT.RIGHT_UP, curveSettings);
                    int rd = GetQuadIndex(lastIndex, QUAD_POINT.RIGHT_DOWN, curveSettings);

                    uvPerVertex[lu].x /= topLength;
                    uvPerVertex[ru].x /= topLength;

                    uvPerVertex[ld].x /= bottomLength;
                    uvPerVertex[rd].x /= bottomLength;
                }
            }

            // assign the uv value to each vertex
            for(int i = 0; i < uvPerVertex.Length; i++)
            {
                ref UIVertex v = ref vertices[i];
                v.uv0 = uvPerVertex[i];
            }

            tris.Capacity = vertices.Length;

            for (int i = 0; i < vertices.Length; i++)
            {           
                tris.Add(vertices[i]);
            }
        }
    }
}
