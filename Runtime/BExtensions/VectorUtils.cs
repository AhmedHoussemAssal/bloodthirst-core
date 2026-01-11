using Bloodthirst.Core.Utils;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Pool;

namespace Bloodthirst.Scripts.Utils
{
    public static class VectorUtils
    {
        public struct VectorIndexPair
        {
            public Vector2 position;
            public int index;
        }

        public static void GetClosestInDirection(IReadOnlyList<Vector2> allSelection, Vector2 start, Vector2 dir, List<VectorIndexPair> sortedResult)
        {
            Assert.IsTrue( Mathf.Approximately( dir.sqrMagnitude , 1));
            Assert.IsTrue(sortedResult.Count == 0);

            // copy vec-index pairs
            for (int i = 0; i < allSelection.Count; i++)
            {
                sortedResult.Add(new VectorIndexPair() { index = i, position = allSelection[i] });
            }

            int rectCurrIdx = allSelection.IndexOf(start);
            Assert.IsTrue(rectCurrIdx != -1);

            VectorIndexPair rectCurr = sortedResult[rectCurrIdx];

            sortedResult.RemoveAt(rectCurrIdx);

            // remove the objects we consider non-aligned with dir
            for (int i = sortedResult.Count - 1; i >= 0; i--)
            {
                VectorIndexPair curr = sortedResult[i];
                Vector2 vecA = curr.position - rectCurr.position;

                float dotA = Vector2.Dot(vecA.normalized, dir);

                if (dotA < 0.1f)
                {
                    sortedResult.RemoveAt(i);
                }

            }

            if (sortedResult.Count == 0)
            {
                return;
            }

            // sort by alignment with dir , if angle is equal then we compare distance
            sortedResult.Sort((a, b) =>
            {
                Vector2 vecA = a.position - rectCurr.position;
                Vector2 vecB = b.position - rectCurr.position;

                float dotA = Vector2.Dot(vecA.normalized, dir);
                float dotB = Vector2.Dot(vecB.normalized, dir);

                float distA = Vector2.SqrMagnitude(vecA);
                float distB = Vector2.SqrMagnitude(vecB);

                float angleDiff = Mathf.Abs(dotA - dotB);

                if (angleDiff < 0.2f)
                {
                    return distA < distB ? -1 : 1;
                }

                return dotA > dotB ? -1 : 1;
            });
        }

        public static Vector3 To3D(this Vector2 vec2)
        {
            return new Vector3(vec2.x, 0, vec2.y);
        }

        public static Vector2 To2D(this Vector3 vec3)
        {
            return new Vector2(vec3.x, vec3.z);
        }

        public static Vector2 NormalizedToLayoutSpace(RectTransform rectTransform, Vector2 normalizedInLayout)
        {
            var sizeX = rectTransform.rect.width;
            var sizeY = rectTransform.rect.height;
            var offsetX = -rectTransform.pivot.x * rectTransform.rect.width;
            var offsetY = -rectTransform.pivot.y * rectTransform.rect.height;

            return new Vector2(normalizedInLayout.x * sizeX + offsetX, normalizedInLayout.y * sizeY + offsetY);
        }

        /// <summary>
        /// Find some projected angle measure off some forward around some axis.
        /// </summary>
        /// <param name="point"></param>
        /// <param name="forward"></param>
        /// <param name="axis"></param>
        /// <returns>Angle in degrees</returns>
        public static float AngleOffAroundAxis(Vector3 point, Vector3 forward, Vector3 axis, bool clockwise = false)
        {
            Vector3 right;
            if (clockwise)
            {
                right = Vector3.Cross(forward, axis);
                forward = Vector3.Cross(axis, right);
            }
            else
            {
                right = Vector3.Cross(axis, forward);
                forward = Vector3.Cross(right, axis);
            }
            return Mathf.Atan2(Vector3.Dot(point, right), Vector3.Dot(point, forward)) * Mathf.Rad2Deg;
        }

        /// Determines whether point P is inside the triangle ABC
        public static bool PointInTriangle(Vector2 A, Vector2 B, Vector2 C, Vector2 P)
        {
            double s1 = C.y - A.y;
            double s2 = C.x - A.x;
            double s3 = B.y - A.y;
            double s4 = P.y - A.y;

            double w1 = (A.x * s1 + s4 * s2 - P.x * s1) / (s3 * s2 - (B.x - A.x) * s1);
            double w2 = (s4 - w1 * s3) / s1;
            return w1 >= 0 && w2 >= 0 && (w1 + w2) <= 1;
        }

        /// <summary>
        /// Changes the vectors velue separatly and returns the vector with the new values
        /// </summary>
        /// <param name="vec"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <returns></returns>
        public static Vector3 Change(this Vector3 vec, float? x = null, float? y = null, float? z = null)
        {
            Vector3 res = vec;

            if (x != null)
                res.x = x.Value;

            if (y != null)
                res.y = y.Value;

            if (z != null)
                res.z = z.Value;

            return res;
        }

        /// <summary>
        /// <para>Returns the angle of the vector in degrees</para>
        /// <para>The value returned is between -180 and 180</para>
        /// </summary>
        /// <param name="vec"></param>
        /// <returns></returns>
        public static float ToAngleDeg(this Vector2 vec)
        {
            return Mathf.Atan2(vec.y, vec.x) * Mathf.Rad2Deg;
        }

        /// <summary>
        /// Snaps to the closest angles
        /// </summary>
        /// <param name="angle"></param>
        /// <param name="discreteAngles">The angles need to be sorted</param>
        /// <returns></returns>
        public static float SnapToClosestAngle(float angle, IReadOnlyList<float> discreteAngles)
        {
            Assert.IsTrue(angle <= 180);
            Assert.IsTrue(angle >= -180);

            if (angle < 0)
            {
                angle = 360 + angle;
            }

            float closestAngle = 0;

            for (int i = 0; i < discreteAngles.Count - 1; ++i)
            {
                var curr = discreteAngles[i];
                var next = discreteAngles[i + 1];
                if (!MathUtils.IsBetween(angle, curr, next))
                {
                    continue;
                }
                var diffA = angle - curr;
                var diffB = next - angle;

                closestAngle = diffA < diffB ? curr : next;
                break;
            }

            return closestAngle;
        }

        /// <summary>
        /// Returns the angle of the vector in radians
        /// </summary>
        /// <param name="vec"></param>
        /// <returns></returns>
        public static float ToAngleRad(this Vector2 vec)
        {
            return Mathf.Atan2(vec.y, vec.x);
        }

        /// <summary>
        /// Get a mask version of the vector
        /// </summary>
        /// <param name="original"></param>
        /// <returns></returns>
        public static Vector3 MulVector(this Vector3 original, float x = 1, float y = 1, float z = 1)
        {
            Vector3 res = original;
            res.x *= x;
            res.y *= y;
            res.z *= z;

            return res;
        }

        /// <summary>
        /// Mutliply color
        /// </summary>
        /// <param name="original"></param>
        /// <returns></returns>
        public static Color MulColor(this Color original, float r = 1, float g = 1, float b = 1, float a = 1)
        {
            Color res = original;
            res.r *= r;
            res.g *= g;
            res.b *= b;
            res.a *= a;

            return res;
        }

        public static Color WithAlpha(this Color src , float a)
        {
            src.a = a;
            return src;
        }

        /// <summary>
        /// return if 2 line intersect with the intersection point
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <param name="p4"></param>
        /// <param name="intersection"></param>
        /// <returns></returns>
        public static bool LineIntersection(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4, ref Vector2 intersection)
        {

            float Ax, Bx, Cx, Ay, By, Cy, d, e, f, num/*,offset*/;

            float x1lo, x1hi, y1lo, y1hi;



            Ax = p2.x - p1.x;

            Bx = p3.x - p4.x;



            // X bound box test/

            if (Ax < 0)
            {

                x1lo = p2.x; x1hi = p1.x;

            }
            else
            {

                x1hi = p2.x; x1lo = p1.x;

            }



            if (Bx > 0)
            {

                if (x1hi < p4.x || p3.x < x1lo) return false;

            }
            else
            {

                if (x1hi < p3.x || p4.x < x1lo) return false;

            }



            Ay = p2.y - p1.y;

            By = p3.y - p4.y;



            // Y bound box test//

            if (Ay < 0)
            {

                y1lo = p2.y; y1hi = p1.y;

            }
            else
            {

                y1hi = p2.y; y1lo = p1.y;

            }



            if (By > 0)
            {

                if (y1hi < p4.y || p3.y < y1lo) return false;

            }
            else
            {

                if (y1hi < p3.y || p4.y < y1lo) return false;

            }



            Cx = p1.x - p3.x;

            Cy = p1.y - p3.y;

            d = By * Cx - Bx * Cy;  // alpha numerator//

            f = Ay * Bx - Ax * By;  // both denominator//



            // alpha tests//

            if (f > 0)
            {

                if (d < 0 || d > f) return false;

            }
            else
            {

                if (d > 0 || d < f) return false;

            }



            e = Ax * Cy - Ay * Cx;  // beta numerator//



            // beta tests //

            if (f > 0)
            {

                if (e < 0 || e > f) return false;

            }
            else
            {

                if (e > 0 || e < f) return false;

            }



            // check if they are parallel

            if (f == 0) return false;

            // compute intersection coordinates //

            num = d * Ax; // numerator //

            //    offset = same_sign(num,f) ? f*0.5f : -f*0.5f;   // round direction //

            //    intersection.x = p1.x + (num+offset) / f;
            intersection.x = p1.x + num / f;



            num = d * Ay;

            //    offset = same_sign(num,f) ? f*0.5f : -f*0.5f;

            //    intersection.y = p1.y + (num+offset) / f;
            intersection.y = p1.y + num / f;



            return true;

        }
    }
}
