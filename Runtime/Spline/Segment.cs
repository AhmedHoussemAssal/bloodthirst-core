#if ODIN_INSPECTOR
	using Sirenix.OdinInspector;
#endif
using UnityEngine;
using UnityEngine.Assertions;

namespace Bloodthirst.Utils
{
    public class Segment<T> where T : INodePosition
    {
        #if ODIN_INSPECTOR[ShowInInspector]#endif
        public Vector3 Point1 { get; set; }
        #if ODIN_INSPECTOR[ShowInInspector]#endif
        public Vector3 Handle1 { get; set; }
        #if ODIN_INSPECTOR[ShowInInspector]#endif
        public Vector3 Point2 { get; set; }
        #if ODIN_INSPECTOR[ShowInInspector]#endif
        public Vector3 Handle2 { get; set; }
        #if ODIN_INSPECTOR[ShowInInspector]#endif
        public T Node1 { get; set; }
        #if ODIN_INSPECTOR[ShowInInspector]#endif
        public T Node2 { get; set; }

        private const int DRAW_ITERATIONS = 20;

        private bool isLengthInit;

        private float cachedLength;
        public float Length
        {
            get
            {
                if (!isLengthInit)
                {
                    cachedLength = CalculateLength();
                    cachedLength = (Point2 - Point1).magnitude;
                    isLengthInit = true;
                }
                return cachedLength;
            }
        }

        private Vector3 GetBezierPoint(Vector3 p0, Vector3 p1, Vector3 p2, float t)
        {
            return Vector3.LerpUnclamped(Vector3.LerpUnclamped(p0, p1, t), Vector3.LerpUnclamped(p1, p2, t), t);
        }
        private Vector3 GetQuadraticBezier(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {

            var a = GetBezierPoint(p0, p1, p2, t);
            var b = GetBezierPoint(p1, p2, p3, t);

            return Vector3.Lerp(a, b, t);
        }
        private Vector3 GetFirstDerivative(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
        {
            t = Mathf.Clamp01(t);
            float oneMinusT = 1f - t;
            return
                3f * oneMinusT * oneMinusT * (p1 - p0) +
                6f * oneMinusT * t * (p2 - p1) +
                3f * t * t * (p3 - p2);
        }

        private float CalculateLength()
        {
            float distance = 0;

            Vector3 lastPos = Point1;

            for (int c = 1; c < DRAW_ITERATIONS; c++)
            {
                float t = c / (float)DRAW_ITERATIONS;

                Vector3 newPos = GetPoint(t);

                distance += Vector3.Distance(lastPos, newPos);

                lastPos = newPos;
            }

            distance += Vector3.Distance(lastPos, Point2);

            return distance;
        }

        public Vector3 GetPoint(float t)
        {
            return GetQuadraticBezier(Point1, Handle1, Handle2, Point2, t);
        }

        public Vector3 GetNormal(float t)
        {
            return GetFirstDerivative(Point1, Handle1, Handle2, Point2, t);
        }
    }
}
