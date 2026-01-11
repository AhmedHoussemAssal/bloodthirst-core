using Bloodthirst.Scripts.Utils;
using Bloodthirst.Utils;
#if ODIN_INSPECTOR
	using Sirenix.OdinInspector;
#endif
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Pool;
using UnityEngine.UI;

namespace Bloodthirst.Core.UI
{
    [ExecuteAlways]
    public class UILineRenderer : MaskableGraphic
    {
        private readonly int LineLengthID = Shader.PropertyToID("_LineLength");
        private readonly int LineThicknessID = Shader.PropertyToID("_LineThickness");

        /// <summary>
        /// Number of sub-parts (segements) in between each point
        /// </summary>
        [SerializeField]
        public int resolution = 10;

        /// <summary>
        /// Thickness of the line
        /// </summary>
        public float lineThickness = 2;

        /// <summary>
        /// Method used for UV smoothing
        /// </summary>
        public UVSmoothingType uvSmoothing;

#if ODIN_INSPECTOR
        [ShowIf(nameof(uvSmoothing), Value = UVSmoothingType.LERP)]
#endif
        [Range(0f, 1f)]
        public float uvSmoothLerp;

        /// <summary>
        /// detail of defining the corners
        /// </summary>
        [Range(1, 10)]
        public int cornerSmoothing;

        public BSpline spline;

        public IReadOnlyList<Vector2> Points => points;

        /// <summary>
        /// The initial points that define the line (non smoothed)
        /// </summary>
        [SerializeField]
        private List<Vector2> points = new List<Vector2>();

        [HideInInspector]
        [SerializeField]
        private List<UIVertex> pooledVerts = new List<UIVertex>();

        [HideInInspector]
        [SerializeField]
        private List<int> pooledIndices = new List<int>();

        protected override void Awake()
        {
            base.Awake();
            InitPoints();
            spline = new BSpline(points);

            SetVerticesDirty();
            this.UpdateGeometry();
        }

        protected override void OnEnable()
        {
            base.OnEnable();

            if (spline == null)
            {
                spline = new BSpline(points);
            }

            SetVerticesDirty();
            this.UpdateGeometry();
        }
#if UNITY_EDITOR
        protected override void OnValidate()
        {
            base.OnValidate();

            if (!enabled)
                return;

            InitPoints();

            if (spline == null)
            {
                spline = new BSpline(points);
            }
            else
            {
                spline.SetVertexData(points);
            }
        }
#endif

        private void InitPoints()
        {
            if (points.Count < 4)
            {
                points.Clear();
                points.Add(VectorUtils.NormalizedToLayoutSpace(rectTransform, new Vector2(0, 0)));
                points.Add(VectorUtils.NormalizedToLayoutSpace(rectTransform, new Vector2(0, 0.5f)));
                points.Add(VectorUtils.NormalizedToLayoutSpace(rectTransform, new Vector2(0.5f, 1f)));
                points.Add(VectorUtils.NormalizedToLayoutSpace(rectTransform, new Vector2(1f, 1f)));
            }
        }

        public void RemovePoint(int index)
        {
            spline.RemovePoint(index);

            SetVerticesDirty();
            this.UpdateGeometry();
        }

        public void UpdatePoint(Vector2 point, int index)
        {
            spline.UpdatePoint(point, index);

            SetVerticesDirty();
            this.UpdateGeometry();
        }


        public void AddPoint(Vector2 point)
        {
            spline.AddPoint(point);

            SetVerticesDirty();
            this.UpdateGeometry();
        }


        [SerializeField]
        [HideInInspector]
        private List<Vector2> cachedInterpolatedPoints = new List<Vector2>();

        protected override void OnPopulateMesh(VertexHelper vbo)
        {
            cachedInterpolatedPoints.Clear();

            cachedInterpolatedPoints.Add(spline.VertexData[0]);

            for (int i = 0; i < spline.VertexData.Count - 3; i += 3)
            {
                for (int d = 0; d < resolution; d++)
                {
                    float t = (d + 1) / (float)(resolution);

                    Vector3 a = spline.GetPosition(i, t);

                    cachedInterpolatedPoints.Add(a);
                }
            }

            LineUtils.CurveSettings settings = new LineUtils.CurveSettings()
            {
                UVSmoothing = uvSmoothing,
                UVSmoothLerp = uvSmoothLerp,
                CornerSmoothing = cornerSmoothing,
                LineThikness = lineThickness,
                HandlesLength = 0,
                Resolution = resolution,
                NormalizeHandles = false,
                InvertHandles = false
            };

            // clear lists
            pooledVerts.Clear();
            pooledIndices.Clear();

            // calculate
            LineUtils.PointsToCurve(cachedInterpolatedPoints, settings, out float lineLength, ref pooledVerts, ref pooledIndices);

            for (int i = 0; i < pooledVerts.Count; i++)
            {
                UIVertex v = pooledVerts[i];
                v.color = color;
                pooledVerts[i] = v;           
            }

            materialForRendering.SetFloat(LineLengthID, lineLength);
            materialForRendering.SetFloat(LineThicknessID, lineThickness);

            // send
            vbo.Clear();
            vbo.AddUIVertexStream(pooledVerts, pooledIndices);
        }

    }

}
