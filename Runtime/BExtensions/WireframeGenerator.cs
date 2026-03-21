using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Pool;

public static class WireframeGenerator
{
    public static void GenerateWireframeMesh(
        Mesh sourceMesh,
        Mesh wireframeMesh,
        float lineWidth)
    {
        Assert.IsNotNull(sourceMesh);
        Assert.IsNotNull(wireframeMesh);
        Assert.IsTrue(wireframeMesh.vertexCount == 0);

        using (ListPool<Vector3>.Get(out var verticies))
        using (ListPool<int>.Get(out var triangles))
        using (ListPool<Vector3>.Get(out var newVerts))
        using (ListPool<int>.Get(out var newTris))
        {
            sourceMesh.GetVertices(verticies);
            sourceMesh.GetTriangles(triangles, 0);

            for (int i = 0; i < triangles.Count; i += 3)
            {
                Vector3 v0 = verticies[triangles[i]];
                Vector3 v1 = verticies[triangles[i + 1]];
                Vector3 v2 = verticies[triangles[i + 2]];

                // Center of triangle
                Vector3 center = (v0 + v1 + v2) / 3f;

                // Inner triangle (controls thickness)
                Vector3 i0 = Vector3.Lerp(v0, center, lineWidth);
                Vector3 i1 = Vector3.Lerp(v1, center, lineWidth);
                Vector3 i2 = Vector3.Lerp(v2, center, lineWidth);

                int baseIndex = newVerts.Count;

                // Outer
                newVerts.Add(v0); // 0
                newVerts.Add(v1); // 1
                newVerts.Add(v2); // 2

                // Inner
                newVerts.Add(i0); // 3
                newVerts.Add(i1); // 4
                newVerts.Add(i2); // 5

                // --- Edge v0-v1 strip ---
                newTris.Add(baseIndex + 0);
                newTris.Add(baseIndex + 1);
                newTris.Add(baseIndex + 4);

                newTris.Add(baseIndex + 0);
                newTris.Add(baseIndex + 4);
                newTris.Add(baseIndex + 3);

                // --- Edge v1-v2 strip ---
                newTris.Add(baseIndex + 1);
                newTris.Add(baseIndex + 2);
                newTris.Add(baseIndex + 5);

                newTris.Add(baseIndex + 1);
                newTris.Add(baseIndex + 5);
                newTris.Add(baseIndex + 4);

                // --- Edge v2-v0 strip ---
                newTris.Add(baseIndex + 2);
                newTris.Add(baseIndex + 0);
                newTris.Add(baseIndex + 3);

                newTris.Add(baseIndex + 2);
                newTris.Add(baseIndex + 3);
                newTris.Add(baseIndex + 5);
            }

            wireframeMesh.SetVertices(newVerts);
            wireframeMesh.SetTriangles(newTris, 0);

            wireframeMesh.RecalculateNormals();
            wireframeMesh.RecalculateBounds();
        }
    }
}