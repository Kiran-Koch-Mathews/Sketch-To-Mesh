using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Topology;
using UnityEngine;

public enum TriangleType 
{ 
	Terminal,
	Sleeve,
	Junction
}
public struct SpineSegment
{
	public Vector2 Start;
	public Vector2 End;
	public TriangleType Type;
}

public class Vector2EqualityComparer : IEqualityComparer<Vector2>
{
	private const float Tolerance = 0.001f;

	public bool Equals(Vector2 a, Vector2 b)
	{
		return Vector2.Distance(a, b) < Tolerance;
	}

	public int GetHashCode(Vector2 obj)
	{
		// Round to nearest 0.01 for hashing
		int x = Mathf.RoundToInt(obj.x * 100f);
		int y = Mathf.RoundToInt(obj.y * 100f);
		return x ^ (y << 16);
	}
}

public static class FreeformMesh
{
	#region Boundary Extraction
	// Simplify Texture2D outline to ordered list of pixels, for better Mesh generation
	public static List<Vector2Int> TraceOutline(List<Vector2Int> rawPixels)
	{
		// Maybe a misclick, ignore
		if (rawPixels.Count < 10) return new List<Vector2Int>();

		HashSet<Vector2Int> pixelSet = new HashSet<Vector2Int>(rawPixels);
		HashSet<Vector2Int> edgePixels = new HashSet<Vector2Int>();

		Vector2Int startPixel = rawPixels[0];
		int minX = int.MaxValue;

		// Filter out inner pixels
		foreach (var p in rawPixels)
		{
			if (!pixelSet.Contains(p + Vector2Int.up) ||
				!pixelSet.Contains(p + Vector2Int.down) ||
				!pixelSet.Contains(p + Vector2Int.left) ||
				!pixelSet.Contains(p + Vector2Int.right))
			{
				edgePixels.Add(p);
				if (p.x < minX) 
				{ 
					minX = p.x; 
					startPixel = p;
				}
			}
		}

		List<Vector2Int> sortedPath = new List<Vector2Int>();
		Vector2Int current = startPixel;
		sortedPath.Add(current);
		edgePixels.Remove(current);

		//The Brush Width is greater than the distance from each outer pixel,
		//so it will only trace the outer edge
		while (edgePixels.Count > 0)
		{
			Vector2Int bestNeighbor = Vector2Int.zero;
			float minDist = float.MaxValue;
			bool found = false;

			//Hard-coded 5x5 search for next pixel
			//Could switch for brush radius later
			for (int y = -2; y <= 2; y++)
			{
				for (int x = -2; x <= 2; x++)
				{
					if (x == 0 && y == 0) continue;

					Vector2Int neighbor = current + new Vector2Int(x, y);
					if (edgePixels.Contains(neighbor))
					{
						float d = Vector2.Distance(current, neighbor);
						if (d < minDist)
						{
							minDist = d;
							bestNeighbor = neighbor;
							found = true;
						}
					}
				}
			}

			if (found)
			{
				current = bestNeighbor;
				sortedPath.Add(current);
				edgePixels.Remove(current);
			}
			else break;
		}
		return sortedPath;
	}

	// According to Teddy, the resampling creates better meshes from freeform paths.
	// Also removes jitter and helps runtime
	public static List<Vector2> ResamplePath(List<Vector2Int> inputPath, float interval)
	{
		if (inputPath.Count < 2) return null;

		List<Vector2> cleanPath = new List<Vector2>();
		Vector2 current = new Vector2(inputPath[0].x, inputPath[0].y);
		cleanPath.Add(current);

		int i = 1;
		while (i < inputPath.Count)
		{
			Vector2 nextTarget = new Vector2(inputPath[i].x, inputPath[i].y);
			float d = Vector2.Distance(current, nextTarget);

			if (d >= interval)
			{
				Vector2 dir = (nextTarget - current).normalized;
				current = current + dir * interval; //Ensure exact spacing
				cleanPath.Add(current);
			}
			else i++;
		}

		// Ensure a closed loop
		if (Vector2.Distance(cleanPath[0], cleanPath[cleanPath.Count - 1]) > interval * 0.5f)
			cleanPath.Add(cleanPath[0]);

		return cleanPath;
	}
	public static float GetDistanceToOutline(Vector2 p, List<Vector2> boundary)
	{
		float minDst = float.MaxValue;
		for (int i = 0; i < boundary.Count; i++)
		{
			Vector2 p1 = boundary[i];
			Vector2 p2 = boundary[(i + 1) % boundary.Count];

			Vector2 pa = p - p1, ba = p2 - p1;
			float h = Mathf.Clamp01(Vector2.Dot(pa, ba) / Vector2.Dot(ba, ba));
			float d = (pa - ba * h).magnitude;

			if (d < minDst) minDst = d;
		}
		return minDst;
	}
	#endregion

	#region Chordal Axis
	private static Vector2 GetVertexPos(TriangleNet.Topology.Triangle tri, int i)
	{
		var v = tri.GetVertex(i);
		return new Vector2((float)v.X, (float)v.Y);
	}

	private static Vector2 GetEdgeMidpoint(TriangleNet.Topology.Triangle tri, int i)
	{
		var v1 = tri.GetVertex((i + 1) % 3);
		var v2 = tri.GetVertex((i + 2) % 3);
		return new Vector2((float)(v1.X + v2.X) * 0.5f, (float)(v1.Y + v2.Y) * 0.5f);
	}

	private static Vector2 GetTriangleCentroid(TriangleNet.Topology.Triangle tri)
	{
		var v0 = tri.GetVertex(0);
		var v1 = tri.GetVertex(1);
		var v2 = tri.GetVertex(2);

		float centerX = (float)(v0.X + v1.X + v2.X) / 3f;
		float centerY = (float)(v0.Y + v1.Y + v2.Y) / 3f;
		return new Vector2(centerX, centerY);
	}

	public static List<SpineSegment> ExtractAxis(TriangleNet.Mesh mesh, HashSet<int> prunedIds = null)
	{
		List<SpineSegment> segments = new List<SpineSegment>();

		foreach (var tri in mesh.Triangles)
		{
			if (prunedIds != null && prunedIds.Contains(tri.ID)) continue;

			List<int> internalEdgeIndices = new List<int>();
			for (int i = 0; i < 3; i++)
			{
				var neighbor = tri.GetNeighbor(i);
				// A neighbor is internal if it exists AND hasn't been pruned
				if (neighbor != null && (prunedIds == null || !prunedIds.Contains(neighbor.ID)))
				{
					internalEdgeIndices.Add(i);
				}
			}

			int neighborCount = internalEdgeIndices.Count;

			switch (neighborCount)
			{
				case 1:
					int edgeIndex = internalEdgeIndices[0];
					segments.Add(new SpineSegment { Start = GetEdgeMidpoint(tri, edgeIndex), End = GetVertexPos(tri, edgeIndex), Type = TriangleType.Terminal });
					break;

				case 2:
					segments.Add(new SpineSegment { Start = GetEdgeMidpoint(tri, internalEdgeIndices[0]), End = GetEdgeMidpoint(tri, internalEdgeIndices[1]), Type = TriangleType.Sleeve });
					break;

				case 3:
					Vector2 center = GetTriangleCentroid(tri);
					foreach (int e in internalEdgeIndices)
						segments.Add(new SpineSegment { Start = center, End = GetEdgeMidpoint(tri, e), Type = TriangleType.Junction });
					break;
			}
		}

		return segments;
	}
	#endregion

	#region Pruning, Fanning, Retriangulation
	#region Helper Methods
	public static Dictionary<int, int> BuildIdToIndexMap(TriangleNet.Mesh mesh)
	{
		Dictionary<int, int> idToIndexMap = new Dictionary<int, int>();

		int unityIndex = 0;
		foreach (var vertex in mesh.Vertices)
		{
			idToIndexMap[vertex.ID] = unityIndex;
			unityIndex++;
		}

		return idToIndexMap;
	}

	private static TriangleType GetTriangleType(TriangleNet.Topology.Triangle t)
	{
		int neighbors = 0;
		for (int i = 0; i < 3; i++)
		{
			if (t.GetNeighbor(i) != null) neighbors++;
		}

		if (neighbors == 1) return TriangleType.Terminal;
		if (neighbors == 2) return TriangleType.Sleeve;
		if (neighbors == 3) return TriangleType.Junction;

		return TriangleType.Terminal; // Safety
	}
	private static bool GetSharedEdge(TriangleNet.Topology.Triangle t1, TriangleNet.Topology.Triangle t2, out Vector2 v1, out Vector2 v2)
	{
		// Find the two vertices shared by t1 and t2
		List<Vertex> shared = new List<Vertex>();
		for (int i = 0; i < 3; i++)
		{
			Vertex v = t1.GetVertex(i);
			// Check if t2 has this vertex
			for (int j = 0; j < 3; j++)
			{
				if (t2.GetVertex(j).ID == v.ID)
				{
					shared.Add(v);
					break;
				}
			}
		}

		if (shared.Count >= 2)
		{
			v1 = new Vector2((float)shared[0].X, (float)shared[0].Y);
			v2 = new Vector2((float)shared[1].X, (float)shared[1].Y);
			return true;
		}

		v1 = Vector2.zero;
		v2 = Vector2.zero;
		return false;
	}
	#endregion

	public static List<SpineSegment> PruneBranches(TriangleNet.Mesh mesh)
	{
        HashSet<int> prunedIds = new HashSet<int>();

        var terminals = mesh.Triangles.Where(t => GetTriangleType(t) == TriangleType.Terminal).ToList();

        foreach (var t in terminals)
        {
            if (prunedIds.Contains(t.ID)) continue; //Somehow reached another terminal triangle

            // Trace the branch inwards
            List<Triangle> branch = new List<Triangle>();
            Triangle curr = t;
            Triangle prev = null;
            Triangle junction = null;
            bool hitJunction = false;

            int safety = 0;
            while (safety++ < 1000)
            {
                branch.Add(curr);

                // Find the next internal neighbor that isn't 'prev'
                Triangle next = null;
                for (int i = 0; i < 3; i++)
                {
                    Triangle n = (Triangle)curr.GetNeighbor(i);
                    if (n != null && n != prev) //Internal
                    {
                        next = n;
                        break;
                    }
                }

                var nextType = GetTriangleType(next);
                if (nextType == TriangleType.Sleeve)
                {
					prev = curr;
					curr = next;
                }
                else if (nextType == TriangleType.Junction)
                {
					hitJunction = true;
					junction = next;
					break;
				}
                else break; // Terminal or null, end of branch
			}

            // Pruning check: If the branch fits inside the semi-circle of the base chord
            if (hitJunction && junction != null)
            {
                // The base chord is the edge shared between the last branch triangle (curr) and the junction
                if (GetSharedEdge(curr, junction, out Vector2 v1, out Vector2 v2))
                {
                    Vector2 midpoint = (v1 + v2) * 0.5f;
                    float diameter = Vector2.Distance(v1, v2);
                    float radius = diameter * 0.5f;

                    bool prune = true;
                    foreach (var tri in branch)
                    {
                        for (int i = 0; i < 3; i++)
                        {
                            Vector2 v = GetVertexPos(tri, i);
                            
							// Semi-Circle
                            if (Vector2.Distance(v, midpoint) > radius + 0.001f)
                            {
                                prune = false;
                                break;
                            }
                        }
                        if (!prune) break;
                    }

                    if (prune)
                    {
                        foreach (var tri in branch) prunedIds.Add(tri.ID);
                    }
                }
            }
        }

        return ExtractAxis(mesh, prunedIds);
    }

	public static TriangleNet.Mesh RetriangulateMeshAroundSpine(TriangleNet.Mesh mesh, List<SpineSegment> prunedSpine, List<Vector2> boundary)
	{
		Polygon newPoly = new Polygon();

		List<Vertex> boundaryVertices = new List<Vertex>();
		foreach (Vector2 p in boundary)
		{
			Vertex v = new Vertex(p.x, p.y, 0); // 0 = boundary marker
			newPoly.Add(v);
			boundaryVertices.Add(v);
		}
		newPoly.Add(new Contour(boundaryVertices));

		// Collect all unique spine points from the pruned spine
		HashSet<Vector2> spinePointsSet = new HashSet<Vector2>(new Vector2EqualityComparer());
		foreach (var segment in prunedSpine)
		{
			spinePointsSet.Add(segment.Start);
			spinePointsSet.Add(segment.End);
		}

		// Add spine points as Steiner points
		List<Vertex> spineVertices = new List<Vertex>();
		foreach (var point in spinePointsSet)
		{
			Vertex v = new Vertex(point.x, point.y, 1); // 1 = spine marker
			newPoly.Add(v);
			spineVertices.Add(v);
		}

		// Add spine segments as internal constraints
		foreach (var segment in prunedSpine)
		{
			// Find the vertices corresponding to this segment
			Vertex start = null, end = null;

			foreach (var v in spineVertices)
			{
				Vector2 vPos = new Vector2((float)v.X, (float)v.Y);
				if (Vector2.Distance(vPos, segment.Start) < 0.001f) start = v;
				if (Vector2.Distance(vPos, segment.End) < 0.001f) end = v;
			}

			// Add as a constraint edge if both vertices found
			if (start != null && end != null)
			{

				var seg = new Segment(start, end, 1); // 1 = spine edge marker
				newPoly.Add(seg);
			}
		}

		// Perform constrained Delaunay triangulation with the spine constraints
		var meshOptions = new TriangleNet.Meshing.ConstraintOptions()
		{
			ConformingDelaunay = false,
			Convex = false
		};

		var retriangulatedMesh = (TriangleNet.Mesh)newPoly.Triangulate(meshOptions, null);
		return retriangulatedMesh;
	}
	#endregion

	#region Smoothing
	/*
	 * Different attempts at smoothing freeform meshes.
	 * 1. Smoothing only along a given direction (z-direction) (This worked the best)
	 * 2. Smoothing with pinned vertices
	 * 3. Taubin smoothing (Laplacian + inverse Laplacian)
	 */

	private static void AddNeighbor(List<int>[] adj, int u, int v)
	{
		// Only add it if its not already there
		if (!adj[u].Contains(v))
			adj[u].Add(v);
	}

	public static Vector3[] SmoothAlongDirection(Vector3[] vertices, int[] triangles, Vector3 dir, float[] weights01,     // 0 = fixed, 1 = fully smooth
														 int iterations, float alpha)
	{
		dir = dir.normalized;
		int n = vertices.Length;

		// Build neighbor list for every vertex
		List<int>[] neighbors = new List<int>[n];
		for (int i = 0; i < n; i++) neighbors[i] = new List<int>(8);

		for (int t = 0; t < triangles.Length; t += 3)
		{
			int a = triangles[t];
			int b = triangles[t + 1];
			int c = triangles[t + 2];

			AddNeighbor(neighbors, a, b);
			AddNeighbor(neighbors, a, c);
			AddNeighbor(neighbors, b, a);
			AddNeighbor(neighbors, b, c);
			AddNeighbor(neighbors, c, a);
			AddNeighbor(neighbors, c, b);
		}

		// Look for base + height
		Vector3[] basePos = new Vector3[n];
		float[] h = new float[n]; // distance along 'dir'
		for (int i = 0; i < n; i++)
		{
			float hi = Vector3.Dot(vertices[i], dir);
			h[i] = hi;
			basePos[i] = vertices[i] - dir * hi;
		}

		// Iterative smoothing
		float[] hNext = new float[n];
		for (int it = 0; it < iterations; it++)
		{
			// Weighted Laplacian step along dir only
			for (int i = 0; i < n; i++)
			{
				// 0 = skip smoothing, 1 = full smoothing
				float w = (weights01 != null && i < weights01.Length) ? Mathf.Clamp01(weights01[i]) : 1f;
				if (w <= 1e-6f || neighbors[i].Count == 0) 
				{ 
					hNext[i] = h[i];
					continue; 
				}

				// Find the average height of neighbors
				float avg = 0f;
				for (int k = 0; k < neighbors[i].Count; k++) 
					avg += h[neighbors[i][k]];

				avg /= neighbors[i].Count;

				hNext[i] = Mathf.Lerp(h[i], avg, alpha * w);
			}

			// Make the newly calculated heights the current heights
			float[] tmp = h;
			h = hNext;
			hNext = tmp;
		}

		// Add Vertices again
		for (int i = 0; i < n; i++)
			vertices[i] = basePos[i] + dir * h[i];

		return vertices;
	}
	#endregion
}
