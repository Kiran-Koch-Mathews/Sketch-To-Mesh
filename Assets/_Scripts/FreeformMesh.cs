using System.Collections.Generic;
using System.Linq;
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

public static class FreeformMesh
{
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

	public static List<SpineSegment> ExtractAxis(TriangleNet.Mesh mesh)
	{
		List<SpineSegment> segments = new List<SpineSegment>();

		foreach (var tri in mesh.Triangles)
		{
			List<int> internalEdgeIndices = new List<int>();
			for (int i = 0; i < 3; i++)
			{
				if (tri.GetNeighbor(i) != null) // null = mesh boundary
				{
					internalEdgeIndices.Add(i);
				}
			}

			int neighborCount = internalEdgeIndices.Count;

			switch (neighborCount)
			{
				case 1: // TERMINAL

					// Connect the midpoint of the internal edge to the opposite vertex
					int edgeIndex = internalEdgeIndices[0];
					Vector2 midPoint = GetEdgeMidpoint(tri, edgeIndex);
					Vector2 oppositeVertex = GetVertexPos(tri, edgeIndex);

					segments.Add(new SpineSegment{ Start = midPoint, End = oppositeVertex, Type = TriangleType.Terminal });
					break;
				
				case 2: // SLEEVE
					// Connect the midpoints of the two internal edges
					Vector2 mid1 = GetEdgeMidpoint(tri, internalEdgeIndices[0]);
					Vector2 mid2 = GetEdgeMidpoint(tri, internalEdgeIndices[1]);

					segments.Add(new SpineSegment{ Start = mid1, End = mid2, Type = TriangleType.Sleeve });
					break;

				default: // JUNCTION
					// Connect the center of the triangle to the midpoints of all 3 edges
					Vector2 center = GetTriangleCentroid(tri);

					foreach (int e in internalEdgeIndices)
					{
						Vector2 end = GetEdgeMidpoint(tri, e);
						segments.Add(new SpineSegment{ Start = center, End = end, Type = TriangleType.Junction });
					}
					break;
			}
		}

		return segments;
	}
	#endregion

	/*
	 * Different attempts at smoothing freeform meshes.
	 * 1. Smoothing only along a given direction (z-direction) (This worked the best)
	 * 2. Smoothing with pinned vertices
	 * 3. Taubin smoothing (Laplacian + inverse Laplacian)
	 */
	#region Smoothing
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

	#region Side Wall Generation
	public static List<int> ExtractOutline(TriangleNet.Mesh tMesh, Dictionary<int, int> idToIndexMap)
	{
		Dictionary<int, List<int>> adj = new Dictionary<int, List<int>>();

		foreach (var tri in tMesh.Triangles)
		{
			for (int i = 0; i < 3; i++)
			{
				// In Triangle.NET, the edge i is between vertices (i+1)%3 and (i+2)%3
				if (tri.GetNeighbor(i) == null)
				{
					int u = tri.GetVertex((i + 1) % 3).ID;
					int v = tri.GetVertex((i + 2) % 3).ID;

					if (!adj.TryGetValue(u, out var lu)) adj[u] = lu = new List<int>(2);
					if (!adj.TryGetValue(v, out var lv)) adj[v] = lv = new List<int>(2);

					lu.Add(v);
					lv.Add(u);
				}
			}
		}

		// Safety check
		if (adj.Count == 0)
		{
			Debug.LogWarning("ExtractOutline: No boundary edges found in the mesh.");
			return new List<int>();
		}

		int start = adj.Keys.Min();
		int prev = -1;
		int cur = start;

		List<int> loop = new List<int>(adj.Count);
		int safety = adj.Count + 10;

		while (safety > 0)
		{
			loop.Add(cur);
			var nbs = adj[cur];

			// Pick the neighbor that isn't the one we just came from
			int next = (nbs.Count > 1 && nbs[0] == prev) ? nbs[1] : nbs[0];

			prev = cur;
			cur = next;

			if (cur == start) break;
			safety--;
		}

		// Safety Check
		if (safety == 0)
		{
			Debug.LogWarning("ExtractOutline: Boundary loop extraction hit safety limit.");
			return new List<int>();
		}

		// Convert Triangle.NET IDs to Unity indices
		List<int> unityLoop = new List<int>(loop.Count);
		foreach (int id in loop)
			unityLoop.Add(idToIndexMap[id]);

		return unityLoop;
	}

	public static void AppendSideWalls(List<int> boundaryLoopFront, int halfCount, List<int> tris, Vector3[] vertices)
	{
		int n = boundaryLoopFront.Count;
		// Calculate the geometric center of the rim to determine "outward"
		Vector3 meshCenter = Vector3.zero;
		foreach (int idx in boundaryLoopFront)
			meshCenter += vertices[idx];

		meshCenter /= n;

		for (int i = 0; i < n; i++)
		{
			int aF = boundaryLoopFront[i];
			int bF = boundaryLoopFront[(i + 1) % n];
			int aB = aF + halfCount;
			int bB = bF + halfCount;

			Vector3 sideCenter = (vertices[aF] + vertices[bF]) * 0.5f;
			Vector3 toSide = (sideCenter - meshCenter).normalized;

			// Calculate the normal of the triangle we are about to make
			Vector3 v1 = vertices[bF] - vertices[aF];
			Vector3 v2 = vertices[aB] - vertices[aF];
			Vector3 normal = Vector3.Cross(v1, v2).normalized;

			// If the normal points inward (toward center), flip the winding
			if (Vector3.Dot(normal, toSide) < 0)
			{
				tris.Add(aF); tris.Add(aB); tris.Add(bF);
				tris.Add(bF); tris.Add(aB); tris.Add(bB);
			}
			else
			{
				tris.Add(aF); tris.Add(bF); tris.Add(aB);
				tris.Add(bF); tris.Add(bB); tris.Add(aB);
			}
		}
	}
	#endregion
}
