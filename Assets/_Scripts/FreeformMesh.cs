using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public static class FreeformMesh
{
	// Simplify Texture2D outline to ordered list of pixels, for better Mesh generation
	public static List<Vector2Int> TraceOutline(List<Vector2Int> rawPixels)
	{
		if (rawPixels.Count < 10) return new List<Vector2Int>();

		List<Vector2Int> sortedPath = new List<Vector2Int>();
		HashSet<Vector2Int> pixelSet = new HashSet<Vector2Int>(rawPixels);

		Vector2Int current = rawPixels[0];
		sortedPath.Add(current);
		pixelSet.Remove(current);

		// Nearest-Neighbor tracing
		while (pixelSet.Count > 0)
		{
			Vector2Int bestNeighbor = Vector2Int.zero;
			float minDst = float.MaxValue;
			bool found = false;

			// Check a 3x3 grid for neighbors
			for (int y = -1; y <= 1; y++)
			{
				for (int x = -1; x <= 1; x++)
				{
					if (x == 0 && y == 0) continue;

					Vector2Int neighbor = current + new Vector2Int(x, y);
					if (pixelSet.Contains(neighbor))
					{
						float d = Vector2.Distance(current, neighbor);
						if (d < minDst)
						{
							minDst = d;
							bestNeighbor = neighbor;
							found = true;
						}
					}
				}
			}

			if (!found)
			{
				float globalMin = float.MaxValue;
				foreach (var p in pixelSet)
				{
					float d = Vector2.Distance(current, p);
					if (d < globalMin)
					{
						globalMin = d;
						bestNeighbor = p;
					}
				}
			}

			current = bestNeighbor;
			sortedPath.Add(current);
			pixelSet.Remove(current);
		}

		return sortedPath;
	}

	// According to Teddy, the resampling creates better meshes from freeform paths.
	public static List<Vector2> ResamplePath(List<Vector2Int> inputPath, float interval)
	{
		// Convert to Vector2, so we can normalize the Vectors
		List<Vector2> vecPath = new List<Vector2>();
		foreach (var p in inputPath)
			vecPath.Add(new Vector2(p.x, p.y));

		if (vecPath.Count < 2) return vecPath;

		List<Vector2> cleanPath = new List<Vector2>();
		cleanPath.Add(vecPath[0]);

		float accumulatedDist = 0f;
		for (int i = 0; i < vecPath.Count - 1; i++)
		{
			Vector2 p1 = vecPath[i];
			Vector2 p2 = vecPath[i + 1];
			float d = Vector2.Distance(p1, p2);

			if (accumulatedDist + d >= interval)
			{
				float remaining = interval - accumulatedDist;
				Vector2 newPoint = p1 + (p2 - p1).normalized * remaining;

				cleanPath.Add(newPoint);

				vecPath.Insert(i + 1, newPoint);
				accumulatedDist = 0f;
			}
			else 
				accumulatedDist += d;
		}

		if (Vector2.Distance(cleanPath[0], cleanPath[cleanPath.Count - 1]) > interval)
			cleanPath.Add(cleanPath[0]);

		return cleanPath;
	}

	public static void RemoveExtraPoints(ref List<Vector2> outline, float eps = 0.001f)
	{
		float eps2 = eps * eps;

		if ((outline[0] - outline[outline.Count - 1]).sqrMagnitude <= eps2)
			outline.RemoveAt(outline.Count - 1);

		for (int i = outline.Count - 1; i > 0; --i)
		{
			if ((outline[i] - outline[i - 1]).sqrMagnitude <= eps2)
				outline.RemoveAt(i);
		}
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

		if (adj.Count == 0)
		{
			Debug.LogWarning("ExtractOutline: No boundary edges found in the mesh.");
			return new List<int>();
		}

		// Standard loop traversal logic
		int start = adj.Keys.Min();
		int prev = -1;
		int cur = start;

		List<int> loop = new List<int>(adj.Count);
		int safety = adj.Count + 8;

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

		// Convert Triangle.NET IDs to Unity indices
		List<int> unityLoop = new List<int>(loop.Count);
		foreach (int id in loop)
			unityLoop.Add(idToIndexMap[id]);

		return unityLoop;
	}

	public static void AppendSideWalls(List<int> boundaryLoopFront, int halfCount, List<int> tris)
	{
		int n = boundaryLoopFront.Count;
		for (int i = 0; i < n; i++)
		{
			int aF = boundaryLoopFront[i];
			int bF = boundaryLoopFront[(i + 1) % n];

			int aB = aF + halfCount;
			int bB = bF + halfCount;

			tris.Add(aF); tris.Add(bF); tris.Add(bB);
			tris.Add(aF); tris.Add(bB); tris.Add(aB);
		}
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
		if (!adj[u].Contains(v)) adj[u].Add(v);
	}

	public static Vector3[] SmoothAlongDirection(Vector3[] vertices, int[] triangles, Vector3 dir, float[] weights01,     // 0 = fixed, 1 = fully smooth
														 int iterations, float alpha)
	{
		dir = dir.normalized;
		int n = vertices.Length;

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
		float[] h = new float[n];
		for (int i = 0; i < n; i++)
		{
			float hi = Vector3.Dot(vertices[i], dir);
			h[i] = hi;
			basePos[i] = vertices[i] - dir * hi;
		}

		float[] hNext = new float[n];

		for (int it = 0; it < iterations; it++)
		{
			// Weighted Laplacian step along dir only
			for (int i = 0; i < n; i++)
			{
				float w = (weights01 != null && i < weights01.Length) ? Mathf.Clamp01(weights01[i]) : 1f;
				if (w <= 1e-6f) { hNext[i] = h[i]; continue; }

				var nb = neighbors[i];
				if (nb.Count == 0) { hNext[i] = h[i]; continue; }

				float avg = 0f;
				for (int k = 0; k < nb.Count; k++) avg += h[nb[k]];
				avg /= nb.Count;

				hNext[i] = Mathf.Lerp(h[i], avg, alpha * w);
			}

			float[] tmp = h;
			h = hNext;
			hNext = tmp;
		}

		// Add Vertices again
		for (int i = 0; i < n; i++)
			vertices[i] = basePos[i] + dir * h[i];

		return vertices;
	}

	public static Vector3[] SmoothMesh(Vector3[] vertices, int[] triangles, Vector3 dir, bool[] pinned, int iterations, float alpha)
	{
		dir.Normalize();

		int n = vertices.Length;

		// Split into planar component + thickness scalar along dir
		Vector3[] planar = new Vector3[n];
		float[] thick = new float[n];

		for (int i = 0; i < n; i++)
		{
			thick[i] = Vector3.Dot(vertices[i], dir);
			planar[i] = vertices[i] - dir * thick[i];
		}

		List<int>[] neighbors = new List<int>[n];
		for (int i = 0; i < n; i++) neighbors[i] = new List<int>(8);

		for (int t = 0; t < triangles.Length; t += 3)
		{
			int a = triangles[t];
			int b = triangles[t + 1];
			int c = triangles[t + 2];

			if (!neighbors[a].Contains(b)) neighbors[a].Add(b);
			if (!neighbors[a].Contains(c)) neighbors[a].Add(c);

			if (!neighbors[b].Contains(a)) neighbors[b].Add(a);
			if (!neighbors[b].Contains(c)) neighbors[b].Add(c);

			if (!neighbors[c].Contains(a)) neighbors[c].Add(a);
			if (!neighbors[c].Contains(b)) neighbors[c].Add(b);
		}

		// Iterative smoothing of thickness only
		for (int it = 0; it < iterations; it++)
		{
			float[] next = new float[n];

			for (int i = 0; i < n; i++)
			{
				if (pinned != null && pinned[i])
				{
					next[i] = thick[i];
					continue;
				}

				var nb = neighbors[i];
				if (nb.Count == 0)
				{
					next[i] = thick[i];
					continue;
				}

				float sum = 0f;
				int count = 0;

				for (int k = 0; k < nb.Count; k++)
				{
					int j = nb[k];
					if (pinned != null && pinned[j]) continue;
					sum += thick[j];
					count++;
				}

				// If all neighbors are pinned, keep current thickness
				if (count == 0)
				{	
					next[i] = thick[i];
					continue;
				}

				float avg = sum / count;
				next[i] = Mathf.Lerp(thick[i], avg, alpha);
			}

			thick = next;
		}

		// Recombine
		for (int i = 0; i < n; i++)
			vertices[i] = planar[i] + dir * thick[i];

		return vertices;
	}

	public static Vector3[] TaubinSmoothPositions(Vector3[] vertices, int[] triangles, bool[] pinned,
												  int iterations, float lambda, float mu)
	{
		int n = vertices.Length;

		List<int>[] neighbors = new List<int>[n];
		for (int i = 0; i < n; i++) neighbors[i] = new List<int>(8);

		for (int t = 0; t < triangles.Length; t += 3)
		{
			int a = triangles[t];
			int b = triangles[t + 1];
			int c = triangles[t + 2];

			if (!neighbors[a].Contains(b)) neighbors[a].Add(b);
			if (!neighbors[a].Contains(c)) neighbors[a].Add(c);

			if (!neighbors[b].Contains(a)) neighbors[b].Add(a);
			if (!neighbors[b].Contains(c)) neighbors[b].Add(c);

			if (!neighbors[c].Contains(a)) neighbors[c].Add(a);
			if (!neighbors[c].Contains(b)) neighbors[c].Add(b);
		}

		Vector3[] v = vertices;
		Vector3[] tmp = new Vector3[n];

		for (int it = 0; it < iterations; it++)
		{
			// lambda to smooth
			for (int i = 0; i < n; i++)
			{
				if (pinned != null && pinned[i]) { tmp[i] = v[i]; continue; }
				var nb = neighbors[i];
				if (nb.Count == 0) { tmp[i] = v[i]; continue; }

				Vector3 avg = Vector3.zero;
				for (int k = 0; k < nb.Count; k++) avg += v[nb[k]];
				avg /= nb.Count;

				tmp[i] = v[i] + lambda * (avg - v[i]);
			}

			// mu to stop shrinkage
			for (int i = 0; i < n; i++)
			{
				if (pinned != null && pinned[i]) { v[i] = tmp[i]; continue; }
				var nb = neighbors[i];
				if (nb.Count == 0) { v[i] = tmp[i]; continue; }

				Vector3 avg = Vector3.zero;
				for (int k = 0; k < nb.Count; k++) avg += tmp[nb[k]];
				avg /= nb.Count;

				v[i] = tmp[i] + mu * (avg - tmp[i]);
			}
		}

		return v;
	}
	#endregion


	//Leaving out until we get a smoother freeform mesh
	#region Mesh Simplification
	// Helpers
	private static ulong EdgeKey(int a, int b)
	{
		uint ua = (uint)Mathf.Min(a, b);
		uint ub = (uint)Mathf.Max(a, b);
		return ((ulong)ua << 32) | ub;
	}

	private static Vector3 TriNormal(Vector3[] v, int i0, int i1, int i2)
	{
		Vector3 a = v[i0], b = v[i1], c = v[i2];
		Vector3 n = Vector3.Cross(b - a, c - a);
		float mag = n.magnitude;
		return mag > 1e-12f ? (n / mag) : Vector3.zero;
	}

	public static void SimplifyMeshEdgeCollapse(ref Vector3[] vertices, ref int[] triangles, bool[] pinned, int targetTriangleCount,
												int maxPasses = 8, float maxEdgeLength = float.PositiveInfinity, float maxDihedralDegrees = 15f)
	{
		if (vertices == null || triangles == null) return;
		if (triangles.Length < 3) return;

		int triCount = triangles.Length / 3;
		targetTriangleCount = Mathf.Max(4, targetTriangleCount);
		if (triCount <= targetTriangleCount) return;

		var vecs = new List<Vector3>(vertices);
		var tris = new List<int>(triangles);

		List<bool> pin = null;
		if (pinned != null && pinned.Length == vertices.Length) pin = new List<bool>(pinned);

		for (int pass = 0; pass < maxPasses; pass++)
		{
			triCount = tris.Count / 3;
			if (triCount <= targetTriangleCount) break;

			// Build edges
			var edgeUse = new Dictionary<ulong, int>(tris.Count);
			var edgeTris = new Dictionary<ulong, (int t0, int t1)>(tris.Count);

			for (int t = 0; t < tris.Count; t += 3)
			{
				int a = tris[t], b = tris[t + 1], c = tris[t + 2];

				void Add(int u, int v, int triIndex)
				{
					ulong k = EdgeKey(u, v);
					edgeUse.TryGetValue(k, out int cnt);
					edgeUse[k] = cnt + 1;

					if (!edgeTris.TryGetValue(k, out var pair))
						edgeTris[k] = (triIndex, -1);
					else if (pair.t1 == -1)
						edgeTris[k] = (pair.t0, triIndex);
				}

				Add(a, b, t);
				Add(b, c, t);
				Add(c, a, t);
			}

			// Precompute normals per triangle for scoring candidates
			var triNormals = new Dictionary<int, Vector3>(triCount);
			for (int t = 0; t < tris.Count; t += 3)
			{
				triNormals[t] = TriNormal(vecs.ToArray(), tris[t], tris[t + 1], tris[t + 2]);
			}

			var candidates = new List<(float score, int u, int v)>(edgeTris.Count);
			foreach (var kv in edgeTris)
			{
				ulong key = kv.Key;
				int use = edgeUse[key];

				// Skip boundary edges
				if (use != 2) continue;

				var (t0, t1) = kv.Value;
				if (t0 < 0 || t1 < 0) continue;

				// Decode u, v back out of key
				int u = (int)(key >> 32);
				int v = (int)(key & 0xffffffff);

				if (u < 0 || v < 0 || u >= vecs.Count || v >= vecs.Count) continue;

				bool pu = (pin != null && u < pin.Count && pin[u]);
				bool pv = (pin != null && v < pin.Count && pin[v]);
				if (pu || pv) continue;

				float len = (vecs[u] - vecs[v]).magnitude;
				if (len > maxEdgeLength) continue;

				Vector3 n0 = triNormals[t0];
				Vector3 n1 = triNormals[t1];
				if (n0 == Vector3.zero || n1 == Vector3.zero) continue;

				float d = Mathf.Clamp(Vector3.Dot(n0, n1), -1f, 1f);
				float dih = Mathf.Acos(d) * Mathf.Rad2Deg;

				if (dih > maxDihedralDegrees) continue;

				float score = len * (1f + (dih / maxDihedralDegrees) * 0.25f);
				candidates.Add((score, u, v));
			}

			if (candidates.Count == 0) break;

			candidates.Sort((a, b) => a.score.CompareTo(b.score));

			// Greedy edge collapse
			int collapsesThisPass = Mathf.Clamp((triCount - targetTriangleCount) / 2, 8, 200);

			int performed = 0;
			var removed = new bool[vecs.Count];
			for (int ci = 0; ci < candidates.Count && performed < collapsesThisPass; ci++)
			{
				int u = candidates[ci].u;
				int v = candidates[ci].v;

				if (u == v) continue;
				if (u < 0 || v < 0 || u >= vecs.Count || v >= vecs.Count) continue;
				if (removed[u] || removed[v]) continue;

				Vector3 mid = 0.5f * (vecs[u] + vecs[v]);
				vecs[u] = mid;

				// Replace v with u
				for (int t = 0; t < tris.Count; t++)
				{
					if (tris[t] == v) tris[t] = u;
				}

				removed[v] = true;
				performed++;
			}

			if (performed == 0) break;

			// Remove bad triangles
			for (int t = tris.Count - 3; t >= 0; t -= 3)
			{
				int a = tris[t], b = tris[t + 1], c = tris[t + 2];
				if (a == b || b == c || c == a) tris.RemoveRange(t, 3);
			}

			// Remap triangles
			int nOld = vecs.Count;
			int[] remap = new int[nOld];
			var newV = new List<Vector3>(nOld);
			List<bool> newPin = (pin != null) ? new List<bool>(nOld) : null;

			for (int i = 0; i < nOld; i++)
			{
				if (removed[i])
				{
					remap[i] = -1;
					continue;
				}
				remap[i] = newV.Count;
				newV.Add(vecs[i]);
				if (newPin != null) newPin.Add(pin[i]);
			}
			
			for (int t = tris.Count - 3; t >= 0; t -= 3)
			{
				int a = remap[tris[t]];
				int b = remap[tris[t + 1]];
				int c = remap[tris[t + 2]];

				if (a < 0 || b < 0 || c < 0 || a == b || b == c || c == a)
				{
					tris.RemoveRange(t, 3);
					continue;
				}

				tris[t] = a;
				tris[t + 1] = b;
				tris[t + 2] = c;
			}

			vecs = newV;
			if (pin != null) pin = newPin;
			if (tris.Count / 3 <= targetTriangleCount) break;
		}

		vertices = vecs.ToArray();
		triangles = tris.ToArray();
	}
	#endregion
}
