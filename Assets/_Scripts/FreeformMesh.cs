using System;
using System.Collections.Generic;
using System.Linq;
using TriangleNet.Geometry;
using TriangleNet.Topology;
using UnityEngine;

public enum TriangleType { Terminal, Sleeve, Junction }
public struct SpineSegment
{
	public Vector2 Start;
	public Vector2 End;
	public TriangleType Type;
}

public class Vector2EqualityComparer : IEqualityComparer<Vector2>
{
	private readonly float _tolerance;
	private readonly float _hashScale;

	public Vector2EqualityComparer(float tolerance = 0.001f)
	{
		_tolerance = tolerance;
		_hashScale = 1f / tolerance;
	}

	public bool Equals(Vector2 a, Vector2 b)
	{
		return Vector2.Distance(a, b) < _tolerance;
	}

	public int GetHashCode(Vector2 obj)
	{
		int x = Mathf.RoundToInt(obj.x * _hashScale);
		int y = Mathf.RoundToInt(obj.y * _hashScale);
		return x ^ (y << 16);
	}
}

public static class FreeformMesh
{
	private const float Eps = 0.001f;

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

		if (cleanPath.Count > 1 && Vector2.Distance(cleanPath[^1], cleanPath[0]) < interval * 0.1f)
			cleanPath.RemoveAt(cleanPath.Count - 1);
		
		return cleanPath;
	}
	#endregion

	#region Triangle.Net Helpers
	private static float Cross(Vector2 o, Vector2 a, Vector2 b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
	private static Vector2 ToVector2(Vertex v) => new Vector2((float)v.X, (float)v.Y);
	private static Vector2 GetTriangleCentroid(Triangle tri)
		=> (ToVector2(tri.GetVertex(0)) + ToVector2(tri.GetVertex(1)) + ToVector2(tri.GetVertex(2))) / 3f;
	private static Vector2 GetEdgeMidpoint(Triangle tri, int i)
		=> (ToVector2(tri.GetVertex((i + 1) % 3)) + ToVector2(tri.GetVertex((i + 2) % 3))) * 0.5f;
	private static TriangleType GetTriangleType(Triangle t)
	{
		int neighbors = 0;
		for (int i = 0; i < 3; i++)
		{
			if (t.GetNeighbor(i) != null) neighbors++;
		}

		if (neighbors == 1) return TriangleType.Terminal;
		if (neighbors == 2) return TriangleType.Sleeve;
		if (neighbors == 3) return TriangleType.Junction;

		Debug.LogWarning($"Triangle {t.ID} has unexpected neighbor count: {neighbors}");
		return TriangleType.Terminal; // Safety
	}
	private static int ClassifyVertex(Vertex v, List<Vector2> spinePoints, List<Vector2> boundaryPoints)
	{
		Vector2 pos = ToVector2(v);
		if (boundaryPoints.Any(b => Vector2.Distance(pos, b) < Eps)) return 0;
		if (spinePoints.Any(s => Vector2.Distance(pos, s) < Eps)) return 1;
		return -1; //Unknown or Steiner Point
	}
	private static bool IsDegenerate(int l0, int l1, int l2) => l0 >= 0 && l0 == l1 && l1 == l2;
	private static (Vertex v0, Vertex v1, Vertex v2) GetTriangleVertices(ITriangle tri)
		=> (tri.GetVertex(0), tri.GetVertex(1), tri.GetVertex(2));
	private static bool IsOnSegment(Vector2 p, Vector2 a, Vector2 b)
	{
		Vector2 ab = b - a;
		float len = ab.magnitude;
		if (len < Eps) return false;

		float cross = Mathf.Abs((b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x));
		if (cross / len > Eps) return false; // not collinear

		float t = Vector2.Dot(p - a, ab) / (len * len);

		// strictly between endpoints
		return t > Eps && t < 1f - Eps; 
	}
	#endregion

	#region Chordal Axis
	public static List<SpineSegment> ExtractAxis(TriangleNet.Mesh mesh, HashSet<int> prunedIds = null)
	{
		List<SpineSegment> segments = new List<SpineSegment>();

		foreach (var tri in mesh.Triangles)
		{
			// Skip pruned triangles if provided
			if (prunedIds != null && prunedIds.Contains(tri.ID)) continue;

			List<int> internalEdgeIndices = new List<int>();
			for (int i = 0; i < 3; i++)
			{
				var neighbor = tri.GetNeighbor(i);
				if (neighbor != null && (prunedIds == null || !prunedIds.Contains(neighbor.ID)))
				{
					internalEdgeIndices.Add(i);
				}
			}

			int neighborCount = internalEdgeIndices.Count;
			switch (neighborCount)
			{
				case 1: // Terminal
					int edgeIndex = internalEdgeIndices[0];
					Vector2 terminalEnd;
					if (prunedIds != null)
					{
						int prunedEdgeIdx = -1;
						for (int i = 0; i < 3; i++)
						{
							var nb = tri.GetNeighbor(i);
							if (nb != null && prunedIds.Contains(nb.ID))
							{
								prunedEdgeIdx = i;
								break;
							}
						}

						terminalEnd = (prunedEdgeIdx >= 0) ? GetEdgeMidpoint(tri, prunedEdgeIdx) : ToVector2(tri.GetVertex(edgeIndex));
					}
					else terminalEnd = ToVector2(tri.GetVertex(edgeIndex));

					segments.Add(new SpineSegment { Start = GetEdgeMidpoint(tri, edgeIndex), End = terminalEnd, Type = TriangleType.Terminal });
					break;

				case 2: // Sleeve
					segments.Add(new SpineSegment { Start = GetEdgeMidpoint(tri, internalEdgeIndices[0]), End = GetEdgeMidpoint(tri, internalEdgeIndices[1]), Type = TriangleType.Sleeve });
					break;

				case 3: // Junction
					Vector2 center = GetTriangleCentroid(tri);
					foreach (int e in internalEdgeIndices)
						segments.Add(new SpineSegment { Start = center, End = GetEdgeMidpoint(tri, e), Type = TriangleType.Junction });
					break;

				default:
					Debug.LogWarning($"Triangle {tri.ID} has unexpected neighbor count: {neighborCount}");
					break;
			}
		}

		return segments;
	}
	#endregion

	#region Pruning
	#region Helpers
	private static bool GetSharedEdge(Triangle t1, Triangle t2, out Vector2 v1, out Vector2 v2)
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
			v1 = ToVector2(shared[0]);
			v2 = ToVector2(shared[1]);
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
		foreach (var terminal in terminals)
		{
			if (prunedIds.Contains(terminal.ID)) continue;
			List<Triangle> region = new List<Triangle> { terminal };
			HashSet<Vector2> regionVerts = new HashSet<Vector2>(new Vector2EqualityComparer());
			for (int i = 0; i < 3; i++)
				regionVerts.Add(ToVector2(terminal.GetVertex(i)));

			Triangle curr = terminal;
			Triangle prev = null;

			int safety = 0;
			while (safety++ < 1000)
			{
				// Find the single un-visited internal neighbor
				Triangle next = null;
				for (int i = 0; i < 3; i++)
				{
					Triangle n = (Triangle)curr.GetNeighbor(i);
					if (n != null && n != prev)
					{
						next = n;
						break;
					}
				}

				// Get shared edge to make semi-circle test.
				if (next == null || !GetSharedEdge(curr, next, out Vector2 v1, out Vector2 v2)) break;

				Vector2 midpoint = (v1 + v2) * 0.5f;
				float radius = Vector2.Distance(v1, v2) * 0.5f;

				// Check every vertex gainst the semicircle.
				bool allInside = true;
				foreach (var v in regionVerts)
				{
					if (Vector2.Distance(v, midpoint) > radius + Eps)
					{
						allInside = false;
						break;
					}
				}

				if (!allInside) break; //End Pruning

				TriangleType nextType = GetTriangleType(next);
				if (nextType == TriangleType.Junction) break; // End Pruning if we hit a junction

				prev = curr;
				curr = next;
				region.Add(curr);
				for (int i = 0; i < 3; i++)
					regionVerts.Add(ToVector2(curr.GetVertex(i)));
			}

			if (safety >= 1000)
			{
				Debug.LogWarning("Safety break in pruning loop. Possible infinite loop or very large branch.");
			}

			foreach (var tri in region)
				prunedIds.Add(tri.ID);
		}

		return ExtractAxis(mesh, prunedIds);
	}
	#endregion

	#region Retriangulation
	#region Helpers
	private static List<(Vector2 spine, Vector2 boundary)> CollectNonDegenerateEdges(TriangleNet.Mesh mesh, List<Vector2> spinePoints, List<Vector2> boundaryPoints)
	{
		var result = new List<(Vector2 spine, Vector2 boundary)>();

		// Track pairs by vertex-ID so we never add the same edge twice.
		var seen = new HashSet<(int spineId, int boundaryId)>();
		foreach (var tri in mesh.Triangles)
		{
			var (v0, v1, v2) = GetTriangleVertices(tri);
			int l0 = ClassifyVertex(v0, spinePoints, boundaryPoints);
			int l1 = ClassifyVertex(v1, spinePoints, boundaryPoints);
			int l2 = ClassifyVertex(v2, spinePoints, boundaryPoints);

			// Skip degenerate triangles — these are exactly what we are trying to remove.
			if (IsDegenerate(l0, l1, l2)) continue;

			Vertex[] verts = { v0, v1, v2 };
			int[] labels = { l0, l1, l2 };

			for (int i = 0; i < 3; i++)
			{
				int j = (i + 1) % 3;
				bool oneSpine = labels[i] == 1 && labels[j] == 0;
				bool oneBoundary = labels[i] == 0 && labels[j] == 1;
				if (!oneSpine && !oneBoundary) continue;

				Vertex sv = oneSpine ? verts[i] : verts[j];
				Vertex bv = oneSpine ? verts[j] : verts[i];

				var key = (sv.ID, bv.ID);
				if (seen.Contains(key)) continue;
				seen.Add(key);

				result.Add((ToVector2(sv), ToVector2(bv)));
			}
		}

		return result;
	}

	private static Vertex FindUnsharedVertex(Vertex[] verts, Vertex sharedA, Vertex sharedB)
	{
		foreach (var v in verts)
		{
			if (Vector2.Distance(ToVector2(v), ToVector2(sharedA)) >= Eps &&
				Vector2.Distance(ToVector2(v), ToVector2(sharedB)) >= Eps)
				return v;
		}
		return null;
	}
	private static bool DiagonalCrossesConstraint(Vector2 a, Vector2 b, List<SpineSegment> prunedSpine)
	{
		foreach (var spine in prunedSpine)
		{
			if (Vector2.Distance(a, spine.Start) < Eps || Vector2.Distance(a, spine.End) < Eps ||
				Vector2.Distance(b, spine.Start) < Eps || Vector2.Distance(b, spine.End) < Eps)
				continue;

			float d1 = Cross(spine.Start, spine.End, a);
			float d2 = Cross(spine.Start, spine.End, b);
			float d3 = Cross(a, b, spine.Start);
			float d4 = Cross(a, b, spine.End);

			if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
				((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
				return true;
		}

		return false;
	}
	private static (Vector2 spine, Vector2 boundary)? FindOneFixingDiagonal(TriangleNet.Mesh mesh, List<Vector2> spinePoints, List<Vector2> boundaryPoints, List<SpineSegment> prunedSpine, List<(Vector2 spine, Vector2 boundary)> existingPairs)
	{
		foreach (var tri in mesh.Triangles)
		{
			var (v0, v1, v2) = GetTriangleVertices(tri);
			int l0 = ClassifyVertex(v0, spinePoints, boundaryPoints);
			int l1 = ClassifyVertex(v1, spinePoints, boundaryPoints);
			int l2 = ClassifyVertex(v2, spinePoints, boundaryPoints);

			if (!IsDegenerate(l0, l1, l2)) continue;

			for (int i = 0; i < 3; i++)
			{
				ITriangle neighbor = tri.GetNeighbor(i);
				if (neighbor == null || neighbor.ID < 0)
					continue;

				var (n0, n1, n2) = GetTriangleVertices(neighbor);
				int nl0 = ClassifyVertex(n0, spinePoints, boundaryPoints);
				int nl1 = ClassifyVertex(n1, spinePoints, boundaryPoints);
				int nl2 = ClassifyVertex(n2, spinePoints, boundaryPoints);

				if (IsDegenerate(nl0, nl1, nl2)) continue;

				Vertex apex = tri.GetVertex(i);
				Vertex sharedA = tri.GetVertex((i + 1) % 3);
				Vertex sharedB = tri.GetVertex((i + 2) % 3);
				Vertex unsharedVertex = FindUnsharedVertex(new[] { n0, n1, n2 }, sharedA, sharedB);

				if (unsharedVertex == null)
				{
					Debug.LogWarning($"Could not find unshared vertex for triangle {neighbor.ID} neighbor of {tri.ID}");
					continue;
				}

				int apexLabel = ClassifyVertex(apex, spinePoints, boundaryPoints);
				int unsharedLabel = ClassifyVertex(unsharedVertex, spinePoints, boundaryPoints);

				// They should be different, otherwise this edge is not a candidate for fixing
				if (apexLabel == unsharedLabel) continue;

				Vector2 spinePos = ToVector2(apexLabel == 1 ? apex : unsharedVertex);
				Vector2 boundaryPos = ToVector2(apexLabel == 1 ? unsharedVertex : apex);

				bool tried = existingPairs.Exists(p =>
					Vector2.Distance(p.spine, spinePos) < Eps &&
					Vector2.Distance(p.boundary, boundaryPos) < Eps);

				if (tried) continue;

				// Reject this diagonal if it crosses any existing spine segment
				if (DiagonalCrossesConstraint(spinePos, boundaryPos, prunedSpine))
					continue;

				return (spinePos, boundaryPos);
			}
		}

		return null;
	}

	private static Polygon BuildPolygon(List<Vector2> boundaryPoints, List<Vector2> spinePoints, List<SpineSegment> prunedSpine, List<(Vector2 spine, Vector2 boundary)> fixingPairs)
	{
		var poly = new Polygon();

		// Add Boundary Vertices and Contours
		List<Vertex> boundaryVertices = new List<Vertex>();
		foreach (var p in boundaryPoints)
			boundaryVertices.Add(new Vertex(p.x, p.y) { Label = 0 });  // Boundary
		poly.Add(new Contour(boundaryVertices));

		// Add Spine Vertices
		List<Vertex> spineVertices = new List<Vertex>();
		foreach (var p in spinePoints)
		{
			var v = new Vertex(p.x, p.y) { Label = 1 }; // Spine
			spineVertices.Add(v);
			poly.Add(v);
		}

		// Add Spine segments
		foreach (var segment in prunedSpine)
		{
			Vertex start = null, end = null;
			foreach (var v in spineVertices)
			{
				Vector2 vPos = ToVector2(v);
				if (Vector2.Distance(vPos, segment.Start) < Eps) start = v;
				if (Vector2.Distance(vPos, segment.End) < Eps) end = v;
			}
			if (start != null && end != null) poly.Add(new Segment(start, end) { Label = 1 });
		}

		// Fixing diagonals
		if (fixingPairs != null)
		{
			foreach (var (spinePos, boundaryPos) in fixingPairs)
			{
				Vertex sv = spineVertices.Find(v => Vector2.Distance(ToVector2(v), spinePos) < Eps);
				Vertex bv = boundaryVertices.Find(v => Vector2.Distance(ToVector2(v), boundaryPos) < Eps);

				if (sv != null && bv != null) poly.Add(new Segment(sv, bv) { Label = 1 });
				else Debug.LogWarning($"Could not resolve fixing diagonal: spine={spinePos} boundary={boundaryPos}");
			}
		}

		return poly;
	}
	#endregion
	public static TriangleNet.Mesh Retriangulate(TriangleNet.Mesh mesh, List<SpineSegment> prunedSpine, List<Vector2> boundaryPoints)
	{
		var meshOptions = new TriangleNet.Meshing.ConstraintOptions()
		{
			ConformingDelaunay = false,
			Convex = false,
			SegmentSplitting = 0 // Only use Spine
		};

		// Unique List of spine points
		HashSet<Vector2> spinePointsSet = new HashSet<Vector2>(new Vector2EqualityComparer());
		foreach (var segment in prunedSpine)
		{
			spinePointsSet.Add(segment.Start);
			spinePointsSet.Add(segment.End);
		}

		var spinePoints = spinePointsSet.ToList();
		var allFixingPairs = new List<(Vector2 spine, Vector2 boundary)>();
		var currentPoly = BuildPolygon(boundaryPoints, spinePoints, prunedSpine, allFixingPairs);
		var currentMesh = (TriangleNet.Mesh)currentPoly.Triangulate(meshOptions, null);
		
		//First, collect all non-degenerate Delauney Triangles
		var seedPairs = CollectNonDegenerateEdges(currentMesh, spinePoints, boundaryPoints);
		foreach (var pair in seedPairs)
		{
			bool tried = allFixingPairs.Exists(p =>
				Vector2.Distance(p.spine, pair.spine) < Eps &&
				Vector2.Distance(p.boundary, pair.boundary) < Eps);

			if (!tried) allFixingPairs.Add(pair);
		}

		if (seedPairs.Count > 0)
		{
			Debug.Log($"Seeded {seedPairs.Count} non-degenerate spine to boundary edge(s) as hard constraints.");
			var seededPoly = BuildPolygon(boundaryPoints, spinePoints, prunedSpine, allFixingPairs);
			currentMesh = (TriangleNet.Mesh)seededPoly.Triangulate(meshOptions, null);
		}
		else Debug.LogWarning("Potential Seeded Triangulation Error");

		int safety = 0;
		while (safety++ < 1000)
		{
			var newPair = FindOneFixingDiagonal(currentMesh, spinePoints, boundaryPoints, prunedSpine, allFixingPairs);

			if (newPair == null)
			{
				Debug.Log($"Mesh clean after {safety - 1} fixing iteration(s).");
				break;
			}

			allFixingPairs.Add(newPair.Value);
			Debug.Log($"Iteration {safety}: adding diagonal {newPair.Value.spine} to {newPair.Value.boundary}");
			var newPoly = BuildPolygon(boundaryPoints, spinePoints, prunedSpine, allFixingPairs);
			currentMesh = (TriangleNet.Mesh)newPoly.Triangulate(meshOptions, null);
		}

		if (safety >= 1000)
			Debug.LogWarning("Safety break in retriangulation loop. Possible infinite loop or very stubborn degenerate cases.");

		return currentMesh;
	}
	#endregion

	#region Subdivision
	public static TriangleNet.Mesh Subdivide(TriangleNet.Mesh mesh, List<Vector2> spinePoints, List<Vector2> boundaryPoints, int subdivisions)
	{
		var meshOptions = new TriangleNet.Meshing.ConstraintOptions()
		{
			ConformingDelaunay = false,
			Convex = false,
			SegmentSplitting = 0 // Only use Spine
		};

		var vcComparer = new Vector2EqualityComparer(1e-5f);
		var vc = new Dictionary<Vector2, int>(vcComparer);
		var verts = new List<Vector2>();
		var edgeSet = new HashSet<(int, int)>();

		#region Inline Helpers
		void AddEdge(Vector2 a, Vector2 b)
		{
			if (!vc.TryGetValue(a, out int ia)) { ia = verts.Count; verts.Add(a); vc[a] = ia; }
			if (!vc.TryGetValue(b, out int ib)) { ib = verts.Count; verts.Add(b); vc[b] = ib; }
			edgeSet.Add(ia < ib ? (ia, ib) : (ib, ia));
		}

		// Recursively subdivides a triangle by splitting it into an apex triangle and a base quad.
		void CollectTriangle(Vector2 ap, Vector2 ba, Vector2 bb, int depth)
		{
			if (depth == 0)
			{
				AddEdge(ap, ba);
				AddEdge(ba, bb);
				AddEdge(ap, bb);
				return;
			}

			Vector2 m1 = (ap + ba) * 0.5f;
			Vector2 m2 = (ap + bb) * 0.5f;
			AddEdge(m1, m2);
			CollectTriangle(ap, m1, m2, depth - 1);
			CollectQuad(m1, m2, ba, bb, depth - 1);
		}

		// Recursively subdivides a quad (trapezoid) by splitting its two lateral edges.
		void CollectQuad(Vector2 topA, Vector2 topB, Vector2 botA, Vector2 botB, int depth)
		{
			if (depth == 0)
			{
				AddEdge(topA, topB); AddEdge(topB, botB);
				AddEdge(botB, botA); AddEdge(botA, topA);
				AddEdge(topA, botB); // diagonal splits the quad into two triangles
				return;
			}
			Vector2 midA = (topA + botA) * 0.5f;
			Vector2 midB = (topB + botB) * 0.5f;
			AddEdge(midA, midB);
			CollectQuad(topA, topB, midA, midB, depth - 1);
			CollectQuad(midA, midB, botA, botB, depth - 1);
		}
		#endregion

		foreach (var tri in mesh.Triangles)
		{
			var (sv0, sv1, sv2) = GetTriangleVertices(tri);
			var v2 = new[] { ToVector2(sv0), ToVector2(sv1), ToVector2(sv2) };
			var lbl = new[]
			{
				ClassifyVertex(sv0, spinePoints, boundaryPoints),
				ClassifyVertex(sv1, spinePoints, boundaryPoints),
				ClassifyVertex(sv2, spinePoints, boundaryPoints)
			};

			if (IsDegenerate(lbl[0], lbl[1], lbl[2])) continue;

			int apexIdx = -1;
			for (int i = 0; i < 3; i++)
				if (lbl[i] != lbl[(i + 1) % 3] && lbl[i] != lbl[(i + 2) % 3]) { apexIdx = i; break; }

			if (apexIdx < 0) { Debug.LogWarning($"Fan triangle {tri.ID}: could not identify apex, skipping."); continue; }

			CollectTriangle(v2[apexIdx], v2[(apexIdx + 1) % 3], v2[(apexIdx + 2) % 3], subdivisions);
		}

		var splitEdges = new HashSet<(int, int)>();
		foreach (var (ia, ib) in edgeSet)
		{
			Vector2 pa = verts[ia], pb = verts[ib];

			// Gather every vertex that lies strictly on this segment, ordered by t.
			var splits = new List<(float t, int k)>();
			for (int k = 0; k < verts.Count; k++)
			{
				if (k == ia || k == ib) continue;
				if (!IsOnSegment(verts[k], pa, pb)) continue;
				float t = Vector2.Dot(verts[k] - pa, pb - pa) / (pb - pa).sqrMagnitude;
				splits.Add((t, k));
			}

			if (splits.Count == 0)
			{
				splitEdges.Add(ia < ib ? (ia, ib) : (ib, ia));
				continue;
			}

			splits.Sort((x, y) => x.t.CompareTo(y.t));
			int prev = ia;
			foreach (var (_, k) in splits)
			{
				splitEdges.Add(prev < k ? (prev, k) : (k, prev));
				prev = k;
			}
			splitEdges.Add(prev < ib ? (prev, ib) : (ib, prev));
		}

		var poly = new Polygon();
		var boundaryTNVerts = boundaryPoints.Select(p => new Vertex(p.x, p.y) { Label = 0 }).ToList();
		poly.Add(new Contour(boundaryTNVerts));

		// Use the tight vcComparer so Triangle.NET vertices match your C# subdivision logic perfectly
		var tnVertMap = new Dictionary<Vector2, Vertex>(vcComparer);
		for (int i = 0; i < boundaryPoints.Count; i++)
			tnVertMap[boundaryPoints[i]] = boundaryTNVerts[i];

		foreach (var p in verts)
		{
			if (tnVertMap.ContainsKey(p)) continue;
			var v = new Vertex(p.x, p.y) { Label = 1 };
			tnVertMap[p] = v;
			poly.Add(v);
		}

		// Update these as well to prevent aggressive collapsing at the boundary seam
		var boundaryIndexMap = new Dictionary<Vector2, int>(vcComparer);
		for (int i = 0; i < boundaryPoints.Count; i++)
			boundaryIndexMap.TryAdd(boundaryPoints[i], i);

		var boundarySet = new HashSet<Vector2>(boundaryPoints, vcComparer);
		foreach (var (ia, ib) in splitEdges)
		{
			Vector2 pa = verts[ia], pb = verts[ib];
			bool aOnBoundary = boundarySet.Contains(pa);
			bool bOnBoundary = boundarySet.Contains(pb);

			if (aOnBoundary && bOnBoundary)
			{
				boundaryIndexMap.TryGetValue(pa, out int idxA);
				boundaryIndexMap.TryGetValue(pb, out int idxB);
				bool aFound = boundaryIndexMap.ContainsKey(pa);
				bool bFound = boundaryIndexMap.ContainsKey(pb);
				if (aFound && bFound)
				{
					bool consecutive = Mathf.Abs(idxA - idxB) == 1 ||
									  (idxA == 0 && idxB == boundaryPoints.Count - 1) ||
									  (idxB == 0 && idxA == boundaryPoints.Count - 1);

					if (consecutive) continue;
				}
			}

			poly.Add(new Segment(tnVertMap[pa], tnVertMap[pb]) { Label = 1 });
		}

		return (TriangleNet.Mesh)poly.Triangulate(meshOptions, null);
	}
	#endregion
}