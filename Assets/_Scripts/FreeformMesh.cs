using System;
using System.Collections.Generic;
using System.Linq;
using System.Xml.Schema;
using TriangleNet.Geometry;
using TriangleNet.Topology;
using UnityEngine;
using static System.Math;

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
	private static Vector2 GetVertexPos(Triangle tri, int i)
	{
		var v = tri.GetVertex(i);
		return new Vector2((float)v.X, (float)v.Y);
	}

	private static Vector2 GetEdgeMidpoint(Triangle tri, int i)
	{
		var v1 = tri.GetVertex((i + 1) % 3);
		var v2 = tri.GetVertex((i + 2) % 3);
		return new Vector2((float)(v1.X + v2.X) * 0.5f, (float)(v1.Y + v2.Y) * 0.5f);
	}

	private static Vector2 GetTriangleCentroid(Triangle tri)
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
						terminalEnd = (prunedEdgeIdx >= 0) ? GetEdgeMidpoint(tri, prunedEdgeIdx) : GetVertexPos(tri, edgeIndex);
					}
					else
					{
						terminalEnd = GetVertexPos(tri, edgeIndex);
					}

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

		return TriangleType.Terminal; // Safety
	}

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
			v1 = new Vector2((float)shared[0].X, (float)shared[0].Y);
			v2 = new Vector2((float)shared[1].X, (float)shared[1].Y);
			return true;
		}

		v1 = Vector2.zero;
		v2 = Vector2.zero;
		return false;
	}

	private static bool SegmentsIntersect(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p4)
	{
		float d1 = Cross(p3, p4, p1);
		float d2 = Cross(p3, p4, p2);
		float d3 = Cross(p1, p2, p3);
		float d4 = Cross(p1, p2, p4);

		if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
			((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)))
			return true;

		return false;
	}

	private static float Cross(Vector2 o, Vector2 a, Vector2 b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
	private static Vector2 ToVector2(Vertex v) => new Vector2((float)v.X, (float)v.Y);

	private static int ClassifyVertex(Vertex v, List<Vector2> spinePoints, List<Vector2> boundaryPoints)
	{
		const float eps = 0.001f;
		Vector2 pos = ToVector2(v);

		foreach (var b in boundaryPoints)
			if (Vector2.Distance(pos, b) < eps) return 0;

		foreach (var s in spinePoints)
			if (Vector2.Distance(pos, s) < eps) return 1;

		return -1; // Unknown / error case
	}

	private static Vertex FindUnsharedVertex(Vertex[] verts, Vertex sharedA, Vertex sharedB)
	{
		const float eps = 0.001f;
		foreach (var v in verts)
		{
			bool matchA = Mathf.Abs((float)(v.X - sharedA.X)) < eps &&
						  Mathf.Abs((float)(v.Y - sharedA.Y)) < eps;
			bool matchB = Mathf.Abs((float)(v.X - sharedB.X)) < eps &&
						  Mathf.Abs((float)(v.Y - sharedB.Y)) < eps;
			if (!matchA && !matchB)
				return v;
		}
		return null;
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
				regionVerts.Add(GetVertexPos(terminal, i));

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

				if (next == null) break;

				// Get shared edge to make semi-circle test.
				if (!GetSharedEdge(curr, next, out Vector2 v1, out Vector2 v2)) break;

				Vector2 midpoint = (v1 + v2) * 0.5f;
				float radius = Vector2.Distance(v1, v2) * 0.5f;

				// Check every vertex gainst the semicircle.
				bool allInside = true;
				foreach (var v in regionVerts)
				{
					if (Vector2.Distance(v, midpoint) > radius + 0.001f)
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
					regionVerts.Add(GetVertexPos(curr, i));
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

	public static TriangleNet.Mesh RetriangulateMeshAroundSpine(TriangleNet.Mesh mesh, List<SpineSegment> prunedSpine, List<Vector2> boundaryPoints)
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
		List<Vector2> spinePoints = spinePointsSet.ToList();

		var allFixingPairs = new List<(Vector2 spine, Vector2 boundary)>();

		var currentPoly = BuildPolygon(boundaryPoints, spinePoints, prunedSpine, allFixingPairs);
		var currentMesh = (TriangleNet.Mesh)currentPoly.Triangulate(meshOptions, null);

		const int maxIterations = 100; // Safety
		for (int iter = 0; iter < maxIterations; iter++)
		{
			var newPair = FindOneFixingDiagonal(currentMesh, spinePoints, boundaryPoints, prunedSpine, allFixingPairs);

			if (newPair == null)
			{
				Debug.Log($"Mesh clean after {iter} fixing iteration(s).");
				break;
			}

			allFixingPairs.Add(newPair.Value);
			Debug.Log($"Iteration {iter + 1}: adding diagonal {newPair.Value.spine} → {newPair.Value.boundary}");

			var newPoly = BuildPolygon(boundaryPoints, spinePoints, prunedSpine, allFixingPairs);
			currentMesh = (TriangleNet.Mesh)newPoly.Triangulate(meshOptions, null);
		}

		var midPointPair = FindMidPointVertex(currentMesh, spinePoints, boundaryPoints, prunedSpine, allFixingPairs);
		if (midPointPair != null)
		{
			Debug.Log($"Adding final diagonal from midpoint vertex: {midPointPair.Value.spine} → {midPointPair.Value.boundary}");
			allFixingPairs.Add(midPointPair.Value);
			var newPoly = BuildPolygon(boundaryPoints, spinePoints, prunedSpine, allFixingPairs);
			currentMesh = (TriangleNet.Mesh)newPoly.Triangulate(meshOptions, null);
		}

		return currentMesh;
	}

	private static (Vector2 spine, Vector2 boundary)? FindOneFixingDiagonal(TriangleNet.Mesh mesh, List<Vector2> spinePoints, List<Vector2> boundaryPoints, List<SpineSegment> prunedSpine, List<(Vector2 spine, Vector2 boundary)> existingPairs)
	{
		foreach (var tri in mesh.Triangles)
		{
			Vertex v0 = tri.GetVertex(0);
			Vertex v1 = tri.GetVertex(1);
			Vertex v2 = tri.GetVertex(2);

			int l0 = ClassifyVertex(v0, spinePoints, boundaryPoints);
			int l1 = ClassifyVertex(v1, spinePoints, boundaryPoints);
			int l2 = ClassifyVertex(v2, spinePoints, boundaryPoints);

			bool isAllSpine = l0 == 1 && l1 == 1 && l2 == 1;
			bool isAllBoundary = l0 == 0 && l1 == 0 && l2 == 0;

			if (!isAllSpine && !isAllBoundary)
				continue;

			for (int i = 0; i < 3; i++)
			{
				ITriangle neighbor = tri.GetNeighbor(i);
				if (neighbor == null || neighbor.ID < 0)
					continue;

				Vertex n0 = neighbor.GetVertex(0);
				Vertex n1 = neighbor.GetVertex(1);
				Vertex n2 = neighbor.GetVertex(2);

				int nl0 = ClassifyVertex(n0, spinePoints, boundaryPoints);
				int nl1 = ClassifyVertex(n1, spinePoints, boundaryPoints);
				int nl2 = ClassifyVertex(n2, spinePoints, boundaryPoints);

				if ((nl0 == 1 && nl1 == 1 && nl2 == 1) ||
					(nl0 == 0 && nl1 == 0 && nl2 == 0)) continue;

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

				bool alreadyTried = existingPairs.Exists(p =>
					Vector2.Distance(p.spine, spinePos) < 0.001f &&
					Vector2.Distance(p.boundary, boundaryPos) < 0.001f);

				if (alreadyTried) continue;

				// Reject this diagonal if it crosses any existing spine segment
				if (DiagonalCrossesConstraint(spinePos, boundaryPos, prunedSpine))
					continue;

				return (spinePos, boundaryPos);
			}
		}

		return null;
	}

	private static (Vector2 spine, Vector2 boundary)? FindMidPointVertex(
		TriangleNet.Mesh mesh, List<Vector2> spinePoints, List<Vector2> boundaryPoints,
		List<SpineSegment> prunedSpine, List<(Vector2 spine, Vector2 boundary)> existingPairs)
	{
		bool IsPointOnSegment(Vector2 pt, Vector2 a, Vector2 b)
		{
			return Mathf.Abs(Vector2.Distance(a, pt) + Vector2.Distance(pt, b) - Vector2.Distance(a, b)) < 0.001f;
		}

		foreach (var tri in mesh.Triangles)
		{
			Vertex v0 = tri.GetVertex(0);
			Vertex v1 = tri.GetVertex(1);
			Vertex v2 = tri.GetVertex(2);

			Vector2 p0 = ToVector2(v0);
			Vector2 p1 = ToVector2(v1);
			Vector2 p2 = ToVector2(v2);

			// Each edge paired with the vertex opposite to it
			var edges = new (Vector2 a, Vector2 b, Vector2 opp, Vertex oppV)[]
			{
				(p0, p1, p2, v2),
				(p1, p2, p0, v0),
				(p2, p0, p1, v1),
			};

			foreach (var spinePos in spinePoints)
			{
				// Skip spine vertices that are already a corner of this triangle
				if (Vector2.Distance(spinePos, p0) < 0.001f ||
					Vector2.Distance(spinePos, p1) < 0.001f ||
					Vector2.Distance(spinePos, p2) < 0.001f)
					continue;

				foreach (var (a, b, opp, oppV) in edges)
				{
					if (!IsPointOnSegment(spinePos, a, b)) continue;

					// Find Opposite Boundary Point
					int oppLabel = ClassifyVertex(oppV, spinePoints, boundaryPoints);
					if (oppLabel != 0) continue;

					Vector2 boundaryPos = opp;

					bool alreadyTried = existingPairs.Exists(p =>
						Vector2.Distance(p.spine, spinePos) < 0.001f &&
						Vector2.Distance(p.boundary, boundaryPos) < 0.001f);
					if (alreadyTried) continue;

					if (DiagonalCrossesConstraint(spinePos, boundaryPos, prunedSpine)) continue;

					return (spinePos, boundaryPos);
				}
			}
		}

		return null;
	}

	private static bool DiagonalCrossesConstraint(Vector2 a, Vector2 b, List<SpineSegment> prunedSpine)
	{
		foreach (var spine in prunedSpine)
		{
			if (Vector2.Distance(a, spine.Start) < 0.001f || Vector2.Distance(a, spine.End) < 0.001f ||
				Vector2.Distance(b, spine.Start) < 0.001f || Vector2.Distance(b, spine.End) < 0.001f)
				continue;

			if (SegmentsIntersect(a, b, spine.Start, spine.End))
				return true;
		}

		return false;
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
				Vector2 vPos = new Vector2((float)v.X, (float)v.Y);
				if (Vector2.Distance(vPos, segment.Start) < 0.001f) start = v;
				if (Vector2.Distance(vPos, segment.End) < 0.001f) end = v;
			}
			if (start != null && end != null) poly.Add(new Segment(start, end) { Label = 1 }); 
		}

		// Fixing diagonals
		if (fixingPairs != null)
		{
			foreach (var (spinePos, boundaryPos) in fixingPairs)
			{
				Vertex sv = spineVertices.Find(v => Vector2.Distance(new Vector2((float)v.X, (float)v.Y), spinePos) < 0.001f);
				Vertex bv = boundaryVertices.Find(v => Vector2.Distance(new Vector2((float)v.X, (float)v.Y), boundaryPos) < 0.001f);

				if (sv != null && bv != null) poly.Add(new Segment(sv, bv) { Label = 1 });
				else Debug.LogWarning($"Could not resolve fixing diagonal: spine={spinePos} boundary={boundaryPos}");
			}
		}

		return poly;
	}
	#endregion
}