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

public class TriangleFan
{
	public SpineSegment SpineSegment;
	public List<Triangle> Triangles = new List<Triangle>(); // Ordered by angle
	public List<float> Angles = new List<float>(); // Corresponding angles
	public Vector2 FanCenter; // Center point of the fan
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

	#region Pruning, Fanning, Retriangulation
	private class SpineNode
	{
		public Vector2 Position;
		public List<SpineNode> Neighbors = new List<SpineNode>();
		public bool IsTerminal = false;
		public bool IsJunction = false;
		public int VisitCount = 0; // How many segments connect to this node
	}

	private class Branch
	{
		public List<Vector2> Path = new List<Vector2>();
		public SpineNode StartNode;
		public SpineNode EndNode;
		public float Length;
		public float Significance;
		public bool IsTerminalBranch;
	}

	#region Helper Methods
	public static Dictionary<int, float> CalculateElevations(TriangleNet.Mesh mesh, List<SpineSegment> prunedSpine, List<Vector2> boundary)
	{
		Dictionary<int, float> elevations = new Dictionary<int, float>();

		foreach (var vertex in mesh.Vertices)
		{
			Vector2 pos = new Vector2((float)vertex.X, (float)vertex.Y);

			// Find closest point on spine
			Vector2 closestSpinePoint = FindClosestPointOnSpine(pos, prunedSpine, out float distToSpine);

			// Get radius at that spine point (distance to boundary)
			float spineRadius = FreeformMesh.GetDistanceToOutline(closestSpinePoint, boundary);

			// Calculate elevation using cross-section formula
			// Common approaches:
			// 1. Linear: h = R - r (simple ramp)
			// 2. Circular: h = sqrt(R² - r²) (circular cross-section)
			// 3. Elliptical: h = sqrt(1 - (r/R)²) * R (normalized)

			float r = distToSpine;
			float R = spineRadius;

			// Using circular cross-section for smooth inflation
			float elevation = 0f;
			if (R > 0.001f && r <= R)
			{
				elevation = Mathf.Sqrt(R * R - r * r);
			}

			elevations[vertex.ID] = elevation;
		}

		return elevations;
	}

	private static SpineSegment FindClosestSpineSegment(Vector2 point, List<SpineSegment> spine)
	{
		SpineSegment closest = new SpineSegment();
		float minDist = float.MaxValue;

		foreach (var segment in spine)
		{
			// Distance to segment
			Vector2 pa = point - segment.Start;
			Vector2 ba = segment.End - segment.Start;
			float h = Mathf.Clamp01(Vector2.Dot(pa, ba) / Vector2.Dot(ba, ba));
			Vector2 closestPoint = segment.Start + ba * h;

			float dist = Vector2.Distance(point, closestPoint);
			if (dist < minDist)
			{
				minDist = dist;
				closest = segment;
			}
		}

		return closest;
	}

	private static Vector2 FindClosestPointOnSpine(
		Vector2 point,
		List<SpineSegment> spine,
		out float distance)
	{
		Vector2 closest = Vector2.zero;
		float minDist = float.MaxValue;

		foreach (var segment in spine)
		{
			Vector2 pa = point - segment.Start;
			Vector2 ba = segment.End - segment.Start;
			float h = Mathf.Clamp01(Vector2.Dot(pa, ba) / Vector2.Dot(ba, ba));
			Vector2 closestPoint = segment.Start + ba * h;

			float dist = Vector2.Distance(point, closestPoint);
			if (dist < minDist)
			{
				minDist = dist;
				closest = closestPoint;
			}
		}

		distance = minDist;
		return closest;
	}

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

	private static Dictionary<Vector2, SpineNode> BuildSpineGraph(List<SpineSegment> segments)
	{
		Dictionary<Vector2, SpineNode> nodeMap = new Dictionary<Vector2, SpineNode>(new Vector2EqualityComparer());

		// Create or get node
		SpineNode GetOrCreateNode(Vector2 pos, TriangleType type)
		{
			if (!nodeMap.TryGetValue(pos, out SpineNode node))
			{
				node = new SpineNode { Position = pos };
				nodeMap[pos] = node;
			}

			// Update node properties based on triangle type
			if (type == TriangleType.Terminal)
				node.IsTerminal = true;
			else if (type == TriangleType.Junction)
				node.IsJunction = true;

			node.VisitCount++;
			return node;
		}

		// Build connectivity
		foreach (var seg in segments)
		{
			SpineNode startNode = GetOrCreateNode(seg.Start, seg.Type);
			SpineNode endNode = GetOrCreateNode(seg.End, seg.Type);

			// Add bidirectional edges
			if (!startNode.Neighbors.Contains(endNode))
				startNode.Neighbors.Add(endNode);
			if (!endNode.Neighbors.Contains(startNode))
				endNode.Neighbors.Add(startNode);
		}

		return nodeMap;
	}

	private static List<Branch> IdentifyBranches(Dictionary<Vector2, SpineNode> nodeMap)
	{
		List<Branch> branches = new List<Branch>();
		HashSet<SpineNode> visited = new HashSet<SpineNode>();

		// Find all junction and terminal nodes (branch endpoints)
		List<SpineNode> branchEndpoints = nodeMap.Values
			.Where(n => n.IsJunction || n.IsTerminal || n.Neighbors.Count != 2)
			.ToList();

		// Special case: if no junctions/terminals (simple loop), pick an arbitrary start
		if (branchEndpoints.Count == 0 && nodeMap.Count > 0)
		{
			branchEndpoints.Add(nodeMap.Values.First());
		}

		// Trace branches from each endpoint
		foreach (var startNode in branchEndpoints)
		{
			foreach (var neighbor in startNode.Neighbors)
			{
				// Trace the path until we hit another endpoint
				List<Vector2> path = new List<Vector2>();
				path.Add(startNode.Position);

				SpineNode current = neighbor;
				SpineNode previous = startNode;

				// Walk along the branch until we hit another endpoint
				while (current != null &&
					   !current.IsJunction &&
					   !current.IsTerminal &&
					   current.Neighbors.Count == 2)
				{
					path.Add(current.Position);

					// Move to next node (not the one we came from)
					SpineNode next = current.Neighbors[0] == previous
						? current.Neighbors[1]
						: current.Neighbors[0];

					previous = current;
					current = next;
				}

				// Add the endpoint
				if (current != null)
					path.Add(current.Position);

				// Only add if this is a valid branch and we haven't seen it in reverse
				if (path.Count >= 2)
				{
					float length = CalculatePathLength(path);

					Branch branch = new Branch
					{
						Path = path,
						StartNode = startNode,
						EndNode = current,
						Length = length,
						IsTerminalBranch = startNode.IsTerminal || (current != null && current.IsTerminal)
					};

					// Check if we already have the reverse of this branch
					bool isDuplicate = branches.Any(b =>
						Vector2.Distance(b.Path[0], path[path.Count - 1]) < 0.001f &&
						Vector2.Distance(b.Path[b.Path.Count - 1], path[0]) < 0.001f);

					if (!isDuplicate)
						branches.Add(branch);
				}
			}
		}

		return branches;
	}

	private static void CalculateBranchSignificance(List<Branch> branches, List<Vector2> boundary)
	{
		if (branches.Count == 0) return;

		// Find max branch length for normalization
		float maxLength = branches.Max(b => b.Length);
		if (maxLength < 0.001f) maxLength = 1f;

		foreach (var branch in branches)
		{
			// Sample points along the branch to measure local width
			float avgWidth = CalculateAverageWidth(branch.Path, boundary);

			// Significance is based on:
			// 1. Normalized length (longer branches are more significant)
			// 2. Average width (wider regions are more significant)
			// 3. Terminal branches get a penalty (they're often artifacts)

			float normalizedLength = branch.Length / maxLength;

			// Width-to-length ratio indicates how "substantial" a branch is
			float widthLengthRatio = avgWidth > 0 ? Mathf.Min(avgWidth / branch.Length, 1f) : 0f;

			// Base significance on length and width
			float significance = normalizedLength * 0.6f + widthLengthRatio * 0.4f;

			// Penalize very short terminal branches (likely noise)
			if (branch.IsTerminalBranch && normalizedLength < 0.2f)
				significance *= 0.5f;

			branch.Significance = Mathf.Clamp01(significance);
		}
	}

	private static float CalculateAverageWidth(List<Vector2> path, List<Vector2> boundary)
	{
		if (path.Count == 0) return 0f;

		float totalWidth = 0f;
		int sampleCount = 0;

		// Sample multiple points along the branch
		int samples = Mathf.Min(5, path.Count);
		for (int i = 0; i < samples; i++)
		{
			int idx = (int)(i * (path.Count - 1) / (float)(samples - 1));
			Vector2 point = path[Mathf.Clamp(idx, 0, path.Count - 1)];

			// Distance to boundary is a proxy for local width
			float dist = FreeformMesh.GetDistanceToOutline(point, boundary);
			totalWidth += dist * 2f; // Multiply by 2 for diameter
			sampleCount++;
		}

		return sampleCount > 0 ? totalWidth / sampleCount : 0f;
	}


	private static List<Branch> PruneInsignificantBranches(List<Branch> branches, float threshold)
	{
		// Keep only significant branches
		List<Branch> kept = branches.Where(b => b.Significance >= threshold).ToList();

		// Always keep at least one branch (the most significant one)
		if (kept.Count == 0 && branches.Count > 0)
		{
			kept.Add(branches.OrderByDescending(b => b.Significance).First());
		}

		return kept;
	}

	private static List<SpineSegment> ConvertBranchesToSegments(List<Branch> branches)
	{
		List<SpineSegment> segments = new List<SpineSegment>();

		foreach (var branch in branches)
		{
			// Convert each segment of the branch path
			for (int i = 0; i < branch.Path.Count - 1; i++)
			{
				TriangleType type = TriangleType.Sleeve;

				// Determine type based on position in branch
				if (i == 0 && branch.IsTerminalBranch)
					type = TriangleType.Terminal;
				else if (branch.StartNode != null && branch.StartNode.IsJunction)
					type = TriangleType.Junction;

				segments.Add(new SpineSegment
				{
					Start = branch.Path[i],
					End = branch.Path[i + 1],
					Type = type
				});
			}
		}

		return segments;
	}

	private static float CalculatePathLength(List<Vector2> path)
	{
		float length = 0f;
		for (int i = 0; i < path.Count - 1; i++)
		{
			length += Vector2.Distance(path[i], path[i + 1]);
		}
		return length;
	}

	public static List<TriangleFan> CreateTriangleFans(
		TriangleNet.Mesh mesh,
		List<SpineSegment> prunedSpine)
	{
		List<TriangleFan> fans = new List<TriangleFan>();

		// Map each spine segment to its adjacent triangles
		Dictionary<SpineSegment, List<Triangle>> segmentTriangles = new Dictionary<SpineSegment, List<Triangle>>();

		foreach (var segment in prunedSpine)
		{
			segmentTriangles[segment] = new List<Triangle>();
		}

		// For each triangle, find which spine segment it's closest to
		foreach (var tri in mesh.Triangles)
		{
			Vector2 triCenter = GetTriangleCentroid(tri);
			SpineSegment closestSegment = FindClosestSpineSegment(triCenter, prunedSpine);

			if (segmentTriangles.ContainsKey(closestSegment))
			{
				segmentTriangles[closestSegment].Add(tri);
			}
		}

		// Create fans by sorting triangles by angle around each segment
		foreach (var segment in prunedSpine)
		{
			List<Triangle> triangles = segmentTriangles[segment];
			if (triangles.Count == 0) continue;

			TriangleFan fan = new TriangleFan
			{
				SpineSegment = segment,
				FanCenter = (segment.Start + segment.End) * 0.5f
			};

			// Calculate angle for each triangle relative to spine direction
			Vector2 spineDir = (segment.End - segment.Start).normalized;
			Vector2 perpDir = new Vector2(-spineDir.y, spineDir.x); // Perpendicular

			foreach (var tri in triangles)
			{
				Vector2 triCenter = GetTriangleCentroid(tri);
				Vector2 toTri = triCenter - fan.FanCenter;

				// Calculate angle relative to perpendicular direction
				float angle = Mathf.Atan2(
					Vector2.Dot(toTri, perpDir),
					Vector2.Dot(toTri, spineDir)
				);

				fan.Triangles.Add(tri);
				fan.Angles.Add(angle);
			}

			// Sort by angle
			var sorted = fan.Triangles
				.Select((t, i) => new { Triangle = t, Angle = fan.Angles[i] })
				.OrderBy(x => x.Angle)
				.ToList();

			fan.Triangles = sorted.Select(x => x.Triangle).ToList();
			fan.Angles = sorted.Select(x => x.Angle).ToList();

			fans.Add(fan);
		}

		return fans;
	}
	#endregion

	public static List<SpineSegment> PruneBranches(List<SpineSegment> segments, List<Vector2> boundary, float significanceThreshold = 0.15f)
	{
		if (segments == null || segments.Count == 0) return new List<SpineSegment>();

		// Step 1: Build graph from segments
		Dictionary<Vector2, SpineNode> nodeMap = BuildSpineGraph(segments);

		// Step 2: Identify branches
		List<Branch> branches = IdentifyBranches(nodeMap);

		// Step 3: Calculate significance for each branch
		CalculateBranchSignificance(branches, boundary);

		// Step 4: Prune insignificant branches
		List<Branch> prunedBranches = PruneInsignificantBranches(branches, significanceThreshold);

		// Step 5: Convert back to SpineSegments
		return ConvertBranchesToSegments(prunedBranches);
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

		// Add spine points as Steiner points (internal points that must be included)
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
}
