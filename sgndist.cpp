#include "common.hh"
#include "sgndist.hh"
#include "trisoup.hh"
#include "math/vec3.hh"
#include <climits>

static const int kVoxelGroupDim = 2;
static const int kVoxelGroupDataLen = kVoxelGroupDim * kVoxelGroupDim * kVoxelGroupDim;
static const int kSmallestAllocationDim = 16;  // Oct tree will contain cubes of this dimension.

COMPILE_ASSERT( (kSmallestAllocationDim / kVoxelGroupDim) >= 0);
COMPILE_ASSERT( (kSmallestAllocationDim % kVoxelGroupDim) == 0 );

////////////////////////////////////////////////////////////////////////////////
// cube of voxels
struct SignedDistanceField::VoxelGroup
{
	VoxelGroup()
		: m_dist()
		, m_closestTri()
		, m_votesInside()
		, m_votesOutside()
	{
		for(int i = 0; i < kVoxelGroupDataLen; ++i)
		{
			m_dist[i] = FLT_MAX;
			m_closestTri[i] = UINT_MAX;
		}
	}

	float m_dist[kVoxelGroupDataLen];
	unsigned int m_closestTri[kVoxelGroupDataLen];
	unsigned short m_votesInside[kVoxelGroupDataLen];
	unsigned short m_votesOutside[kVoxelGroupDataLen];
};

////////////////////////////////////////////////////////////////////////////////
// cube of voxel groups
struct SignedDistanceField::VoxelLeafBlock
{
	static const int kLeafDim = kSmallestAllocationDim / kVoxelGroupDim;
	static const int kLeafDataLen = kLeafDim * kLeafDim * kLeafDim;
	VoxelLeafBlock()
		: m_groups()
	{}

	VoxelGroup m_groups[kLeafDataLen];
};

////////////////////////////////////////////////////////////////////////////////
// Octree Node
struct SignedDistanceField::VoxelOctNode
{
	VoxelOctNode()
		: m_minBounds()
		, m_maxBounds()
		, m_children()
		, m_leaf(0)
	{}

	~VoxelOctNode()
	{
		delete m_leaf;
		for(int i = 0; i < 8; ++i)
			delete m_children[i];
	}

	int m_minBounds[3];
	int m_maxBounds[3];
	VoxelOctNode* m_children[8];
	VoxelLeafBlock* m_leaf;

private:
	// non-copyable
	VoxelOctNode(const VoxelOctNode&);
	VoxelOctNode& operator=(const VoxelOctNode&);
};

////////////////////////////////////////////////////////////////////////////////
SignedDistanceField::SignedDistanceField(const TriSoup& triSoup, float resolution)
	: m_triSoup(new TriSoup(triSoup))
	, m_bounds()
	, m_gridBoundsMin()
	, m_gridBoundsMax()
	, m_dims()
	, m_resolution(resolution)
	, m_invResolution(1.f / resolution)
	//, m_blocks()
	, m_root(0)
{
	for(int i = 0, c = m_triSoup->NumVertices(); i < c; ++i)
		m_bounds.Extend(m_triSoup->GetVertexPos(i));

	for(int i = 0; i < 3; ++i)
	{
		m_gridBoundsMin[i] = int(floorf(m_bounds.m_min[i] / resolution));
		m_gridBoundsMax[i] = int(floorf(m_bounds.m_max[i] / resolution));
	}

	Vec3 diff = m_bounds.m_max - m_bounds.m_min;
	for(int i = 0; i < 3; ++i)
		m_dims[i] = Max(1, int(ceilf(diff[i]/resolution)));

	m_root = new VoxelOctNode();
	for(int i = 0; i < 3; ++i)
	{
		// align voxel root bounds to kSmallestAllocationDim.
		m_root->m_minBounds[i] = int(floorf(m_gridBoundsMin[i] / kSmallestAllocationDim) * kSmallestAllocationDim);
		m_root->m_maxBounds[i] = int(ceilf(m_gridBoundsMax[i] / kSmallestAllocationDim) * kSmallestAllocationDim);
	}
	if( (m_root->m_maxBounds[0] - m_root->m_minBounds[0] <= kSmallestAllocationDim) && 
			(m_root->m_maxBounds[1] - m_root->m_minBounds[1] <= kSmallestAllocationDim) &&
			(m_root->m_maxBounds[2] - m_root->m_minBounds[2] <= kSmallestAllocationDim))
		m_root->m_leaf = new VoxelLeafBlock;

	for(int i = 0, c = triSoup.NumFaces(); i < c; ++i)
	{
		int indices[3];
		triSoup.GetFace(i, indices);
		Vec3 verts[3];
		for(int j = 0; j < 3; ++j)
			verts[j] = triSoup.GetVertexPos(indices[j]);
		ComputeTriangleDistances(i, verts);
	}
}

SignedDistanceField::~SignedDistanceField()
{
	//for(int i = 0, c = m_blocks.size(); i < c; ++i)
		//delete m_blocks[i];
	delete m_root;
}

SignedDistanceField::VoxelGroup* SignedDistanceField::FindVoxelGroup(int ix, int iy, int iz, VoxelOctNode*& cache) 
{
	ASSERT(m_root);
	VoxelOctNode* curNode = m_root;
	if(cache && ix >= cache->m_minBounds[0] && ix < cache->m_maxBounds[0] &&
			iy >= cache->m_minBounds[1] && iy < cache->m_maxBounds[1] &&
			iz >= cache->m_minBounds[2] && iz < cache->m_maxBounds[2])
	{
		curNode = cache;
	}
	else
	{
		if(ix < curNode->m_minBounds[0] || ix >= curNode->m_maxBounds[0] ||
				iy < curNode->m_minBounds[1] || iy >= curNode->m_maxBounds[1] ||
				iz < curNode->m_minBounds[2] || iz >= curNode->m_maxBounds[2])
			return 0;
	}

	VoxelGroup *grp = 0;
	while(true)
	{
		ASSERT(ix >= curNode->m_minBounds[0] && ix < curNode->m_maxBounds[0] &&
			iy >= curNode->m_minBounds[1] && iy < curNode->m_maxBounds[1] &&
			iz >= curNode->m_minBounds[2] && iz < curNode->m_maxBounds[2]);
		if(curNode->m_leaf)
		{
			cache = curNode;
			int localX = ix - curNode->m_minBounds[0];
			int localY = iy - curNode->m_minBounds[1];
			int localZ = iz - curNode->m_minBounds[2];
			ASSERT(localX < kSmallestAllocationDim);
			ASSERT(localY < kSmallestAllocationDim);
			ASSERT(localZ < kSmallestAllocationDim);
		
			localX /= kVoxelGroupDim;
			localY /= kVoxelGroupDim;
			localZ /= kVoxelGroupDim;

			grp = &curNode->m_leaf->m_groups[
				localZ * VoxelLeafBlock::kLeafDim * VoxelLeafBlock::kLeafDim +
				localY * VoxelLeafBlock::kLeafDim + 
				localX];
			break;
		}
		else
		{
			int xHalf = (((curNode->m_maxBounds[0] + curNode->m_minBounds[0]) >> 1) / kSmallestAllocationDim) * 
				kSmallestAllocationDim;
			int yHalf = (((curNode->m_maxBounds[1] + curNode->m_minBounds[1]) >> 1) / kSmallestAllocationDim) * 
				kSmallestAllocationDim;
			int zHalf = (((curNode->m_maxBounds[2] + curNode->m_minBounds[2]) >> 1) / kSmallestAllocationDim) * 
				kSmallestAllocationDim;
			int childIndex = (int(iz >= zHalf) << 2) + (int(iy >= yHalf) << 1) + int(ix >= xHalf);
			ASSERT(childIndex >= 0 && childIndex < 8);

			if(!curNode->m_children[childIndex])
			{
				VoxelOctNode* node = new VoxelOctNode;
				node->m_minBounds[0] = ix >= xHalf ? xHalf : curNode->m_minBounds[0];
				node->m_maxBounds[0] = ix >= xHalf ? curNode->m_maxBounds[0] : xHalf;
				node->m_minBounds[1] = iy >= yHalf ? yHalf : curNode->m_minBounds[1];
				node->m_maxBounds[1] = iy >= yHalf ? curNode->m_maxBounds[1] : yHalf;
				node->m_minBounds[2] = iz >= zHalf ? zHalf : curNode->m_minBounds[2];
				node->m_maxBounds[2] = iz >= zHalf ? curNode->m_maxBounds[2] : zHalf;

				if(node->m_maxBounds[0] - node->m_minBounds[0] <= kSmallestAllocationDim &&
					node->m_maxBounds[1] - node->m_minBounds[1] <= kSmallestAllocationDim &&
					node->m_maxBounds[2] - node->m_minBounds[2] <= kSmallestAllocationDim)
					node->m_leaf = new VoxelLeafBlock;
				
				curNode->m_children[childIndex] = node;
			}
			curNode = curNode->m_children[childIndex];
		}
	}
	return grp;
}

void SignedDistanceField::GridCoordsFromVec(Vec3_arg pos, int& ix, int& iy, int &iz) const
{
	float invRes = m_invResolution;
	ix = floorf(pos.x * invRes);
	iy = floorf(pos.y * invRes);
	iz = floorf(pos.z * invRes);
}

Vec3 SignedDistanceField::CellCenterFromGridCoords(int ix, int iy, int iz) const
{
	const float res = m_resolution;
	const float halfRes = res * 0.5f;
	return Vec3(float(ix) * res + halfRes,
		float(iy) * res + halfRes,
		float(iz) * res + halfRes);
}

void SignedDistanceField::ComputeTriangleDistances(int tri, const Vec3 (&verts)[3])
{
	const float resolution = m_resolution;
	const float obbSize = resolution * 10;

	// Setup planes for fast iteration.
	float w = 1/3.f;
	Vec3 center = w * verts[0] + w * verts[1] + w * verts[2];

	Vec3 diffs[3];
	float diffsSq[3];
	for(int i = 0; i < 3; ++i)
	{
		diffs[i] = verts[(i+1)%3] - verts[i];
		diffsSq[i] = magnitude_squared(diffs[i]);
	}

	// Find longest side.
	int longest = 0;
	for(int i = 1; i < 2; ++i)
		if(diffsSq[i] > diffsSq[longest])
			longest = i;

	// Compute plane and distance constraint data around the triangle
	Vec3 norm[3];
	norm[2] = normalize(cross(diffs[0], -diffs[2]));
	norm[1] = normalize(diffs[longest]);
	norm[0] = normalize(cross(norm[1], norm[2]));

	float distances[3];
	distances[0] = obbSize + dot(norm[0], verts[longest] - center);
	distances[1] = obbSize + dot(norm[1], verts[(longest+1)%3] - center);
	distances[2] = obbSize;

	// Determine what axis aligned bounds to iterate over
	AABB bounds;
	bounds.Extend(center + norm[0] * distances[0] + norm[1] * distances[1] + norm[2] * distances[2]);
	bounds.Extend(center - norm[0] * distances[0] - norm[1] * distances[1] - norm[2] * distances[2]);

	int ix, iy, iz;
	GridCoordsFromVec(bounds.m_min, ix, iy, iz);
	ix = (ix / kVoxelGroupDim) * kVoxelGroupDim;
	iy = (iy / kVoxelGroupDim) * kVoxelGroupDim;
	iz = (iz / kVoxelGroupDim) * kVoxelGroupDim;

	int endX, endY, endZ;
	GridCoordsFromVec(bounds.m_max, endX, endY, endZ);

	Vec3 gridPosStart = CellCenterFromGridCoords(ix, iy, iz);
	float planeStart[3];
	float planeIncX[3]; // value to add when incrementing along X to plane i
	float planeIncY[3]; 
	float planeIncZ[3]; 
	for(int i = 0; i < 3; ++i)
	{
		planeStart[i] = dot(gridPosStart, norm[i]) - dot(norm[i], center);
		planeIncX[i] = resolution * norm[i].x;
		planeIncY[i] = resolution * norm[i].y;
		planeIncZ[i] = resolution * norm[i].z;
	}

	// And start the compute.
	float curPlanes[3];

	VoxelOctNode* cache = 0;
	for(int curZ = iz; curZ <= endZ; curZ += kVoxelGroupDim)
	{
		int offZ = curZ - iz;
		for(int curY = iy; curY <= endY; curY += kVoxelGroupDim)
		{
			int offY = curY - iy;
			for(int i = 0; i < 3; ++i)
				curPlanes[i] = planeStart[i] + offZ * planeIncZ[i] + offY * planeIncY[i];
		
			for(int curX = ix; curX <= endX; curX += kVoxelGroupDim)
			{
				VoxelGroup* grp = FindVoxelGroup(ix, iy, iz, cache);
				if(grp)
				{
					SplatVoxelGroup(
							grp,
							ix, iy, iz, tri, 
							verts, diffs,  
							curPlanes, 
							planeIncX,
							planeIncY,
							planeIncZ,
							distances);
				}
				for(int i = 0; i < 3; ++i)
					curPlanes[i] += kVoxelGroupDim * planeIncX[i];
			}
		}
	}
}

void SignedDistanceField::SplatVoxelGroup(
		VoxelGroup *grp,
		int ix, int iy, int iz, int tri,
		const Vec3 (&verts)[3], 
		const Vec3 (&diff)[3],
		const float (&curPlanes)[3], 
		const float (&planeIncX)[3], 
		const float (&planeIncY)[3], 
		const float (&planeIncZ)[3],
		const float (&distances)[3])
{
	ASSERT(grp);
	Vec3 ab = diff[0];
	Vec3 ac = -diff[2];
	Vec3 gridPosStart = CellCenterFromGridCoords(ix, iy, iz);
	int voxelIndex = 0;
	for(int z = 0; z < kVoxelGroupDim; ++z)
	{
		for(int y = 0; y < kVoxelGroupDim; ++y)
		{
			float plane0 = curPlanes[0] + planeIncZ[0] * z + planeIncY[0] * y;
			float plane1 = curPlanes[1] + planeIncZ[1] * z + planeIncY[1] * y;
			float plane2 = curPlanes[2] + planeIncZ[2] * z + planeIncY[2] * y;
			Vec3 gridPos = gridPosStart;
			gridPos.z += z * m_resolution;
			gridPos.y += y * m_resolution;
			for(int x = 0; x < kVoxelGroupDim; ++x)
			{
				float dist = FLT_MAX;
				// this could turn into a write mask
				if(fabs(plane0) < distances[0] &&
					fabs(plane1) < distances[1] &&
					fabs(plane2) < distances[2])
				{
					// detect if point is in face voronoi region of triangle or not
					//  if it is, use the plane value to set the distance & sign.
					//  if it is in another voronoi region, compute the nearest point
					//  and use that.
					//  (Derivation of this on p136 of RT Collision Det. book)
					// n = Cross(ab, ac);
					// va = Dot(n, Cross(b-p, c-p))
					// vb = Dot(n, Cross(c-p, a-p))
					// vc = Dot(n, Cross(a-p, b-p))
					// and using Lagrange identity:
					//  dot(cross(u,v), cross(s, t)) = dot(u,s) * dot(v, t) - dot(v, s) * dot(u, t)
					Vec3 ap = gridPos - verts[0];
					float d1 = dot(ab, ap);
					float d2 = dot(ac, ap);

					// This is enough to determine if we're in the VR of a vertex. 
					//  because the parameterized point along each edge is: 
					//   fromA = dot((p - start), (end - start))
					//   fromB = dot((p - end), (start - end))
					//    t = fromA / (fromA + fromB)
					//  if start is a and end is b, then fromA is dot(ap, ab) or d1
					//  and fromB is dot(bp, ba) or -d3 below.

					do 
					{
						if(d1 <= 0.0f && d2 <= 0.0f)  // barycentric coordinates (1, 0, 0)
						{
							dist = magnitude(verts[0] - gridPos);
							break;
						}

						Vec3 bp = gridPos - verts[1];
						float d3 = dot(ab, bp);
						float d4 = dot(ac, bp);

						if(d3 >= 0.f && d4 <= d3) // barycentric coordinates (0, 1, 0)
						{
							dist = magnitude(verts[1] - gridPos);
							break;
						}
						float vc = d1*d4 - d3*d2;
						if(vc <= 0.f && d1 >= 0.f && d3 <= 0.f) // barycentric coordinates (1-v, v, 0)
						{
							float v = d1 / (d1 - d3); 
							dist = magnitude(verts[0] + v * ab - gridPos);  
							break;
						}

						Vec3 cp = gridPos - verts[2];
						float d5 = dot(ab, cp);
						float d6 = dot(ac, cp);

						if(d6 >= 0.f && d5 <= d6) // barycentric coordinates (0, 0, 1)
						{
							dist = magnitude(verts[2] - gridPos);
							break;
						}
						
						float vb = d5*d2 - d1*d6;
						if(vb <= 0.f && d2 >= 0.f && d6 <= 0.f) // barycentric coordinates (1-v, 0, v)
						{
							float v = d2 / (d2 - d6);
							dist = magnitude(verts[0] + v * ac - gridPos);
							break;
						}

						float va = d3*d6 - d5*d4;
						if(va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) // barycentric coordinates (0, v, 1-v)
						{
							float v = (d4 - d3) / (d4 - d3 + d5 - d6);
							dist = magnitude(verts[1] + v * diff[1] - gridPos);
							break;
						}

						// Outside of this context, we could find the barycentric coordinates of each
						//  component to find the exact point in the triangle. We only care about
						//  distance for this though, so at this point it's safe to just use the plane2 
						//  absolute distance.

						dist = fabs(plane2);
					} while(false); // uhm, disassemble this sometime and find out how bad it is
				}

				if(dist < grp->m_dist[voxelIndex])
				{
					grp->m_dist[voxelIndex] = dist;
					grp->m_closestTri[voxelIndex] = tri;
				}
				// This should really have an intersection test too. We don't want to vote for cells
				// that have collisions between the gridPos and the closest Point on the triangle.
				if(plane2 > 0.f) // again, mask
					++grp->m_votesOutside[voxelIndex];
				else
					++grp->m_votesInside[voxelIndex];

				plane0 += planeIncX[0];
				plane1 += planeIncX[1];
				plane2 += planeIncX[2];
				gridPos.x += m_resolution;
				++voxelIndex;
			}
		}
	}

}
