#include "common.hh"
#include "sgndist.hh"
#include "trisoup.hh"
#include "math/vec3.hh"
#include "debugdraw.hh"
#include <climits>

#undef SGNDIST_EXTRA_DEBUG

static const int kVoxelGroupDim = 4;
static const int kVoxelGroupDataLen = kVoxelGroupDim * kVoxelGroupDim * kVoxelGroupDim;
static const int kSmallestAllocationDim = 32;  // Oct tree will contain cubes of this dimension.

COMPILE_ASSERT( (kSmallestAllocationDim / kVoxelGroupDim) >= 0);
COMPILE_ASSERT( (kSmallestAllocationDim % kVoxelGroupDim) == 0 );

////////////////////////////////////////////////////////////////////////////////
// cube of voxels
struct SignedDistanceField::VoxelGroup
{
	VoxelGroup()
		: m_dist()
		, m_closestTri()
	{
		for(int i = 0; i < kVoxelGroupDataLen; ++i)
		{
			m_dist[i] = FLT_MAX;
			m_closestTri[i] = UINT_MAX;
		}
	}

	float m_dist[kVoxelGroupDataLen];
	unsigned int m_closestTri[kVoxelGroupDataLen];
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
		, m_parent(0)
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
	VoxelOctNode* m_parent;
	VoxelLeafBlock* m_leaf;

private:
	// non-copyable
	VoxelOctNode(const VoxelOctNode&);
	VoxelOctNode& operator=(const VoxelOctNode&);
};

////////////////////////////////////////////////////////////////////////////////
SignedDistanceField::SignedDistanceField(const TriSoup& triSoup, float resolution, float narrowBandWidth)
	: m_triSoup(new TriSoup(triSoup))
	, m_bounds()
	, m_gridBoundsMin()
	, m_gridBoundsMax()
	, m_resolution(resolution)
	, m_invResolution(1.f / resolution)
	//, m_blocks()
	, m_root(0)
	, m_narrowBandWidth(narrowBandWidth)
{
	for(int i = 0, c = m_triSoup->NumVertices(); i < c; ++i)
		m_bounds.Extend(m_triSoup->GetVertexPos(i));

	for(int i = 0; i < 3; ++i)
	{
		m_gridBoundsMin[i] = int(floorf(m_bounds.m_min[i] / resolution));
		m_gridBoundsMax[i] = int(ceilf(m_bounds.m_max[i] / resolution));
	}

	m_root = new VoxelOctNode();
	for(int i = 0; i < 3; ++i)
	{
		// align voxel root bounds to kSmallestAllocationDim.
		m_root->m_minBounds[i] = floorf(m_gridBoundsMin[i] / float(kSmallestAllocationDim)) * kSmallestAllocationDim;
		m_root->m_maxBounds[i] = floorf((m_gridBoundsMax[i] + kSmallestAllocationDim - 1) / float(kSmallestAllocationDim)) *
			kSmallestAllocationDim;
	}
	if( (m_root->m_maxBounds[0] - m_root->m_minBounds[0] <= kSmallestAllocationDim) && 
			(m_root->m_maxBounds[1] - m_root->m_minBounds[1] <= kSmallestAllocationDim) &&
			(m_root->m_maxBounds[2] - m_root->m_minBounds[2] <= kSmallestAllocationDim))
	{
		m_root->m_leaf = new VoxelLeafBlock;
	}
}

SignedDistanceField::~SignedDistanceField()
{
	//for(int i = 0, c = m_blocks.size(); i < c; ++i)
		//delete m_blocks[i];
	delete m_root;
}

void SignedDistanceField::Compute()
{
	for(int i = 0, c = m_triSoup->NumFaces(); i < c; ++i)
	{
		ComputeTri(i);
	}
}

void SignedDistanceField::ComputeTri(int i)
{
	int indices[3];
	m_triSoup->GetFace(i, indices);
	Vec3 verts[3];
	for(int j = 0; j < 3; ++j)
		verts[j] = m_triSoup->GetVertexPos(indices[j]);
	ComputeTriangleDistances(i, verts);
}

int SignedDistanceField::NumTris() const
{
	return m_triSoup->NumFaces();
}
	
SignedDistanceField::Iterator SignedDistanceField::GetFirst() 
{
	return Iterator(this, m_root);
}

SignedDistanceField::Sampler SignedDistanceField::GetSampler() 
{
	return Sampler(this);
}

SignedDistanceField::VoxelGroup* SignedDistanceField::FindVoxelGroup(int ix, int iy, int iz, VoxelOctNode*& cache, bool createIfMissing) 
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
		if(ix < m_root->m_minBounds[0] || ix >= m_root->m_maxBounds[0] ||
				iy < m_root->m_minBounds[1] || iy >= m_root->m_maxBounds[1] ||
				iz < m_root->m_minBounds[2] || iz >= m_root->m_maxBounds[2])
			return 0;
	}

	VoxelGroup *grp = 0;
	while(curNode)
	{

#ifdef SGNDIST_EXTRA_DEBUG
		{
			AABB bounds;
			bounds.Extend(CellCenterFromGridCoords(
						curNode->m_minBounds[0],
						curNode->m_minBounds[1],			
						curNode->m_minBounds[2]));
			bounds.Extend(CellCenterFromGridCoords(
						curNode->m_maxBounds[0],
						curNode->m_maxBounds[1],			
						curNode->m_maxBounds[2]));
			DebugDrawAABB(bounds);
		}
#endif

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
			int xHalf = (curNode->m_maxBounds[0] + curNode->m_minBounds[0]) >> 1;
			int yHalf = (curNode->m_maxBounds[1] + curNode->m_minBounds[1]) >> 1;
			int zHalf = (curNode->m_maxBounds[2] + curNode->m_minBounds[2]) >> 1;

			xHalf = floorf((xHalf + kSmallestAllocationDim - 1) / float(kSmallestAllocationDim)) * kSmallestAllocationDim;
			yHalf = floorf((yHalf + kSmallestAllocationDim - 1) / float(kSmallestAllocationDim)) * kSmallestAllocationDim;
			zHalf = floorf((zHalf + kSmallestAllocationDim - 1) / float(kSmallestAllocationDim)) * kSmallestAllocationDim;

			unsigned int xSide = (ix >= xHalf) ? 1 : 0;
			unsigned int ySide = (iy >= yHalf) ? 1 : 0;
			unsigned int zSide = (iz >= zHalf) ? 1 : 0;
			unsigned childIndex = (zSide << 2) + (ySide << 1) + xSide;
			ASSERT(childIndex < 8);

			if(!curNode->m_children[childIndex] && createIfMissing)
			{
				VoxelOctNode* node = new VoxelOctNode;
				node->m_parent = curNode;
				int minX[] = { curNode->m_minBounds[0], xHalf };
				int minY[] = { curNode->m_minBounds[1], yHalf };
				int minZ[] = { curNode->m_minBounds[2], zHalf };
				int maxX[] = { xHalf, curNode->m_maxBounds[0] };
				int maxY[] = { yHalf, curNode->m_maxBounds[1] };
				int maxZ[] = { zHalf, curNode->m_maxBounds[2] };
				node->m_minBounds[0] = minX[xSide];
				node->m_maxBounds[0] = maxX[xSide];
				node->m_minBounds[1] = minY[ySide];
				node->m_maxBounds[1] = maxY[ySide];
				node->m_minBounds[2] = minZ[zSide];
				node->m_maxBounds[2] = maxZ[zSide];

				ASSERT(node->m_minBounds[0] != node->m_maxBounds[0]);
				ASSERT(node->m_minBounds[1] != node->m_maxBounds[1]);
				ASSERT(node->m_minBounds[2] != node->m_maxBounds[2]);

				if(node->m_maxBounds[0] - node->m_minBounds[0] <= kSmallestAllocationDim &&
					node->m_maxBounds[1] - node->m_minBounds[1] <= kSmallestAllocationDim &&
					node->m_maxBounds[2] - node->m_minBounds[2] <= kSmallestAllocationDim)
				{

					node->m_leaf = new VoxelLeafBlock;
				}
				
				curNode->m_children[childIndex] = node;
			}
			curNode = curNode->m_children[childIndex];
		}
	}
	return grp;
}

const AABB SignedDistanceField::GetAlignedBounds() const
{
	return AABB(
		CellCenterFromGridCoords(m_gridBoundsMin[0], m_gridBoundsMin[1], m_gridBoundsMin[2]),
		CellCenterFromGridCoords(m_gridBoundsMax[0], m_gridBoundsMax[1], m_gridBoundsMax[2]));
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
	const float obbSize = m_narrowBandWidth;

	// Setup planes for iteration.
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
	OBB boundsOBB(center, norm[0] * distances[0], norm[1] * distances[1], norm[2] * distances[2]);
	AABB bounds;
	bounds.Extend(boundsOBB);

#ifdef DEBUG 
	DebugDrawOBB(boundsOBB);
	DebugDrawAABB(bounds);
#endif

	int startX, startY, startZ;
	GridCoordsFromVec(bounds.m_min, startX, startY, startZ);
	startX = (startX / kVoxelGroupDim) * kVoxelGroupDim;
	startY = (startY / kVoxelGroupDim) * kVoxelGroupDim;
	startZ = (startZ / kVoxelGroupDim) * kVoxelGroupDim;

	int endX, endY, endZ;
	GridCoordsFromVec(bounds.m_max, endX, endY, endZ);

	Vec3 gridPosStart = CellCenterFromGridCoords(startX, startY, startZ);
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
	VoxelOctNode* cache = 0;
	for(int curZ = startZ; curZ <= endZ; curZ += kVoxelGroupDim)
	{
		int offZ = curZ - startZ;
		for(int curY = startY; curY <= endY; curY += kVoxelGroupDim)
		{
			float curPlanes[3];
			int offY = curY - startY;
			for(int i = 0; i < 3; ++i)
				curPlanes[i] = planeStart[i] + offZ * planeIncZ[i] + offY * planeIncY[i];
		
			for(int curX = startX; curX <= endX; curX += kVoxelGroupDim)
			{
				VoxelGroup* grp = FindVoxelGroup(curX, curY, curZ, cache, true);
				if(grp)
				{
					SplatVoxelGroup(
							grp,
							curX, curY, curZ,
							tri, 
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
	const float resolution = m_resolution;
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
			gridPos.z += z * resolution;
			gridPos.y += y * resolution;
			for(int x = 0; x < kVoxelGroupDim; ++x)
			{
				float dist = FLT_MAX;
				Vec3 closestPt(0,0,0);
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
							closestPt = verts[0];
							break;
						}

						Vec3 bp = gridPos - verts[1];
						float d3 = dot(ab, bp);
						float d4 = dot(ac, bp);

						if(d3 >= 0.f && d4 <= d3) // barycentric coordinates (0, 1, 0)
						{
							closestPt = verts[1];
							break;
						}
						float vc = d1*d4 - d3*d2;
						if(vc <= 0.f && d1 >= 0.f && d3 <= 0.f) // barycentric coordinates (1-v, v, 0)
						{
							float v = d1 / (d1 - d3); 
							closestPt = verts[0] + v * ab;
							break;
						}

						Vec3 cp = gridPos - verts[2];
						float d5 = dot(ab, cp);
						float d6 = dot(ac, cp);

						if(d6 >= 0.f && d5 <= d6) // barycentric coordinates (0, 0, 1)
						{
							closestPt = verts[2];
							break;
						}
						
						float vb = d5*d2 - d1*d6;
						if(vb <= 0.f && d2 >= 0.f && d6 <= 0.f) // barycentric coordinates (1-v, 0, v)
						{
							float v = d2 / (d2 - d6);
							closestPt = verts[0] + v * ac;
							break;
						}

						float va = d3*d6 - d5*d4;
						if(va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) // barycentric coordinates (0, v, 1-v)
						{
							float v = (d4 - d3) / (d4 - d3 + d5 - d6);
							closestPt = verts[1] + v * diff[1];
							break;
						}

						float inv_abc = 1.f / (va + vb + vc);
						float u = va * inv_abc;
						float v = vb * inv_abc;
						float w = vc * inv_abc;
						closestPt = verts[0] * u + verts[1] * v + verts[2] * w;
					} while(false); // uhm, disassemble this sometime and find out how bad it is

					dist = magnitude(closestPt - gridPos);
					if(dist < fabs(grp->m_dist[voxelIndex]))
					{
						DebugDrawLine(gridPos, closestPt, 1.f, 1.f, 0.f, 1.f);
#ifdef SGNDIST_EXTRA_DEBUG
						DebugDrawPoint(gridPos, 1.f, 0, 0);
#endif
						grp->m_dist[voxelIndex] = dist * (plane2 / fabs(plane2));
						grp->m_closestTri[voxelIndex] = tri;
					}
				}
				plane0 += planeIncX[0];
				plane1 += planeIncX[1];
				plane2 += planeIncX[2];
				gridPos.x += resolution;
				++voxelIndex;
			}
		}
	}

}

////////////////////////////////////////////////////////////////////////////////
SignedDistanceField::Iterator::Iterator()
	: m_field(0)
	, m_node(0)
	, m_pos()
{
}

SignedDistanceField::Iterator::Iterator(SignedDistanceField* field, VoxelOctNode *top)
	: m_field(field)
	, m_node(0)
	, m_pos()
{
	m_node = FindNextLeaf(top);
	if(m_node)
	{
		for(int i = 0; i < 3; ++i)
			m_pos[i] = m_node->m_minBounds[i];
	}
}
	
SignedDistanceField::Iterator::Iterator(const SignedDistanceField::Iterator& other)
	: m_field(other.m_field)
	, m_node(other.m_node)
	, m_pos()
{
	for(int i = 0; i < 3; ++i)
		m_pos[i] = other.m_pos[i];
}

SignedDistanceField::Iterator& SignedDistanceField::Iterator::operator=(const SignedDistanceField::Iterator& other)
{
	if(this != &other)
	{
		this->m_field = other.m_field;
		this->m_node = other.m_node;
		for(int i = 0; i < 3; ++i)
			m_pos[i] = other.m_pos[i];
	}
	return *this;
}

SignedDistanceField::VoxelOctNode* SignedDistanceField::Iterator::FindNextLeaf(VoxelOctNode* top)
{
	int indexInParent = -1;
	VoxelOctNode* cur = top;
	while(cur)
	{	
		if(cur->m_leaf)
		{
			if(cur == m_node)
			{
				indexInParent = IndexInParent(cur);
				cur = cur->m_parent;
			}
			else
			{
				break;
			}
		}
		else
		{
			int i;
			for(i = indexInParent + 1; i < 8; ++i)
			{
				if(cur->m_children[i])
				{
					indexInParent = -1;
					cur = cur->m_children[i];
					break;
				}
			}

			if(i == 8)
			{
				indexInParent = IndexInParent(cur);
				cur = cur->m_parent;
			}
		}
	}
#ifdef SGNDIST_EXTRA_DEBUG
	if(cur)
	{
		AABB aabb;
		aabb.Extend(m_field->CellCenterFromGridCoords(cur->m_minBounds[0], cur->m_minBounds[1], cur->m_minBounds[2]));
		aabb.Extend(m_field->CellCenterFromGridCoords(cur->m_maxBounds[0], cur->m_maxBounds[1], cur->m_maxBounds[2]));

		DebugDrawAABB(aabb);
	}
#endif
	return cur;
}

bool SignedDistanceField::Iterator::Valid() const
{
	return m_node != 0;
}
	
bool SignedDistanceField::Iterator::NextVoxel()
{
	if(!m_node) return false;
	if(m_pos[0] < m_node->m_maxBounds[0] - 1)
	{
		++m_pos[0];
		return true;
	}
	else if(m_pos[1] < m_node->m_maxBounds[1] - 1)
	{
		m_pos[0] = m_node->m_minBounds[0];
		++m_pos[1];
		return true;
	}
	else if(m_pos[2] < m_node->m_maxBounds[2] - 1)
	{
		m_pos[0] = m_node->m_minBounds[0];
		m_pos[1] = m_node->m_minBounds[1];
		++m_pos[2];
		return true;
	}
	else return NextNode();
}

bool SignedDistanceField::Iterator::NextNode()
{
	m_node = FindNextLeaf(m_node);
	if(m_node)
	{
		for(int i = 0; i < 3; ++i)
		{
			m_pos[i] = m_node->m_minBounds[i];
		}
	}
	else
	{
		for(int i = 0; i < 3; ++i)
			m_pos[i] = 0;
	}
	return m_node != 0;
}

int SignedDistanceField::Iterator::IndexInParent(VoxelOctNode* node) const
{
	if(node->m_parent == 0)
		return -1;
	int i;
	for(i = 0; i < 8; ++i)
	{
		if(node->m_parent->m_children[i] == node)
			break;
	}
	ASSERT(i != 8);
	return i;
}
	
Vec3 SignedDistanceField::Iterator::GetCenter() const
{
	if(m_node)
	{
		ASSERT(m_node->m_leaf);
		return m_field->CellCenterFromGridCoords(m_pos[0], m_pos[1], m_pos[2]);
	}
	else
	{
		return Vec3(0,0,0);
	}
}

float SignedDistanceField::Iterator::GetDistance() const
{
	if(m_node)
	{
		ASSERT(m_pos[0] >= m_node->m_minBounds[0] && m_pos[0] < m_node->m_maxBounds[0] &&
			m_pos[1] >= m_node->m_minBounds[1] && m_pos[1] < m_node->m_maxBounds[1] &&
			m_pos[2] >= m_node->m_minBounds[2] && m_pos[2] < m_node->m_maxBounds[2]);
		
		VoxelOctNode *cache = m_node;
		VoxelGroup* grp = m_field->FindVoxelGroup(m_pos[0], m_pos[1], m_pos[2], cache, false);
		ASSERT(grp);

		int groupLocal[3];
		for(int i = 0; i < 3; ++i)
		{
			int local = m_pos[i] - m_node->m_minBounds[i];
			groupLocal[i] = local % kVoxelGroupDim;
		}

		int grpIdx = groupLocal[0] +
			kVoxelGroupDim * groupLocal[1] +
			kVoxelGroupDim * kVoxelGroupDim * groupLocal[2];
		ASSERT(grpIdx >= 0 && grpIdx < kVoxelGroupDataLen);

		if(grp->m_closestTri[grpIdx] == UINT_MAX)
			return FLT_MAX;

		float dist = grp->m_dist[grpIdx];
		return dist;
	}
	else
	{
		return FLT_MAX;
	}
}

int SignedDistanceField::Iterator::GetClosestTri() const
{
	if(m_node == 0)
		return -1;

	ASSERT(m_pos[0] >= m_node->m_minBounds[0] && m_pos[0] < m_node->m_maxBounds[0] &&
			m_pos[1] >= m_node->m_minBounds[1] && m_pos[1] < m_node->m_maxBounds[1] &&
			m_pos[2] >= m_node->m_minBounds[2] && m_pos[2] < m_node->m_maxBounds[2]);
	VoxelOctNode *cache = m_node;
	VoxelGroup* grp = m_field->FindVoxelGroup(m_pos[0], m_pos[1], m_pos[2], cache, false);
	ASSERT(grp);
	
	int groupLocal[3];
	for(int i = 0; i < 3; ++i)
	{
		int local = m_pos[i] - m_node->m_minBounds[i];
		groupLocal[i] = local % kVoxelGroupDim;
	}

	int grpIdx = groupLocal[0] +
		kVoxelGroupDataLen * groupLocal[1] +
		kVoxelGroupDataLen * kVoxelGroupDataLen * groupLocal[2];

	ASSERT(grpIdx >= 0 && grpIdx < kVoxelGroupDataLen);
	return grp->m_closestTri[grpIdx];	
}


////////////////////////////////////////////////////////////////////////////////
SignedDistanceField::Sampler::Sampler()
	: m_field(0)
	, m_cache(0)
{
}

SignedDistanceField::Sampler::Sampler(const Sampler& other)
	: m_field(other.m_field)
	, m_cache(other.m_cache)
{
}

SignedDistanceField::Sampler& SignedDistanceField::Sampler::operator=(const Sampler& other)
{
	if(this != &other)	// actually this doesn't really matter here
	{
		m_field = other.m_field;
		m_cache = other.m_cache;
	}
	return *this;
}

SignedDistanceField::Sampler::Sampler(SignedDistanceField* field)
	: m_field(field)
	, m_cache(0)
{
}

float SignedDistanceField::Sampler::GetDistance(Vec3_arg pos) 
{
	const float halfWidth = m_field->GetGridWidth() * 0.5f; 
	const float invWidth = m_field->GetInvGridWidth();
	int ix, iy, iz;
	m_field->GridCoordsFromVec(pos - Vec3(halfWidth,halfWidth,halfWidth), ix, iy, iz);
	Vec3 cellCenter = m_field->CellCenterFromGridCoords(ix,iy,iz);
	Vec3 toPos = pos - cellCenter;
	float u = toPos.x * invWidth;
	float v = toPos.y * invWidth;
	float w = toPos.z * invWidth;
	ASSERT(u >= 0.f && u <= 1.f);
	ASSERT(v >= 0.f && v <= 1.f);
	ASSERT(w >= 0.f && w <= 1.f);
	float uOther = 1.f - u;
	float vOther = 1.f - v;
	float wOther = 1.f - w;
	float weights[8] = {
		uOther * vOther * wOther,
		u * vOther * wOther,
		uOther * v * wOther,
		u * v * wOther,
		uOther * vOther * w,
		u * vOther * w,
		uOther * v * w,
		u * v * w,
	};

	float samples[8];
	float totalWeight = 0;
	int curSample = 0;
	float smallest = FLT_MAX, largest = -FLT_MAX;
	for(int z = 0; z < 2; ++z)
	{
		for(int y = 0; y < 2; ++y)
		{
			for(int x = 0; x < 2; ++x, ++curSample)
			{
				VoxelGroup *grp = m_field->FindVoxelGroup(ix + x, iy + y, iz + z, m_cache, false);
				if(grp)
				{
					ASSERT(m_cache);
					int groupLocal[3] = {
						(ix + x - m_cache->m_minBounds[0]) % kVoxelGroupDim,
						(iy + y - m_cache->m_minBounds[1]) % kVoxelGroupDim,
						(iz + z - m_cache->m_minBounds[2]) % kVoxelGroupDim,
					};

					int grpIdx = groupLocal[0] +
						kVoxelGroupDim * groupLocal[1] +
						kVoxelGroupDim * kVoxelGroupDim * groupLocal[2];
					ASSERT(grpIdx >= 0 && grpIdx < kVoxelGroupDataLen);

	//				if(grp->m_closestTri[grpIdx] != UINT_MAX)
	//				{
	//					DebugDrawPoint(m_field->CellCenterFromGridCoords(ix + x, iy + y, iz +z), 1.f, 0.f, 1.f);
						float dist = grp->m_dist[grpIdx];
						samples[curSample] = dist;
						totalWeight += weights[curSample];

						smallest = Min(dist, smallest);
						largest = Max(dist, largest);
	//				}
				}
			}
		}
	}
	
	if(smallest <= 0.f && largest <= 0.f)
	{
		for(int i = 0; i < 8; ++i)
			if(samples[i] == FLT_MAX)
				samples[i] = -m_field->GetNarrowBandWidth();
	}
	else if(smallest > 0.f && largest > 0.f)
	{
		for(int i = 0; i < 8; ++i)
			if(samples[i] == FLT_MAX)
				samples[i] = m_field->GetNarrowBandWidth();
	}

	float sampleValue = 0.f;
	for(int i = 0; i < 8; ++i)
		sampleValue += weights[i] * samples[i];

	//if(totalWeight > 0.f)
	//	sampleValue /= totalWeight;
	//else
	//{
	//	sampleValue = FLT_MAX;
	//}
	return sampleValue;
}

int SignedDistanceField::Sampler::GetClosestTri(Vec3_arg pos) 
{
	int ix, iy, iz;
	m_field->GridCoordsFromVec(pos, ix, iy, iz);
	VoxelGroup *grp = m_field->FindVoxelGroup(ix, iy, iz, m_cache, false);
	if(grp)
	{
		ASSERT(m_cache);
		int groupLocal[3] = {
			(ix - m_cache->m_minBounds[0]) % kVoxelGroupDim,
			(iy - m_cache->m_minBounds[1]) % kVoxelGroupDim,
			(iz - m_cache->m_minBounds[2]) % kVoxelGroupDim,
		};

		int grpIdx = groupLocal[0] +
			kVoxelGroupDim * groupLocal[1] +
			kVoxelGroupDim * kVoxelGroupDim * groupLocal[2];
		ASSERT(grpIdx >= 0 && grpIdx < kVoxelGroupDataLen);

		return grp->m_closestTri[grpIdx];
	}

	return -1;
}

