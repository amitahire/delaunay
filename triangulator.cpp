#include <cstdio>
#include "math/math.hh"
#include "triangulator.hh"
#include "sparsegrid.hh"
#include "debugdraw.hh"

Triangulator::Triangulator(SparsePointGrid *grid)
	: m_grid(grid)
	, m_bStarted(false)
{
}

Triangulator::~Triangulator()
{
	for(std::list<SplitNode*>::iterator iter = m_nodesTodo.begin(),
		end = m_nodesTodo.end(); iter != end; ++iter)
	{
		delete *iter;
	}
}

bool Triangulator::IsDone() const
{
	return m_bStarted && m_nodesTodo.empty();
}

void Triangulator::Step()
{
	ASSERT(m_grid);
	if(m_bStarted == false)
	{
		m_bStarted = true;
		SplitNode* node = new SplitNode(0, m_grid->GetAllPointsAABB());
		m_nodesTodo.push_front(node);
		StartWall(node);
	}
	else
	{
		SplitNode* curNode = 0;
		while(!m_nodesTodo.empty())
		{
			curNode = m_nodesTodo.front();
			if(curNode->m_activeFaces.Empty())
			{
				m_nodesTodo.pop_front();
				delete curNode;
				curNode = 0;
			}
			else
				break;
		}

		if(curNode)
		{
			ContinueWall(curNode);
		}
	}
	
}
	
void Triangulator::StartWall(SplitNode* node)
{
	DebugDrawAABB(node->m_bounds);
	// Make the first tetrahedron
	int v0 = m_grid->ClosestPointToSplit(SparsePointGrid::SplitDir(node->m_splitDir), node->m_bounds);
	if(v0 == -1)
		return;
	DebugDrawPoint(m_grid->GetPos(v0), 1.f, 0.f, 0.f);

	Plane splitPlane;
	MakeSplitPlane(splitPlane, node->m_splitDir, node->m_bounds);
	if( dot(splitPlane.m_normal, m_grid->GetPos(v0)) - splitPlane.m_d > 0 )
	{
		splitPlane.m_normal = -splitPlane.m_normal;
		splitPlane.m_d = -splitPlane.m_d;
	}

	DebugDrawPlane(node->m_bounds, splitPlane);
	int v1 = m_grid->NearestNeighborAcrossPlane(v0, splitPlane);
	if(v1 == -1)
		return;
	DebugDrawPoint(m_grid->GetPos(v1), 1.f, 0.f, 0.f);

#ifdef DEBUG
	float d0 = dot(splitPlane.m_normal, m_grid->GetPos(v0)) - splitPlane.m_d;
	float d1 = dot(splitPlane.m_normal, m_grid->GetPos(v1)) - splitPlane.m_d;
	ASSERT( (d0 < 0.f) != (d1 < 0.f) );
#endif

	int v2 = m_grid->PointWithMinCircumcircle(v0, v1);
	if(v2 == -1)
		return;
	DebugDrawPoint(m_grid->GetPos(v2), 0.f, 1.f, 0.f);

	int v3 = m_grid->PointWithMinCircumsphere(v0, v1, v2);
	if(v3 == -1)
		return;
	DebugDrawPoint(m_grid->GetPos(v3), 1.f, 0.f, 1.f);

	AddSimplex(node, splitPlane, v0, v1, v2, v3 );

	// find center, 
	// orient so faces are CCW, 
	// make prim
	// add faces to appropriate active face lists
	
}
	
void Triangulator::AddSimplex(SplitNode* node, const Plane& plane, int v0, int v1, int v2, int v3)
{
	Vec3 v0Pos = m_grid->GetPos(v0);
	Vec3 v1Pos = m_grid->GetPos(v1);
	Vec3 v2Pos = m_grid->GetPos(v2);
	Vec3 v3Pos = m_grid->GetPos(v3);
	// Sort the first 3 vertices so they are counter clockwise from the outside of the tetrahedron
	Vec3 middle = 0.25f * v0Pos + 0.25f * v1Pos + 0.25f * v2Pos + 0.25f * v3Pos;
	Vec3 v0v1 = v1Pos - v0Pos;
	Vec3 v0v2 = v2Pos - v0Pos;
	Vec3 normal = cross(v0v1, v0v2);
	// if the middle is "above" the plane, then we need to switch the order of v1 and v2.
	if(dot(normal, middle) > dot(normal, v0Pos))
	{
		int temp = v1;
		v1 = v2;
		v2 = temp;

		Vec3 tempPos = v1Pos;
		v1Pos = v2Pos;
		v2Pos = tempPos;

		normal = -normal;
	}

	m_results.push_back(Tetrahedron(v0,v1,v2,v3));
	Vec3 triangleData[3*4] =
	{
		v0Pos, v1Pos, v2Pos,
		v1Pos, v0Pos, v3Pos,
		v0Pos, v2Pos, v3Pos,
		v1Pos, v3Pos, v2Pos,
	};

	int triangleIndices[3*4] =
	{
		v0, v1, v2,
		v1, v0, v3,
		v0, v2, v3,
		v1, v3, v2
	};

	int results[4];
	PlaneIntersectsTriangleList(plane, 4, triangleData, results);

	for(int i = 0; i < 4; ++i)
	{
		int *indices = &triangleIndices[3*i];
		if(results[i] == TRI_PLANE_INTERSECT_ALL_ABOVE)
		{
			if(node->m_children[0] == 0)
			{
				AABB bounds = node->m_bounds;
				bounds.m_min[node->m_splitDir] = 0.5f * bounds.m_min[node->m_splitDir]
					+ 0.5f * bounds.m_max[node->m_splitDir];
				node->m_children[0] = new SplitNode((node->m_splitDir + 1) % 3, bounds);
			}
			node->m_children[0]->m_activeFaces.AddOrRemove(indices[0], indices[1], indices[2]);
		}
		else if(results[i] == TRI_PLANE_INTERSECT_ALL_BELOW)
		{
			if(node->m_children[1] == 0)
			{
				AABB bounds = node->m_bounds;
				bounds.m_max[node->m_splitDir] = 0.5f * bounds.m_min[node->m_splitDir]
					+ 0.5f * bounds.m_max[node->m_splitDir];
				node->m_children[0] = new SplitNode((node->m_splitDir + 1) % 3, bounds);
			}
			node->m_children[1]->m_activeFaces.AddOrRemove(indices[0], indices[1], indices[2]);
		} 
		else
		{
			node->m_activeFaces.AddOrRemove(indices[0], indices[1], indices[2]);
		}
	}
}

void Triangulator::ContinueWall(SplitNode* node)
{
}

////////////////////////////////////////////////////////////////////////////////
// TriangleHashList

TriangleHashList::TriangleHashList(int numBuckets)
	: m_buckets(numBuckets, static_cast<HashValue*>(0))
	, m_lastInserted(0)
{
	memset(&m_stats, 0, sizeof(m_stats));
}

TriangleHashList::~TriangleHashList()
{
	for(int i = 0, c = m_buckets.size(); i < c; ++i)
	{	
		HashValue* value = m_buckets[i];
		if(value)
		{
			HashValue* next = value->next;
			delete value;
			value = next;
		}
	}
}

static int MakeHash(int v0, int v1, int v2)
{
	// Totally arbitrary primes. Should probably verify that this is half-decent.
	int hash = (31 * v0) ^ (3023 * v1) ^ (1783 * v2);
	return hash;
}

void TriangleHashList::AddOrRemove(int v0, int v1, int v2)
{
	int sorted[3];
	sorted[0] = v0;
	sorted[1] = v1;
	sorted[2] = v2;

	if(sorted[1] < sorted[0])
		Swap(sorted[1], sorted[0]);
	if(sorted[2] < sorted[0])
		Swap(sorted[2], sorted[0]);
	if(sorted[1] < sorted[2])
		Swap(sorted[1], sorted[2]);

	ASSERT(m_lastInserted == 0 || m_lastInserted->prevInserted == 0);

	int hash = MakeHash(sorted[0], sorted[1], sorted[2]);
	int idx = hash % m_buckets.size();

	if(RemoveFromBuckets(idx, sorted[0], sorted[1], sorted[2])) 
		return;

	HashValue *value = new HashValue;
	value->v0 = sorted[0];
	value->v1 = sorted[1];
	value->v2 = sorted[2];
	value->nextInserted = m_lastInserted;
	if(m_lastInserted)
		m_lastInserted->prevInserted = value;
	m_lastInserted = value;
	++m_stats.numInsertions;
	if(m_buckets[idx])
	{
		++m_stats.numCollisions;
		value->next = m_buckets[idx];
	}
	m_buckets[idx] = value;
}

bool TriangleHashList::GetNext(int *verts)
{
	if(m_lastInserted)
	{
		HashValue *result = m_lastInserted;
		verts[0] = result->v0;
		verts[1] = result->v1;
		verts[2] = result->v2;
		RemoveFromBuckets(result->v0, result->v1, result->v2);
		return true;	
	}
	else return false;
}

bool TriangleHashList::RemoveFromBuckets(int v0, int v1, int v2)
{
	int hash = MakeHash(v0, v1, v2);
	int idx = hash % m_buckets.size();
	return RemoveFromBuckets(idx, v0, v1, v2);
}

bool TriangleHashList::RemoveFromBuckets(int idx, int v0, int v1, int v2)
{
	HashValue* value = m_buckets[idx];
	HashValue* last = 0;
	
	while(value)
	{
		if(value->v0 == v0 &&
			value->v1 == v1 &&
			value->v2 == v2)
		{
			if(last) {
				last->next = value->next;
			} else {
				m_buckets[idx] = value->next;
			}

			if(value->prevInserted) value->prevInserted->nextInserted = value->nextInserted;
			if(value->nextInserted) value->nextInserted->prevInserted = value->prevInserted;

			if(value == m_lastInserted)
			{
				m_lastInserted = value->nextInserted;
				ASSERT(m_lastInserted == 0 || m_lastInserted->prevInserted == 0);
			}

			delete value;
			return true;
		}

		last = value;
		value = value->next;
	}
	return false;
}

