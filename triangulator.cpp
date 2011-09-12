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
		SplitNode* node = new SplitNode;
		node->m_bounds = m_grid->GetAllPointsAABB();
		m_nodesTodo.push_front(node);
		StartWall(node);
	}
	else
	{
		SplitNode* curNode = 0;
		while(!m_nodesTodo.empty())
		{
			curNode = m_nodesTodo.front();
			if(curNode->m_activeFaces.empty())
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

	// find center, 
	// orient so faces are CCW, 
	// make prim
	// add faces to appropriate active face lists
	
}

void Triangulator::ContinueWall(SplitNode* node)
{
}

