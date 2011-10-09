#pragma once

#include <list>
#include <vector>

class SparsePointGrid;

class TriangleHashList
{
	struct HashValue
	{
		HashValue() : v0(-1), v1(-1), v2(-1), frontFacing(false),
			next(0), nextInserted(0), prevInserted(0) {}

		int v0,v1,v2;
		bool frontFacing;
		HashValue *next;

		HashValue *nextInserted;
		HashValue *prevInserted;
	};

	std::vector< HashValue * > m_buckets;
	HashValue* m_lastInserted;

	struct StatsType
	{
		int numInsertions;
		int numCollisions;	
	} m_stats;
public:
	TriangleHashList(int numBuckets);
	~TriangleHashList();

	bool AddOrRemove(int v0, int v1, int v2);
	bool Remove(int v0, int v1, int v2);
	bool GetFront(int *verts);
	void GetAllFaces(std::vector<int>& outFaceIndices) const;	
	bool Empty() const { return m_lastInserted == 0; }

	void DebugDrawFaces(const SparsePointGrid * grid);
private:
	bool RemoveFromBuckets(int v0, int v1, int v2);
	bool RemoveFromBuckets(int idx, int v0, int v1, int v2);
};

class Triangulator
{
public:
	struct Tetrahedron
	{
		Tetrahedron(int v0, int v1, int v2, int v3)
			: v0(v0), v1(v1), v2(v2), v3(v3) {}
		int v0, v1, v2, v3;
	};
private:

	struct SplitNode
	{
		SplitNode(int dir, const AABB& bounds) 
			: m_splitDir(dir)
			, m_bounds(bounds)
			, m_activeFaces( Max(4,4*int(bounds.m_max[dir] - bounds.m_min[dir])))
		{
			m_children[0] = m_children[1] = 0;
		}

		int m_splitDir;
		AABB m_bounds;
		TriangleHashList m_activeFaces;
		SplitNode* m_children[2];
	};

	SparsePointGrid* m_grid;
	std::list< SplitNode* > m_nodesTodo;
	bool m_bStarted;
	std::vector< Tetrahedron > m_results;

public:
	Triangulator(SparsePointGrid* grid);
	~Triangulator();
	bool IsDone() const;
	void Step();

	int GetNumTetrahedrons() const { return m_results.size(); }
	const Tetrahedron& GetTetrahedron(int idx) const { return m_results[idx]; }

private:
	void StartWall(SplitNode* node);
	void ContinueWall(SplitNode* node);
	void AddSimplex(SplitNode* node, const Plane& plane, int v0, int v1, int v2, int v3);
	void InitActiveFaces(SplitNode* node);
};

