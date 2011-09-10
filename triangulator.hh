#ifndef INCLUDED_triangulator_HH
#define INCLUDED_triangulator_HH

#include <list>

class SparsePointGrid;

class Triangulator
{
	struct ActiveFace
	{
		int v0, v1, v2;
	};

	struct Tetrahedron
	{
		int v0, v1, v2, v3;
	};

	struct SplitNode
	{
		SplitNode() : m_splitDir(0) 
		{
			m_children[0] = m_children[1] = 0;
		}

		int m_splitDir;
		AABB m_bounds;
		std::list< ActiveFace > m_activeFaces;
		SplitNode* m_children[2];
	};

	SparsePointGrid* m_grid;
	std::list< SplitNode* > m_nodesTodo;
	bool m_bStarted;
	std::list< Tetrahedron > m_results;

public:
	Triangulator(SparsePointGrid* grid);
	~Triangulator();
	bool IsDone() const;
	void Step();

private:
	void StartWall(SplitNode* node);
	void ContinueWall(SplitNode* node);
};


#endif

