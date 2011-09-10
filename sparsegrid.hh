#ifndef INCLUDED_sparsegrid_HH
#define INCLUDED_sparsegrid_HH

class SparsePointGrid
{
	struct PointData {
		PointData() {}
		PointData(Vec3_arg pos) : m_pos(pos) {}
		Vec3 m_pos;
	};

	struct Cell {
		Cell() : m_numPoints(0), m_points(0) {}
		~Cell() { delete[] m_points; }
		int m_numPoints;
		int * m_points;

	private:
		Cell(const Cell&);
		Cell& operator=(const Cell&);
	};

	PointData * m_allPoints;
	int m_numPoints;
	AABB m_pointsAABB;
	Cell** m_cells; 

	float m_cellDim;
	int m_cellsPerDim;
	float m_gridDim;		// total length of grid side
	float m_minGridDim;		
	float m_maxGridDim;		

public:
	enum SplitDir
	{
		SPLITDIR_X = 0,
		SPLITDIR_Y,
		SPLITDIR_Z,
	};

	SparsePointGrid(float gridDims, int cellsPerDim);
	~SparsePointGrid();

	void InsertPoints(const Vec3* points, int numPoints);
	int ClosestPointToSplit(SplitDir dir, const AABB& boundsToSplit);
	int NearestNeighborAcrossPlane(int from, const Plane& plane);
	int PointWithMinCircumcircle(int v0, int v1);
	int PointWithMinCircumsphere(int v0, int v1, int v2);
	Vec3 GetPos(int pointIdx);

	const AABB& GetAllPointsAABB() const { return m_pointsAABB; }
private:
	void Clear();

	void ToGrid(Vec3_arg v, int &ix, int &iy, int &iz);
	void ToGrid(float value, int& index);
	void ToGridNoClamp(Vec3_arg v, int &ix, int &iy, int &iz);
	void ToGridNoClamp(float value, int& index);
	void FindClosestPointInCell(const Cell* cell, const Plane& plane, int& closestPointIndex, float &closestPointDist);
	void FindClosestPointInCellAbovePlane(const Cell* cell, Vec3_arg fromPos, const Plane& plane, int& closestPointIndex, float &closestPointDistSq);
};

#endif

