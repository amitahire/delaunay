#include <cstdio>
#include <climits>
#include "assert.hh"
#include "math/math.hh"
#include "sparsegrid.hh"
#include "debugdraw.hh"

SparsePointGrid::SparsePointGrid(float gridDims, int cellsPerDim)
	: m_allPoints(0)
	, m_numPoints(0)
	, m_cells(0)
	, m_cellDim(gridDims / cellsPerDim)
	, m_cellsPerDim(cellsPerDim)
	, m_gridDim(gridDims)
	, m_minGridDim(-0.5f * gridDims)
	, m_maxGridDim(0.5f * gridDims)
{
	m_cells = new Cell*[ cellsPerDim * cellsPerDim * cellsPerDim ];
	memset(m_cells, 0, sizeof(Cell*) * cellsPerDim * cellsPerDim * cellsPerDim);
}

SparsePointGrid::~SparsePointGrid()
{
	Clear();
}

void SparsePointGrid::Clear()
{
	int zOff = 0;
	for(int z = 0; z < m_cellsPerDim; ++z)
	{	
		int yOff = 0;
		for(int y = 0; y < m_cellsPerDim; ++y)
		{
			for(int x = 0; x < m_cellsPerDim; ++x)
			{
				Cell* cell = m_cells[x + yOff + zOff];
				delete cell;
			}
			yOff += m_cellsPerDim;
		}
		zOff += m_cellsPerDim * m_cellsPerDim;
	}

	delete[] m_cells;
	delete[] m_allPoints;

	m_cells = 0;
	m_allPoints = 0;
}
	
void SparsePointGrid::ToGrid(float value, int& index)
{
	index = Clamp(int(floorf((value - m_minGridDim) / m_cellDim)), 0, m_cellsPerDim - 1);
}
	
void SparsePointGrid::ToGrid(Vec3_arg v, int &ix, int &iy, int &iz)
{
	ToGrid(v.x, ix);
	ToGrid(v.y, iy);
	ToGrid(v.z, iz);
}

void SparsePointGrid::ToGridNoClamp(float value, int& index)
{
	index = int(floorf((value - m_minGridDim) / m_cellDim));
}
	
void SparsePointGrid::ToGridNoClamp(Vec3_arg v, int &ix, int &iy, int &iz)
{
	ToGridNoClamp(v.x, ix);
	ToGridNoClamp(v.y, iy);
	ToGridNoClamp(v.z, iz);
}


void SparsePointGrid::InsertPoints(const Vec3* points, int numPoints)
{
	// helper values for insertion
	int zDimMult = m_cellsPerDim * m_cellsPerDim;
	int yDimMult = m_cellsPerDim;

	int maxIndex = m_cellsPerDim;

	int numPointsInserted = 0;
	// accumulate cells.
	int ix, iy, iz;
	for(int i = 0; i < numPoints; ++i)
	{
		ToGridNoClamp(points[i], ix, iy, iz);
		
		if(ix >= 0 && ix < maxIndex &&
			iy >= 0 && iy < maxIndex &&
			iz >= 0 && iz < maxIndex)
		{
			int cellIdx = ix + yDimMult * iy + zDimMult * iz;
			ASSERT(cellIdx >= 0 && cellIdx < (m_cellsPerDim * m_cellsPerDim * m_cellsPerDim));
			Cell* cell = m_cells[cellIdx];
			if(!cell)
			{
				m_cells[cellIdx] = cell = new Cell();
				AABB cellBounds;
				cellBounds.m_min = Vec3(ix * m_cellDim + m_minGridDim, 
						iy * m_cellDim + m_minGridDim, 
						iz * m_cellDim + m_minGridDim);
				cellBounds.m_max = cellBounds.m_min + Vec3(m_cellDim, m_cellDim, m_cellDim);
				DebugDrawAABB(cellBounds);
			}
				
			++cell->m_numPoints;
			++numPointsInserted;
		}
	}

	m_allPoints = new PointData[numPointsInserted];
	m_numPoints = numPointsInserted;
	m_pointsAABB = AABB();

	// insert points and allocate cells
	int *curCounts = new int[m_cellsPerDim * m_cellsPerDim * m_cellsPerDim];
	memset(curCounts, 0, sizeof(int) * m_cellsPerDim * m_cellsPerDim * m_cellsPerDim);
	int nextPointIndex = 0;
	PointData * allPoints = m_allPoints;

	for(int i = 0; i < numPoints; ++i)
	{
		ToGridNoClamp(points[i], ix, iy, iz);

		if(ix >= 0 && ix < maxIndex &&
			iy >= 0 && iy < maxIndex &&
			iz >= 0 && iz < maxIndex)
		{
			int cellIdx = ix + yDimMult * iy + zDimMult * iz;
			ASSERT(cellIdx >= 0 && cellIdx < (m_cellsPerDim * m_cellsPerDim * m_cellsPerDim));
			Cell* cell = m_cells[cellIdx];
			ASSERT(cell);
			if(cell)
			{
				if(cell->m_points == 0)
				{
					cell->m_points = new int[cell->m_numPoints];
					curCounts[cellIdx] = 0;
				}
				ASSERT(curCounts[cellIdx] < cell->m_numPoints);
				ASSERT(nextPointIndex < numPointsInserted);

				allPoints[nextPointIndex] = points[i];
				m_pointsAABB.Extend(points[i]);
				cell->m_points[curCounts[cellIdx]++] = nextPointIndex;
				++nextPointIndex;
			}	
		}
	}

	delete[] curCounts;
}
	
int SparsePointGrid::ClosestPointToSplit(SplitDir dir, const AABB& boundsToSplit)
{
	int zDimMult = m_cellsPerDim * m_cellsPerDim;
	int yDimMult = m_cellsPerDim;

	Vec3 middle = boundsToSplit.m_min * 0.5f + boundsToSplit.m_max * 0.5f;
	int ix, iy, iz;
	ToGrid(middle, ix, iy, iz);
	int closestPointIndex = -1;
	float closestPointDist = FLT_MAX;
	float searchDistMax = m_cellDim;
	float searchDistMin = 0.f;
	Plane splitPlane;
	MakeSplitPlane(splitPlane, dir, boundsToSplit);

	int iStartLeft[3];
	int iEndLeft[3];
	int iStartRight[3];
	int iEndRight[3];
	int *startSides[2] = { iStartLeft, iStartRight };
	int *endSides[2] = { iEndLeft, iEndRight };
	ASSERT(dir >= 0 && dir < 3);
	for(int i = 0; i < 3; ++i)
	{
		iStartLeft[i] = iStartRight[i] = 0;
		iEndLeft[i] = iEndRight[i] = -1;

		if(i != dir)
		{
			ToGrid(boundsToSplit.m_min[i], iStartLeft[i]);
			ToGrid(boundsToSplit.m_max[i], iEndLeft[i]);
		}
	}
			
	while(closestPointIndex == -1)
	{
		iStartLeft[dir] = iStartRight[dir] = 0;
		iEndLeft[dir] = iEndRight[dir] = -1;

		ToGrid(middle[dir] - searchDistMax, iStartLeft[dir]);
		ToGrid(middle[dir] - searchDistMin, iEndLeft[dir]);
		ToGrid(middle[dir] + searchDistMin, iStartRight[dir]);
		ToGrid(middle[dir] + searchDistMax, iEndRight[dir]);

		if((middle[dir] - searchDistMin < boundsToSplit.m_min[dir]) ||
				(middle[dir] + searchDistMin > boundsToSplit.m_max[dir]))
			break;

		for(int zSide = 0; zSide < 2; ++zSide)
		{
			for(int iz = startSides[zSide][2]; iz <= endSides[zSide][2]; ++iz)
			{
				for(int ySide = 0; ySide < 2; ++ySide)
				{
					for(int iy = startSides[ySide][1]; iy <= endSides[ySide][1]; ++iy)
					{
						for(int xSide = 0; xSide < 2; ++xSide)
						{
							for(int ix = startSides[xSide][0]; ix <= startSides[xSide][0]; ++ix)
							{
								int cellIdx = ix + yDimMult * iy + zDimMult * iz;
								Cell* cell = m_cells[cellIdx];
								if(cell)
									FindClosestPointInCell(cell, splitPlane, closestPointIndex, closestPointDist);
							}
						}
					}
				}
			}
		}

		searchDistMin = searchDistMax;
		searchDistMax +=m_cellDim ;
	}
	return closestPointIndex;
}
	
void SparsePointGrid::FindClosestPointInCell(const Cell* cell, 
	const Plane& plane, int& closestPointIndex, float &closestPointDist)
{
	int * points = cell->m_points;
	for(int i = 0, c = cell->m_numPoints; i < c; ++i)
	{
		float dist = DistToPlane(m_allPoints[points[i]].m_pos, plane);
		if(dist < closestPointDist)
		{
			closestPointDist = dist;
			closestPointIndex = points[i];
		}
	}
}
	
void SparsePointGrid::FindClosestPointInCellAbovePlane(const Cell* cell, 
	Vec3_arg fromPos, const Plane& plane, int& closestPointIndex, float &closestPointDistSq)
{
	int * points = cell->m_points;
	for(int i = 0, c = cell->m_numPoints; i < c; ++i)
	{
		Vec3 pos = m_allPoints[points[i]].m_pos;
		if(dot(pos, plane.m_normal) - plane.m_d > 0.f)
		{
			float distSq = magnitude_squared(pos - fromPos);
			if(distSq < closestPointDistSq)
			{
				closestPointDistSq = distSq;
				closestPointIndex = points[i];
			}
		}
	}
}

int SparsePointGrid::NearestNeighborAcrossPlane(int from, const Plane& plane)
{
	Vec3 fromPos = GetPos(from);

	float searchDistMin = 0.f;
	float searchDistMax = m_cellDim;

	int closestPointIndex = -1;
	float closestPointDistSq = FLT_MAX;

	while(closestPointIndex == -1)
	{
		if(fromPos[0] - searchDistMin < m_minGridDim || fromPos[0] + searchDistMin > m_maxGridDim) break;
		if(fromPos[1] - searchDistMin < m_minGridDim || fromPos[1] + searchDistMin > m_maxGridDim) break;
		if(fromPos[2] - searchDistMin < m_minGridDim || fromPos[2] + searchDistMin > m_maxGridDim) break;

		int iStart[3];
		int iEnd[3];
		ToGrid(fromPos - Vec3(searchDistMin, searchDistMin, searchDistMin), iStart[0], iStart[1], iStart[2]);
		ToGrid(fromPos + Vec3(searchDistMin, searchDistMin, searchDistMin), iEnd[0], iEnd[1], iEnd[2]);

		int zDimOff = iStart[2] * m_cellsPerDim * m_cellsPerDim;
		for(int iz = iStart[2]; iz <= iEnd[2]; ++iz)
		{
			int yDimOff = iStart[1] * m_cellsPerDim;
			for(int iy = iStart[1]; iy <= iEnd[1]; ++iy)
			{
				for(int ix = iStart[0]; ix <= iEnd[0]; ++ix)
				{
					int cellIdx = ix + yDimOff + zDimOff;
					ASSERT(cellIdx >= 0 && cellIdx < (m_cellsPerDim * m_cellsPerDim * m_cellsPerDim));
					Cell* cell = m_cells[cellIdx];
					if(cell)
					{
						AABB cellBounds;
						cellBounds.m_min = Vec3(ix * m_cellDim + m_minGridDim, 
								iy * m_cellDim + m_minGridDim, 
								iz * m_cellDim + m_minGridDim);
						cellBounds.m_max = cellBounds.m_min + Vec3(m_cellDim, m_cellDim, m_cellDim);
						
						if(AABBIntersectsShell(cellBounds, fromPos, searchDistMin, searchDistMax) &&
								AABBAbovePlane(cellBounds, plane))
						{	
							DebugDrawAABB(cellBounds);
							FindClosestPointInCellAbovePlane(cell, fromPos, plane, closestPointIndex, 
									closestPointDistSq);
						}
					}
				}
				yDimOff += m_cellsPerDim;
			}
			zDimOff += m_cellsPerDim * m_cellsPerDim;
		}
		
		searchDistMin = searchDistMax;
		searchDistMax +=m_cellDim ;
	}

	return closestPointIndex;	
}

int SparsePointGrid::PointWithMinCircumcircle(int v0, int v1)
{
	Vec3 v0Pos = GetPos(v0);
	Vec3 v1Pos = GetPos(v1);
	Vec3 center = 0.5f * v0Pos + 0.5f * v1Pos;

	int bestRadiusIndex = -1;
	float bestRadius = FLT_MAX;
	float bestRadiusSq = FLT_MAX;		
	float searchDistMax = magnitude(center - v0Pos);

	int iSearchedStart[3], iSearchedEnd[3];
	for(int i = 0; i < 3; ++i)
	{
		iSearchedStart[i] = INT_MAX;
		iSearchedEnd[i] = INT_MIN;
	}

	while(true)
	{
		int iStart[3];
		int iEnd[3];

		ToGrid(center - Vec3(searchDistMax, searchDistMax, searchDistMax), iStart[0], iStart[1], iStart[2]);
		ToGrid(center + Vec3(searchDistMax, searchDistMax, searchDistMax), iEnd[0], iEnd[1], iEnd[2]);

		int dim;
		for(dim = 0; dim < 3; ++dim)
		{
			if(iEnd[dim] > iSearchedEnd[dim] && iStart[dim] < iSearchedStart[dim])
				break;
		}
		if(dim == 3)
			break; // We're entirely in already searched cells, so we can quit.

		int zDimOff = iStart[2] * m_cellsPerDim * m_cellsPerDim;
		for(int iz = iStart[2]; iz <= iEnd[2]; ++iz, zDimOff += m_cellsPerDim * m_cellsPerDim)
		{
			int yDimOff = iStart[1] * m_cellsPerDim;
			for(int iy = iStart[1]; iy <= iEnd[1]; ++iy, yDimOff += m_cellsPerDim)
			{
				for(int ix = iStart[0]; ix <= iEnd[0]; ++ix)
				{
					if(ix >= iSearchedStart[0] && ix <= iSearchedEnd[0] &&
						iy >= iSearchedStart[1] && iy <= iSearchedEnd[1] &&
						iz >= iSearchedStart[2] && iz <= iSearchedEnd[2])
						continue;

					int cellIdx = ix + yDimOff + zDimOff;
					ASSERT(cellIdx >= 0 && cellIdx < (m_cellsPerDim * m_cellsPerDim * m_cellsPerDim));
					Cell* cell = m_cells[cellIdx];
					if(cell)
					{
						AABB cellBounds;
						cellBounds.m_min = Vec3(ix * m_cellDim + m_minGridDim, 
								iy * m_cellDim + m_minGridDim, 
								iz * m_cellDim + m_minGridDim);
						cellBounds.m_max = cellBounds.m_min + Vec3(m_cellDim, m_cellDim, m_cellDim);

						float distToClosestSq = DistSqAABBToPoint(cellBounds, center);
						if( distToClosestSq < bestRadiusSq)
						{	
							DebugDrawAABB(cellBounds);
							int * points = cell->m_points;
							for(int i = 0, c = cell->m_numPoints; i < c; ++i)
							{
								float radiusSq;
								Vec3 testCenter;
								Vec3 pos = m_allPoints[points[i]].m_pos;
								float distToPointSq = magnitude_squared(center - pos);
								if( distToPointSq < bestRadiusSq &&
									ComputeCircumcircle(v0Pos, v1Pos, pos, testCenter, radiusSq) && 
									radiusSq < bestRadiusSq)
								{
									bestRadiusIndex = points[i];
									bestRadiusSq = radiusSq;
									bestRadius = sqrtf(radiusSq);
									center = testCenter;
								}
							}
						}
					}
				}
			}
		}

		for(int i = 0; i < 3; ++i)
		{
			iSearchedStart[i] = iSearchedStart[i] < iStart[i] ? iSearchedStart[i] : iStart[i];
			iSearchedEnd[i] = iSearchedEnd[i] > iEnd[i] ? iSearchedEnd[i] : iEnd[i];
		}
	
		if(bestRadiusIndex == -1)
			searchDistMax += m_cellDim;
		else
			searchDistMax = bestRadius;
	}

	DebugDrawSphere(center, bestRadius, 0.3f, 0.3f, 0.5f, 0.3f);

	return bestRadiusIndex;	
}

int SparsePointGrid::PointWithMinCircumsphere(int v0, int v1, int v2)
{
	Vec3 v0Pos = GetPos(v0);
	Vec3 v1Pos = GetPos(v1);
	Vec3 v2Pos = GetPos(v2);
	Vec3 center = 0.5f * v0Pos + 0.5f * v1Pos;

	int bestRadiusIndex = -1;
	float bestRadius = FLT_MAX;
	float bestRadiusSq = FLT_MAX;		
	float searchDistMax = magnitude(center - v0Pos);

	int iSearchedStart[3], iSearchedEnd[3];
	for(int i = 0; i < 3; ++i)
	{
		iSearchedStart[i] = INT_MAX;
		iSearchedEnd[i] = INT_MIN;
	}

	while(true)
	{
		int iStart[3];
		int iEnd[3];

		ToGrid(center - Vec3(searchDistMax, searchDistMax, searchDistMax), iStart[0], iStart[1], iStart[2]);
		ToGrid(center + Vec3(searchDistMax, searchDistMax, searchDistMax), iEnd[0], iEnd[1], iEnd[2]);

		int dim;
		for(dim = 0; dim < 3; ++dim)
		{
			if(iEnd[dim] > iSearchedEnd[dim] && iStart[dim] < iSearchedStart[dim])
				break;
		}
		if(dim == 3)
			break; // We're entirely in already searched cells, so we can quit.

		int zDimOff = iStart[2] * m_cellsPerDim * m_cellsPerDim;
		for(int iz = iStart[2]; iz <= iEnd[2]; ++iz, zDimOff += m_cellsPerDim * m_cellsPerDim)
		{
			int yDimOff = iStart[1] * m_cellsPerDim;
			for(int iy = iStart[1]; iy <= iEnd[1]; ++iy, yDimOff += m_cellsPerDim)
			{
				for(int ix = iStart[0]; ix <= iEnd[0]; ++ix)
				{
					if(ix >= iSearchedStart[0] && ix <= iSearchedEnd[0] &&
						iy >= iSearchedStart[1] && iy <= iSearchedEnd[1] &&
						iz >= iSearchedStart[2] && iz <= iSearchedEnd[2])
						continue;

					int cellIdx = ix + yDimOff + zDimOff;
					ASSERT(cellIdx >= 0 && cellIdx < (m_cellsPerDim * m_cellsPerDim * m_cellsPerDim));
					Cell* cell = m_cells[cellIdx];
					if(cell)
					{
						AABB cellBounds;
						cellBounds.m_min = Vec3(ix * m_cellDim + m_minGridDim, 
								iy * m_cellDim + m_minGridDim, 
								iz * m_cellDim + m_minGridDim);
						cellBounds.m_max = cellBounds.m_min + Vec3(m_cellDim, m_cellDim, m_cellDim);

						float distToClosestSq = DistSqAABBToPoint(cellBounds, center);
						if(distToClosestSq < bestRadiusSq)
						{	
							DebugDrawAABB(cellBounds);
							int * points = cell->m_points;
							for(int i = 0, c = cell->m_numPoints; i < c; ++i)
							{
								float radiusSq;
								Vec3 testCenter;
								Vec3 pos = m_allPoints[points[i]].m_pos;
								float distToPointSq = magnitude_squared(center - pos);
								if( distToPointSq < bestRadiusSq &&
									ComputeCircumsphere(v0Pos, v1Pos, v2Pos, pos, testCenter, radiusSq) && 
									radiusSq < bestRadiusSq)
								{
									bestRadiusIndex = points[i];
									bestRadiusSq = radiusSq;
									bestRadius = sqrtf(radiusSq);
									center = testCenter;
								}
							}
						}
					}
				}
			}
		}

		for(int i = 0; i < 3; ++i)
		{
			iSearchedStart[i] = iSearchedStart[i] < iStart[i] ? iSearchedStart[i] : iStart[i];
			iSearchedEnd[i] = iSearchedEnd[i] > iEnd[i] ? iSearchedEnd[i] : iEnd[i];
		}
	
		if(bestRadiusIndex == -1)
			searchDistMax += m_cellDim;
		else 
			searchDistMax = bestRadius;
	}

	DebugDrawSphere(center, bestRadius, 0.3f, 0.3f, 0.5f, 0.3f);

	return bestRadiusIndex;	
}

Vec3 SparsePointGrid::GetPos(int pointIdx)
{
	ASSERT(pointIdx >= 0 && pointIdx < m_numPoints);
	return m_allPoints[pointIdx].m_pos;
}

