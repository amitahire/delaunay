#include "common.hh"
#include "surfconst.hh"
#include "trisoup.hh"
#include "debugdraw.hh"

static const unsigned int g_edgeTable[16] =
{
	0x00, // 0000
	0x0D, // 0001
	0x13, // 0010
	0x1E, // 0011
	0x26, // 0100
	0x2B, // 0101
	0x35, // 0110
	0x38, // 0111
	0x38, // 1000
	0x35, // 1001
	0x2B, // 1010
	0x26, // 1011
	0x1E, // 1100
	0x13, // 1101
	0x0D, // 1110
	0x00  // 1111
};

static const int g_triTable[16][6] =
{
	{ -1, -1, -1, -1, -1, -1 }, // 0000
	{ 0, 3, 2, -1, -1, -1 }, 	// 0001
	{ 0, 1, 4, -1, -1, -1 }, 	// 0010
	{ 1, 4, 3, 1, 3, 2 },		// 0011 
	{ 1, 2, 5, -1, -1, -1 }, 	// 0100
	{ 1, 3, 5, 1, 0, 3 }, 		// 0101
	{ 2, 5, 4, 2, 4, 0 }, 		// 0110
	{ 3, 5, 4, -1, -1, -1 },	// 0111
	{ 3, 4, 5, -1, -1, -1 },	// 1000
	{ 2, 4, 5, 2, 0, 4 }, 		// 1001
	{ 1, 5, 3, 1, 3, 0 }, 		// 1010
	{ 1, 5, 2, -1, -1, -1 }, 	// 1011
	{ 1, 3, 4, 1, 2, 3 },		// 1100 
	{ 0, 4, 1, -1, -1, -1 }, 	// 1101
	{ 0, 2, 3, -1, -1, -1 }, 	// 1110
	{ -1, -1, -1, -1, -1, -1 }, // 1111
};

inline Vec3 InterpPoints(Vec3_arg v0, Vec3_arg v1, float d0, float d1)
{
	float t = -d0 / (d1 - d0);
#ifdef DEBUG
	DebugDrawPoint(v0 + t * (v1 - v0), 0, 0, 1);
	DebugDrawLine(v0, v1, 1, 1, 0, 1);
#endif
	return v0 + t * (v1 - v0);
}

inline int MakeTableIndex(float (&distances)[4])
{
	int index = 0;
	if(distances[0] < 0.f) index |= 1;
	if(distances[1] < 0.f) index |= 2;
	if(distances[2] < 0.f) index |= 4;
	if(distances[3] < 0.f) index |= 8;
	return index;
}

////////////////////////////////////////////////////////////////////////////////
TetrahedronMarcher::TetrahedronMarcher(SignedDistanceField * distField,
	const AABB &bounds,
	float resolution)
	: m_resultMesh(0)
	, m_distField(distField)
	, m_sampler(distField->GetSampler())
	, m_bounds(bounds)
	, m_cellWidth(resolution)
	, m_curCenter()
{
}

void TetrahedronMarcher::Create()
{
	if(m_resultMesh == 0)
	{
		m_resultMesh = new TriSoup();
	}

	Vec3 center = m_bounds.m_min;
	while(AABBContains(m_bounds, center))
	{
		CreateFaces(center);

		if(center[0] < m_bounds.m_max[0] - m_cellWidth)
 		{
			center[0] += m_cellWidth;
		}
		else
		{
			center[0] = m_bounds.m_min[0];
			if(center[1]  < m_bounds.m_max[1] - m_cellWidth)
			{
				center[1] += m_cellWidth;
			}
			else
			{
				center[1] = m_bounds.m_min[1];
				if(center[2] < m_bounds.m_max[2] - m_cellWidth)
				{
					center[2] += m_cellWidth;
				}
				else 
					break;
			}
		}
	}
}

bool TetrahedronMarcher::Step()
{
	if(m_resultMesh == 0)
	{
		m_resultMesh = new TriSoup();
		m_curCenter = m_bounds.m_min;
	}
	int numCreated = 0;
	while( numCreated < 1)
	{
		numCreated = CreateFaces(m_curCenter);

		if(m_curCenter[0] < m_bounds.m_max[0] - m_cellWidth)
		{
			m_curCenter[0] += m_cellWidth;
		}
		else
		{
			m_curCenter[0] = m_bounds.m_min[0];
			if(m_curCenter[1]  < m_bounds.m_max[1] - m_cellWidth)
			{
				m_curCenter[1] += m_cellWidth;
			}
			else
			{
				m_curCenter[1] = m_bounds.m_min[1];
				if(m_curCenter[2] < m_bounds.m_max[2] - m_cellWidth)
				{
					m_curCenter[2] += m_cellWidth;
				}
				else return false;
			}
		}
	}
	return AABBContains(m_bounds, m_curCenter);
}

const TriSoup* TetrahedronMarcher::GetMesh() const
{
	return m_resultMesh;
}

void TetrahedronMarcher::AcquireMesh(ScopedPtr<TriSoup>::Type &result)
{
	result = m_resultMesh;
}

int TetrahedronMarcher::CreateFaces(Vec3_arg start)
{	
	Vec3 pt[8];
	const float w = m_cellWidth;
	pt[0] = start;
	pt[1] = pt[0] + Vec3(w, 0.f, 0.f);
	pt[2] = pt[0] + Vec3(w, w, 0.f);
	pt[3] = pt[0] + Vec3(0.f, w, 0.f);
	pt[4] = pt[0] + Vec3(0.f, 0.f, w);
	pt[5] = pt[0] + Vec3(w, 0.f, w);
	pt[6] = pt[0] + Vec3(w, w, w);
	pt[7] = pt[0] + Vec3(0.f, w, w);

	float distances[8];
	float smallest = FLT_MAX, largest = -FLT_MAX;
	for(int i = 0; i < 8; ++i)
	{
		float dist = m_sampler.GetDistance(pt[i]);
		if(dist != FLT_MAX)
		{
			smallest = Min(smallest, dist);
			largest = Max(largest, dist);
		}
		distances[i] = dist;
	}

	// TODO
	// this part is questionable - same data will look different from different
	//  approaches
	if(smallest <= 0.f && largest <= 0.f)
	{
		for(int i = 0; i < 8; ++i)
			if(distances[i] == FLT_MAX)
				distances[i] = -m_distField->GetNarrowBandWidth();
	}
	else if(smallest > 0.f && largest > 0.f)
	{
		for(int i = 0; i < 8; ++i)
			if(distances[i] == FLT_MAX)
				distances[i] = m_distField->GetNarrowBandWidth();
	}

#ifdef DEBUG
	for(int i = 0; i < 8; ++i)
		DebugDrawPoint(pt[i], distances[i] > 0.f ? 1.f : 0.f,
			distances[i] <= 0.f ? 1.f : 0.f, 
			0.f);
#endif

	int numCreated = 0;
	numCreated += CreateTetrahedronTris(pt, distances, 0, 3, 1, 5); 
	numCreated += CreateTetrahedronTris(pt, distances, 1, 3, 2, 5); 
	numCreated += CreateTetrahedronTris(pt, distances, 4, 5, 7, 3); 
	numCreated += CreateTetrahedronTris(pt, distances, 5, 6, 7, 3); 
	numCreated += CreateTetrahedronTris(pt, distances, 0, 5, 4, 3);
	numCreated += CreateTetrahedronTris(pt, distances, 5, 2, 6, 3);
	return numCreated;
}

int TetrahedronMarcher::CreateTetrahedronTris(
	Vec3 (&pt)[8],
	float (&allDistances)[8],
	int v0, int v1, int v2, int v3)
{
	ASSERT(m_resultMesh);
	float samples[4] = { allDistances[v0],
		allDistances[v1], allDistances[v2], allDistances[v3] };
	int tetIndex = MakeTableIndex(samples);

	int edges = g_edgeTable[tetIndex];

	int edgeVerts[6];
	if(edges & 1)
		edgeVerts[0] = LookupVertex(InterpPoints(pt[v0], pt[v1], allDistances[v0], allDistances[v1]));
	if(edges & 2)
		edgeVerts[1] = LookupVertex(InterpPoints(pt[v1], pt[v2], allDistances[v1], allDistances[v2]));
	if(edges & 4)
		edgeVerts[2] = LookupVertex(InterpPoints(pt[v2], pt[v0], allDistances[v2], allDistances[v0]));
	if(edges & 8)
		edgeVerts[3] = LookupVertex(InterpPoints(pt[v0], pt[v3], allDistances[v0], allDistances[v3]));
	if(edges & 16)
		edgeVerts[4] = LookupVertex(InterpPoints(pt[v1], pt[v3], allDistances[v1], allDistances[v3]));
	if(edges & 32)
		edgeVerts[5] = LookupVertex(InterpPoints(pt[v2], pt[v3], allDistances[v2], allDistances[v3]));

	int numCreated = 0;
	const int* triTable = g_triTable[tetIndex];
	for(int i = 0; i < 6 && triTable[i] >= 0; i+=3)
	{
		m_resultMesh->AddFace(
			edgeVerts[triTable[i]],
			edgeVerts[triTable[i+1]],
			edgeVerts[triTable[i+2]]);
		++numCreated;
	}
	return numCreated;
}

int TetrahedronMarcher::LookupVertex(Vec3_arg pos)
{
	ASSERT(m_resultMesh);
	// TODO: hashgrid lookup for very close positions
	return m_resultMesh->AddVertex(pos);
}



