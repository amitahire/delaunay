#pragma once

#include "ptr.hh"
#include "sgndist.hh"

class SignedDistanceField; 
class TriSoup;

////////////////////////////////////////////////////////////////////////////////
class TetrahedronMarcher 
{
public:
	////////////////////////////////////////////////////////////////////////////////
	TetrahedronMarcher(SignedDistanceField * distField,  const AABB &bounds, float resolution);

	// Creation functions
	void Create();
	bool Step();

	// Access functions
	const TriSoup* GetMesh() const;

	void AcquireMesh(ScopedPtr<TriSoup>::Type &result);
private:
	// non-copyable
	TetrahedronMarcher(const TetrahedronMarcher&);
	TetrahedronMarcher& operator=(const TetrahedronMarcher&);

	////////////////////////////////////////////////////////////////////////////////
	int CreateFaces(Vec3_arg start);
	int CreateTetrahedronTris(Vec3 (&pt)[8],
		float (&distances)[8], int v0, int v1, int v2, int v3);
	int LookupVertex(Vec3_arg pos);

	////////////////////////////////////////////////////////////////////////////////
	ScopedPtr<TriSoup>::Type m_resultMesh;
	SignedDistanceField * m_distField;
	SignedDistanceField::Sampler m_sampler;
	AABB m_bounds;
	float m_cellWidth;
	Vec3 m_curCenter;
};

