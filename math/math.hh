#pragma once

#include "math/vec3.hh"
#include "math/mat4.hh"
#include "math/obb.hh"
#include <cfloat>

////////////////////////////////////////////////////////////////////////////////
#define EPSILON 1e-3f
#define EPSILON_SQ (EPSILON * EPSILON)

////////////////////////////////////////////////////////////////////////////////
struct Plane
{
	Plane() : m_normal(), m_d() {}
	Plane(Vec3_arg normal, float d) : m_normal(normal), m_d(d) {}
	Vec3 m_normal;
	float m_d;
};

struct DPlane
{
	DPlane() : m_normal(), m_d() {}
	DPlane(DVec3_arg normal, double d) : m_normal(normal), m_d(d) {}
	DVec3 m_normal;
	double m_d;
};

struct OBB
{
	OBB() : m_center(), m_u(), m_v(), m_w() {}
	OBB(Vec3_arg center, Vec3_arg u, Vec3_arg v, Vec3_arg w)
		: m_center(center), m_u(u), m_v(v), m_w(w) {}
	Vec3 m_center;
	Vec3 m_u;
	Vec3 m_v;
	Vec3 m_w;
};

struct AABB
{
	AABB() : m_min(FLT_MAX, FLT_MAX, FLT_MAX), m_max(-FLT_MAX, -FLT_MAX, -FLT_MAX) {}
	AABB(Vec3_arg mn, Vec3_arg mx) : m_min(mn), m_max(mx) {}
	AABB(const AABB& o) : m_min(o.m_min), m_max(o.m_max) {}
	Vec3 m_min;
	Vec3 m_max;

	void Extend(Vec3_arg v) 
	{
		m_min = VecMin(v, m_min);
		m_max = VecMax(v, m_max);
	}

	void Extend(const OBB& obb)
	{	
		Extend(obb.m_center - obb.m_u - obb.m_v - obb.m_w);
		Extend(obb.m_center + obb.m_u - obb.m_v - obb.m_w);
		Extend(obb.m_center + obb.m_u + obb.m_v - obb.m_w);
		Extend(obb.m_center - obb.m_u + obb.m_v - obb.m_w);
		Extend(obb.m_center - obb.m_u - obb.m_v + obb.m_w);
		Extend(obb.m_center + obb.m_u - obb.m_v + obb.m_w);
		Extend(obb.m_center + obb.m_u + obb.m_v + obb.m_w);
		Extend(obb.m_center - obb.m_u + obb.m_v + obb.m_w);
	}
};

////////////////////////////////////////////////////////////////////////////////
enum TriPlaneIntersectType
{
	TRI_PLANE_INTERSECT_ALL_BELOW = 0x0,
	TRI_PLANE_INTERSECT_ALL_ABOVE = 0x07
};

////////////////////////////////////////////////////////////////////////////////
float DistToPlane(Vec3_arg p, const Plane& plane);
Vec3 ClosestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt);
Vec3 FurthestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt);
float DistSqAABBToPoint(const AABB& aabb, Vec3_arg pt);
bool AABBIntersectsSphere(const AABB& aabb, Vec3_arg sphereCenter, float sphereRadius);
bool AABBIntersectsShell(const AABB& aabb, Vec3_arg sphereCenter, float minRadius, float maxRadius);
bool AABBAbovePlane(const AABB& aabb, const Plane& plane);
bool AABBAbovePlane(const AABB& aabb, const DPlane& plane);
bool AABBContains(const AABB& aabb, Vec3_arg pt);
bool AABBContains(const AABB& aabb, DVec3_arg pt);
Vec3 MakeSplitNormal(int splitdir);
void MakeSplitPlane(Plane& plane, int dir, const AABB& bounds);
bool ComputeCircumcircle(Vec3_arg a, Vec3_arg b, Vec3_arg c, Vec3& outCenter, float &outRadiusSq);
bool ComputeCircumsphere(Vec3_arg a, Vec3_arg b, Vec3_arg c, Vec3_arg d, Vec3 &outCenter, float &outRadiusSq);
bool ComputeCircumsphere(DVec3_arg a, DVec3_arg b, DVec3_arg c, DVec3_arg d, DVec3 &outCenter, double &outRadiusSq);
void PlaneIntersectsTriangleList(const Plane& plane, int numTriangles, const Vec3* triangleData, int *results);
Vec3 ProjectN(Vec3_arg vec, Vec3_arg n);

////////////////////////////////////////////////////////////////////////////////
template<class T>
inline T Clamp(T val, T min, T max)
{
	if(val < min)
		return min;
	else if(val > max)
		return max;
	else
		return val;
}

template<class T>
inline T Max(T val0, T val1)
{
	return val0 > val1 ? val0 : val1;
}

template<class T>
inline T Min(T val0, T val1)
{
	return val0 < val1 ? val0 : val1;
}

template<class T>
inline void Swap(T& val0, T& val1)
{
	T temp = val0;
	val0 = val1;
	val1 = temp;
}

inline float AngleWrap(float angle)
{
	while(angle < 0.f)
		angle += 2.f * M_PI;
	while(angle > 2.f * M_PI)
		angle -= 2.f * M_PI;
	return angle;
}


