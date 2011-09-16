#ifndef INCLUDED_math_allmath_HH
#define INCLUDED_math_allmath_HH

#include "math/vec3.hh"
#include "math/mat4.hh"
#include "math/aabb.hh"

#define EPSILON 1e-3f
#define EPSILON_SQ (EPSILON * EPSILON)
struct Plane
{
	Plane() {}
	Plane(Vec3_arg normal, float d) : m_normal(normal), m_d(d) {}
	Vec3 m_normal;
	float m_d;
};

enum TriPlaneIntersectType
{
	TRI_PLANE_INTERSECT_ALL_BELOW = 0x0,
	TRI_PLANE_INTERSECT_ALL_ABOVE = 0x0111,
};

float DistToPlane(Vec3_arg p, const Plane& plane);
Vec3 ClosestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt);
Vec3 FurthestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt);
float DistSqAABBToPoint(const AABB& aabb, Vec3_arg pt);
bool AABBIntersectsSphere(const AABB& aabb, Vec3_arg sphereCenter, float sphereRadius);
bool AABBIntersectsShell(const AABB& aabb, Vec3_arg sphereCenter, float minRadius, float maxRadius);
bool AABBAbovePlane(const AABB& aabb, const Plane& plane);
Vec3 MakeSplitNormal(int splitdir);
void MakeSplitPlane(Plane& plane, int dir, const AABB& bounds);
bool ComputeCircumcircle(Vec3_arg a, Vec3_arg b, Vec3_arg c, Vec3& outCenter, float &outRadiusSq);
bool ComputeCircumsphere(Vec3_arg a, Vec3_arg b, Vec3_arg c, Vec3_arg d, Vec3 &outCenter, float &outRadiusSq);
void PlaneIntersectsTriangleList(const Plane& plane, int numTriangles, const Vec3* triangleData, int *results);

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

#endif

