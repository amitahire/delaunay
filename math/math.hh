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

float DistToPlane(Vec3_arg p, const Plane& plane);
Vec3 ClosestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt);
Vec3 FurthestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt);
float DistSqAABBToPoint(const AABB& aabb, Vec3_arg pt);
bool AABBIntersectsSphere(const AABB& aabb, Vec3_arg sphereCenter, float sphereRadius);
bool AABBIntersectsShell(const AABB& aabb, Vec3_arg sphereCenter, float minRadius, float maxRadius);
bool AABBAbovePlane(const AABB& aabb, const Plane& plane);
Vec3 MakeSplitNormal(int splitdir);
void MakeSplitPlane(Plane& plane, int dir, const AABB& bounds);

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

inline float AngleWrap(float angle)
{
	while(angle < 0.f)
		angle += 2.f * M_PI;
	while(angle > 2.f * M_PI)
		angle -= 2.f * M_PI;
	return angle;
}

#endif

