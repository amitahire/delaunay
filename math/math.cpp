#include <cstdio>
#include "math/math.hh"
#include "debugdraw.hh"

float DistToPlane(Vec3_arg p, const Plane& plane)
{
	return fabs(dot(plane.m_normal, p) - plane.m_d);
}

Vec3 ClosestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt)
{
	Vec3 result = pt;
	result = VecMax(result, aabb.m_min);
	result = VecMin(result, aabb.m_max);
	return result;
}

Vec3 FurthestPointOnAABBToPoint(const AABB& aabb, Vec3_arg pt)
{
	Vec3 result = pt;
	result = VecMax(result, aabb.m_max);
	result = VecMin(result, aabb.m_min);
	return result;
}

float DistSqAABBToPoint(const AABB& aabb, Vec3_arg pt)
{
	float distSq = 0.f;
	for(int i = 0; i < 3; ++i)
	{
		if(pt[i] < aabb.m_min[i]) distSq += (aabb.m_min[i] - pt[i]) * (aabb.m_min[i] - pt[i]);
		if(pt[i] > aabb.m_max[i]) distSq += (pt[i] - aabb.m_max[i]) * (pt[i] - aabb.m_max[i]);
	}
	return distSq;
}

bool AABBIntersectsSphere(const AABB& aabb, Vec3_arg sphereCenter, float sphereRadius)
{
	float distSq = DistSqAABBToPoint(aabb, sphereCenter);
	return distSq < (sphereRadius * sphereRadius);
}

bool AABBIntersectsShell(const AABB& aabb, Vec3_arg sphereCenter, float minRadius, float maxRadius)
{
	float distSq = DistSqAABBToPoint(aabb, sphereCenter);
	if(distSq < (maxRadius * maxRadius))
	{
		Vec3 furthestPt = FurthestPointOnAABBToPoint(aabb, sphereCenter);
		distSq = magnitude_squared(furthestPt - sphereCenter);
		return distSq > (minRadius * minRadius);
	}
	return false;	
}

bool AABBAbovePlane(const AABB& aabb, const Plane& plane)
{
	Vec3 diff = 0.5f * aabb.m_max - 0.5f * aabb.m_min;
	float radius = fabs(diff[0]) + fabs(diff[1]) + fabs(diff[2]);
	Vec3 center = aabb.m_min + diff;
	float planeDist = dot(plane.m_normal, center) - plane.m_d;
	return planeDist > -radius;
}

Vec3 MakeSplitNormal(int splitdir)
{
	return Vec3(
		float(splitdir == 0),
		float(splitdir == 1),
		float(splitdir == 2));
}

void MakeSplitPlane(Plane& plane, int dir, const AABB& bounds)
{
	plane.m_normal = MakeSplitNormal(dir);
	plane.m_d = dot(0.5f*(bounds.m_max + bounds.m_min), plane.m_normal);
}

