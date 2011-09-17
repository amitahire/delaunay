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
	return planeDist >= -radius;
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

bool ComputeCircumcircle(Vec3_arg a, Vec3_arg b, Vec3_arg c, Vec3& outCenter, float &outRadiusSq)
{
	// Solving with Cramer's rule - just solving one t-parameter of two lines intersecting.
	Vec3 ab = b - a;
	Vec3 bc = c - b;
	Vec3 abHalf = 0.5f * a + 0.5f * b;
	Vec3 bcHalf = 0.5f * b + 0.5f * c;
	Vec3 midpointDiff = bcHalf - abHalf;

	Vec3 normal = cross(ab, bc);
	Vec3 abNormal = cross(normal, ab);
	Vec3 bcNormal = cross(normal, bc);

	float dot0 = dot(abNormal, abNormal);
	float dot1 = dot(abNormal, bcNormal);
	float dot3 = -dot(bcNormal, bcNormal);

	float denom = dot0 * dot3 + dot1 * dot1; // dot1 * dot1 = - (-dot1) * (dot1)
	if( fabs(denom) < EPSILON )
		return false;

	float dot4 = dot(abNormal, midpointDiff);
	float dot5 = dot(bcNormal, midpointDiff);

	float t = ( dot4 * dot3 + dot1 * dot5 ) / denom; // + dot 1 = - (-dot1)
	outCenter = abHalf + t * abNormal ;
	outRadiusSq = magnitude_squared(a - outCenter);
	
	return true;

}

bool ComputeCircumsphere(Vec3_arg a, Vec3_arg b, Vec3_arg c, Vec3_arg d, Vec3 &outCenter, float &outRadiusSq)
{
	Mat4 aMat(a.x, a.y, a.z, 1.f,
		b.x, b.y, b.z, 1.f,
		c.x, c.y, c.z, 1.f,
		d.x, d.y, d.z, 1.f);
	double aCoef = ddet(aMat);

	if(fabs(aCoef) < EPSILON)
		return false;

	Mat4 bMat = aMat;
	bMat.m[0] = a.x * a.x + a.y * a.y + a.z * a.z;
	bMat.m[4] = b.x * b.x + b.y * b.y + b.z * b.z;
	bMat.m[8] = c.x * c.x + c.y * c.y + c.z * c.z;
	bMat.m[12] = d.x * d.x + d.y * d.y + d.z * d.z;
	double bxCoef = ddet(bMat);

	bMat.m[1] = a.x;
	bMat.m[5] = b.x;
	bMat.m[9] = c.x;
	bMat.m[13] = d.x;
	double byCoef = -ddet(bMat);

	bMat.m[2] = a.y;
	bMat.m[6] = b.y;
	bMat.m[10] = c.y;
	bMat.m[14] = d.y;
	double bzCoef = ddet(bMat);

	bMat.m[3] = a.z;
	bMat.m[7] = b.z;
	bMat.m[11] = c.z;
	bMat.m[15] = d.z;
	double cCoef = ddet(bMat);

	double inv_double_a = 1.0 / (2.0 * aCoef);
	outCenter.x = bxCoef * inv_double_a;
	outCenter.y = byCoef * inv_double_a;
	outCenter.z = bzCoef * inv_double_a;

	outRadiusSq = float((bxCoef * bxCoef + byCoef * byCoef + bzCoef * bzCoef - 4.f * aCoef * cCoef) / (4.f * aCoef * aCoef));

	return true;
}

void PlaneIntersectsTriangleList(const Plane& plane, int numTriangles, const Vec3* triangleData, int *results)
{
	int triangleOff = 0;
	for(int i = 0; i < numTriangles; ++i)
	{
		int v0r = (int(dot(plane.m_normal, triangleData[triangleOff]) - plane.m_d >= 0.f)) & 1;
		int v1r = (int(dot(plane.m_normal, triangleData[triangleOff+1]) - plane.m_d >= 0.f) & 1) << 1 ;
		int v2r = (int(dot(plane.m_normal, triangleData[triangleOff+2]) - plane.m_d >= 0.f) & 1) << 2 ;

		results[i] = v0r | v1r | v2r;
		triangleOff += 3;
	}
}

