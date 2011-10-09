#pragma once

#include "math/vec3.hh"
#include <cfloat>

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
};

