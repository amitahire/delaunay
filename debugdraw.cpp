#include "debugdraw.hh"
#include "draw.hh"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <cstdio>
#include <vector>

static bool g_bDebugDrawEnabled = true;

void EnableDebugDraw() { g_bDebugDrawEnabled = true; }
void DisableDebugDraw() { g_bDebugDrawEnabled = false; }

////////////////////////////////////////////////////////////////////////////////
// Structs

struct ddPoint
{
	ddPoint( Vec3_arg p, float r, float g, float b)
		: point(p)
		, color(r,g,b)
	{
	}

	Vec3 point;
	Vec3 color;
};

struct ddPlane
{
	ddPlane(const AABB& bounds, const Plane& plane)
		: plane(plane), bounds(bounds) {}
	Plane plane;
	AABB bounds;
};

struct ddSphere
{
	ddSphere(Vec3_arg center, float radius, float r, float g, float b, float a)
		: center(center), radius(radius), r(r), g(g), b(b), a(a) {}
	Vec3 center;
	float radius;
	float r,g,b,a;
};

struct ddTri
{
	ddTri(Vec3_arg pos0, Vec3_arg pos1, Vec3_arg pos2, float r, float g, float b, float a)
		: pos0(pos0)
		, pos1(pos1)
		, pos2(pos2)
		, r(r), g(g), b(b), a(a) 
	{}

	Vec3 pos0, pos1, pos2;
	float r,g,b,a;
};

struct ddVec
{
	ddVec(Vec3_arg from, Vec3_arg vec, float r, float g, float b, float a)
		: from(from), vec(vec), r(r), g(g), b(b), a(a)
		{}
	
	Vec3 from, vec;
	float r,g,b,a;
};

////////////////////////////////////////////////////////////////////////////////
// Global State for debug rendering
static std::vector< AABB > s_ddAABBs;
static std::vector< ddPoint > s_ddPoints;
static std::vector< ddPlane > s_ddPlanes;
static std::vector< ddSphere > s_ddSpheres;
static std::vector< ddTri > s_ddTris;
static std::vector< ddVec > s_ddVecs;
void ClearDebugDraw()
{
	s_ddAABBs.clear();
	s_ddPoints.clear();	
	s_ddPlanes.clear();
	s_ddSpheres.clear();
	s_ddTris.clear();
	s_ddVecs.clear();
}

static void ddRenderAABBs();
static void ddRenderPoints();
static void ddRenderPlanes();
static void ddRenderSpheres(); 
static void ddRenderTris();
static void ddRenderVecs();

void RenderDebugDraw()
{
	ddRenderAABBs();
	ddRenderPoints();
	ddRenderPlanes();
	ddRenderSpheres();
	ddRenderTris();
	ddRenderVecs();
}

////////////////////////////////////////////////////////////////////////////////
// Render functions
static void ddRenderAABBs()
{
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.1f);
	for(int i = 0, c = s_ddAABBs.size(); i < c; ++i)
	{
		const AABB& aabb = s_ddAABBs[i];

		// Bottom half
		glVertex3f(aabb.m_min.x, aabb.m_min.y, aabb.m_min.z);
		glVertex3f(aabb.m_min.x, aabb.m_min.y, aabb.m_max.z);

		glVertex3f(aabb.m_min.x, aabb.m_min.y, aabb.m_max.z);
		glVertex3f(aabb.m_max.x, aabb.m_min.y, aabb.m_max.z);

		glVertex3f(aabb.m_max.x, aabb.m_min.y, aabb.m_max.z);
		glVertex3f(aabb.m_max.x, aabb.m_min.y, aabb.m_min.z);

		glVertex3f(aabb.m_max.x, aabb.m_min.y, aabb.m_min.z);
		glVertex3f(aabb.m_min.x, aabb.m_min.y, aabb.m_min.z);

		// Top half
		glVertex3f(aabb.m_min.x, aabb.m_max.y, aabb.m_min.z);
		glVertex3f(aabb.m_min.x, aabb.m_max.y, aabb.m_max.z);

		glVertex3f(aabb.m_min.x, aabb.m_max.y, aabb.m_max.z);
		glVertex3f(aabb.m_max.x, aabb.m_max.y, aabb.m_max.z);

		glVertex3f(aabb.m_max.x, aabb.m_max.y, aabb.m_max.z);
		glVertex3f(aabb.m_max.x, aabb.m_max.y, aabb.m_min.z);

		glVertex3f(aabb.m_max.x, aabb.m_max.y, aabb.m_min.z);
		glVertex3f(aabb.m_min.x, aabb.m_max.y, aabb.m_min.z);

		// Connecting lines

		glVertex3f(aabb.m_min.x, aabb.m_min.y, aabb.m_min.z);
		glVertex3f(aabb.m_min.x, aabb.m_max.y, aabb.m_min.z);

		glVertex3f(aabb.m_max.x, aabb.m_min.y, aabb.m_min.z);
		glVertex3f(aabb.m_max.x, aabb.m_max.y, aabb.m_min.z);

		glVertex3f(aabb.m_max.x, aabb.m_min.y, aabb.m_max.z);
		glVertex3f(aabb.m_max.x, aabb.m_max.y, aabb.m_max.z);

		glVertex3f(aabb.m_min.x, aabb.m_min.y, aabb.m_max.z);
		glVertex3f(aabb.m_min.x, aabb.m_max.y, aabb.m_max.z); 

	}
	glEnd();
}

static void ddRenderPoints()
{
	glPointSize(4.f);
	glBegin(GL_POINTS);
	for(int i = 0, c = s_ddPoints.size(); i < c; ++i)
	{
		glColor3fv(&s_ddPoints[i].color.x);
		glVertex3fv(&s_ddPoints[i].point.x);
	}
	glEnd();
	glPointSize(1.f);
	
}

static void ddRenderPlanes()
{
	for(int i = 0, c = s_ddPlanes.size(); i < c; ++i)
	{
		Plane plane = s_ddPlanes[i].plane;
		AABB bounds = s_ddPlanes[i].bounds;
		DrawPlane(bounds, plane);
	}
}

static void ddRenderSpheres()
{
	for(int i = 0, c = s_ddSpheres.size(); i < c; ++i)
	{
		ddSphere & sphere = s_ddSpheres[i];
		const int subdivs = 12;
		glColor4f(sphere.r, sphere.g, sphere.b, sphere.a);

		glPushMatrix();
		glTranslatef(sphere.center.x, sphere.center.y, sphere.center.z);
		glutSolidSphere(sphere.radius, subdivs, subdivs);
		glPopMatrix();
	}
}

static void ddRenderTris()
{
	glBegin(GL_TRIANGLES);
	for(int i = 0, c = s_ddTris.size(); i < c; ++i)
	{
		ddTri & tri = s_ddTris[i];
		glColor4f(tri.r, tri.g, tri.b, tri.a);
		glVertex3fv(&tri.pos0.x);
		glVertex3fv(&tri.pos1.x);
		glVertex3fv(&tri.pos2.x);
	}
	glEnd();
}

static void ddRenderVecs()
{
	glBegin(GL_LINES);
	for(int i = 0, c = s_ddVecs.size(); i < c; ++i)
	{
		ddVec & vec = s_ddVecs[i];
		Vec3 to = vec.from + vec.vec;
		glColor4f(vec.r, vec.g, vec.b, vec.a);
		glVertex3fv(&vec.from.x);
		glVertex3fv(&to.x);
	}
	glEnd();
	
}

////////////////////////////////////////////////////////////////////////////////
// Append functions

void DebugDrawAABB(const AABB& aabb)
{
	if(!g_bDebugDrawEnabled) return;
	s_ddAABBs.push_back(aabb);
}

void DebugDrawPlane(const AABB& aabb, const Plane& plane)
{
	if(!g_bDebugDrawEnabled) return;
	s_ddPlanes.push_back( ddPlane(aabb, plane) );
}

void DebugDrawPoint(Vec3_arg pos, float r, float g, float b)
{
	if(!g_bDebugDrawEnabled) return;
	s_ddPoints.push_back( ddPoint(pos, r, g, b) );
}

void DebugDrawSphere(Vec3_arg center, float radius, float r, float g, float b, float a)
{
	if(!g_bDebugDrawEnabled) return;
	s_ddSpheres.push_back( ddSphere(center, radius, r, g, b, a) );
}

void DebugDrawTriangle(Vec3_arg pos0, Vec3_arg pos1, Vec3_arg pos2, float r, float g, float b, float a)
{
	if(!g_bDebugDrawEnabled) return;
	s_ddTris.push_back( ddTri(pos0, pos1, pos2, r, g, b, a) );
}

void DebugDrawVector(Vec3_arg from, Vec3_arg vec, float r, float g, float b, float a)
{
	if(!g_bDebugDrawEnabled) return;
	s_ddVecs.push_back( ddVec(from, vec, r, g, b, a) );
}

void DebugDrawLine(Vec3_arg from, Vec3_arg to, float r, float g, float b, float a)
{
	if(!g_bDebugDrawEnabled) return;
	s_ddVecs.push_back( ddVec(from, to - from, r, g, b, a) );
}
