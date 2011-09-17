#include "debugdraw.hh"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <cstdio>
#include <vector>

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
	struct Edge
	{
		int start;
		int end;
	};

	static Edge edges[12] =
	{
		// bottom
		{0, 1},
		{1, 3},
		{3, 2},
		{2, 0},

		// top
		{4, 5},
		{5, 7},
		{7, 6},
		{6, 4},

		// top to bottom sides
		{0, 4},
		{1, 5},
		{2, 6},
		{3, 7},
	};

	Vec3 points[6];
	for(int i = 0, c = s_ddPlanes.size(); i < c; ++i)
	{
		Plane plane = s_ddPlanes[i].plane;
		AABB bounds = s_ddPlanes[i].bounds;
		// clip plane to bounds and render a quad

		Vec3 corners[8];
		Vec3 *minmax[2] = { &bounds.m_min, &bounds.m_max };	
		for(int j = 0; j < 8; ++j)
		{
			int iz = j & 1;
			int ix = (j >> 1) & 1;
			int iy = (j >> 2) & 1;
			corners[j] = Vec3(minmax[ix]->x, minmax[iy]->y, minmax[iz]->z);
		}

		int numPoints = 0;

		// add corners
		for(int j = 0; j < 8; ++j)
		{
			float planeDist = dot(plane.m_normal, corners[j]) - plane.m_d;
			if(fabs(planeDist) < EPSILON)
				points[numPoints++] = corners[j];
		}

		// add edges
		for(int j = 0; j < 12; ++j)
		{
			Vec3 a = corners[edges[j].start];
			Vec3 b = corners[edges[j].end];
			Vec3 ab = b - a;

			float t = (plane.m_d - dot(plane.m_normal, a)) / dot(plane.m_normal, ab);
			if(t >= 0.f && t <= 1.f)
			{
				Vec3 pt = a + t * ab;
				Vec3 ptA = a - pt;
				Vec3 ptB = b - pt;
				float distSqA = magnitude_squared(ptA);
				float distSqB = magnitude_squared(ptB);
				if(distSqA > EPSILON_SQ && distSqB > EPSILON_SQ)
				{
					points[numPoints++] = pt;
					if(numPoints == 6)
						break;
				}	
			}
		}

		if(numPoints < 3)
			continue;

		// Sort results
		float inv_num = 1.f / numPoints;
		Vec3 center(0,0,0);
		for(int j = 0; j < numPoints; ++j)
			center += points[j] * inv_num;

		Vec3 sideVec = normalize(points[0] - center);
		Vec3 upVec = normalize(cross(plane.m_normal, sideVec));

		for(int j = 1; j < numPoints; ++j)
		{	
			Vec3 toPointJ = points[j] - center;
			float angleJ = AngleWrap(atan2(dot(upVec, toPointJ), dot(sideVec, toPointJ)));
			for(int k = j+1; k < numPoints; ++k)
			{
				Vec3 toPointK = points[k] - center;
				float angleK = AngleWrap(atan2(dot(upVec, toPointK), dot(sideVec, toPointK)));
				if(angleK < angleJ) 
				{
					angleJ = angleK;
					Vec3 temp = points[j];
					points[j] = points[k];
					points[k] = temp;
				}
			}
		}

		// Draw outline
		glColor3f(7.f, 0.3f, 0.3f);
		glLineWidth(2.f);

		glBegin(GL_LINES);
		for(int j = 0; j < numPoints; ++j)
		{
			int next = (j + 1) % numPoints;
			glVertex3fv(&points[j].x);
			glVertex3fv(&points[next].x);
		}
		glEnd();

		glLineWidth(1.f);
		
		// Draw triangles
		glColor4f(7.f, 0.3f, 0.3f, 0.3f);
		glBegin(GL_TRIANGLE_FAN);
		glVertex3fv(&center.x);
		for(int j = 0; j < numPoints; ++j)
			glVertex3fv(&points[j].x);
		glVertex3fv(&points[0].x);
		glEnd();

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
	s_ddAABBs.push_back(aabb);
}

void DebugDrawPlane(const AABB& aabb, const Plane& plane)
{
	s_ddPlanes.push_back( ddPlane(aabb, plane) );
}

void DebugDrawPoint(Vec3_arg pos, float r, float g, float b)
{
	s_ddPoints.push_back( ddPoint(pos, r, g, b) );
}

void DebugDrawSphere(Vec3_arg center, float radius, float r, float g, float b, float a)
{
	s_ddSpheres.push_back( ddSphere(center, radius, r, g, b, a) );
}

void DebugDrawTriangle(Vec3_arg pos0, Vec3_arg pos1, Vec3_arg pos2, float r, float g, float b, float a)
{
	s_ddTris.push_back( ddTri(pos0, pos1, pos2, r, g, b, a) );
}

void DebugDrawVector(Vec3_arg from, Vec3_arg vec, float r, float g, float b, float a)
{
	s_ddVecs.push_back( ddVec(from, vec, r, g, b, a) );
}

void DebugDrawLine(Vec3_arg from, Vec3_arg to, float r, float g, float b, float a)
{
	s_ddVecs.push_back( ddVec(from, to - from, r, g, b, a) );
}
