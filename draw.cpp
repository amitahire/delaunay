#include "draw.hh"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "math/math.hh"

void DrawPlane(const AABB& bounds, const Plane& plane)
{
	Vec3 points[6];
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

	// clip plane to bounds and render a quad

	Vec3 corners[8];
	const Vec3 *minmax[2] = { &bounds.m_min, &bounds.m_max };	
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
		return;

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

	glBegin(GL_TRIANGLE_FAN);
	glVertex3fv(&center.x);
	for(int j = numPoints-1; j >= 0; --j)
		glVertex3fv(&points[j].x);
	glVertex3fv(&points[numPoints-1].x);
	glEnd();

}

