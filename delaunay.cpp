#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <debugdraw.hh>
#include <math/math.hh>
#include <sparsegrid.hh>
#include <triangulator.hh>

enum AppState
{
	STATE_Init,
	STATE_BuildGrid,
	STATE_Triangulating,
	STATE_DisplayPoints,
};

////////////////////////////////////////////////////////////////////////////////
// Global state (file scope)

static int g_state = STATE_Init;
static int g_width, g_height;
static float g_eyeDist = 310.f, g_pitch = M_PI / 4.f, g_yaw = M_PI / 4.f;
static float g_eyeDistTarget = 310.f;
static Vec3 g_center(0,0,0);
static Vec3 g_centerTarget(0.f, 0.f, 0.f);
static Vec3 *g_points;
static int g_numPoints = 100;
static unsigned int g_seed = 12345U;
static SparsePointGrid *s_grid;
static Triangulator *s_triangulator;
static bool s_bStepTriangulator = false;
static timespec g_last_time;

////////////////////////////////////////////////////////////////////////////////
// GLUT callbacks

void on_reshape(int width, int height)
{
	g_width = width;
	g_height = height;
	glViewport(0, 0, g_width, g_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.f, g_width / (float)g_height, 1.f, 1000.f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void setup_gl()
{
	glClearColor(0.1f, 0.1f, 0.1f, 1.f);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glShadeModel(GL_FLAT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	on_reshape(800, 800);
}

void generate_points()
{
	if(g_points)
		delete[] g_points;

	g_points = new Vec3[g_numPoints];

	unsigned int seed = g_seed;
	const float inv_randmax = 1.f/RAND_MAX;
	for(int i = 0, c = g_numPoints; i < c; ++i)
	{
		int ix = rand_r(&seed);
		int iy = rand_r(&seed);
		int iz = rand_r(&seed);

		g_points[i].x = ((ix * inv_randmax) * 2.f - 1.f) * 100.f;
		g_points[i].y = ((iy * inv_randmax) * 2.f - 1.f) * 100.f;
		g_points[i].z = ((iz * inv_randmax) * 2.f - 1.f) * 100.f;
	}
}

void on_idle(void)
{
	timespec current_time;
	clock_gettime(CLOCK_MONOTONIC, &current_time);

	time_t diffSec = current_time.tv_sec - g_last_time.tv_sec;
	long diffNsec = current_time.tv_nsec - g_last_time.tv_nsec;

	long diffMsec = diffSec * 1000 + diffNsec / (1000 * 1000);
	float dt = diffMsec / 1000.f;


	if(dt > 1/60.f)
	{
		g_last_time = current_time;
		float t = 0.9f * dt;

		g_center = t * g_centerTarget + (1.f - t) * g_center;
		g_eyeDist = t * g_eyeDistTarget + (1.f - t) * g_eyeDist;
		if(magnitude_squared(g_center - g_centerTarget) > EPSILON_SQ ||
			(fabs(g_eyeDistTarget - g_eyeDist) > EPSILON))
		{
			glutPostRedisplay();
		}

	}

	if(g_state == STATE_Init)
	{
		printf("Initializing points...\n");
		generate_points();
		++g_state;
		glutPostRedisplay();
	}
	else if(g_state == STATE_BuildGrid)
	{
		printf("building grid...\n");
		s_grid = new SparsePointGrid(256.f, 64);
		s_grid->InsertPoints(g_points, g_numPoints);
		++g_state;
		glutPostRedisplay();

		printf("Triangulating...\n");
		s_triangulator = new Triangulator(s_grid);
	}
	else if(g_state == STATE_Triangulating)
	{
		if(s_triangulator->IsDone())
		{
			ClearDebugDraw();
			printf("Done!\n");
			// extract triangulation? for now just let the s_triangulator stay in mem
			++g_state;
			glutPostRedisplay();
		}
		else
		{
			if(s_bStepTriangulator)
			{
				ClearDebugDraw();
				printf("Step\n");
				s_triangulator->Step();
				s_bStepTriangulator = false;
				glutPostRedisplay();
			}
		}

		int lastTet = s_triangulator->GetNumTetrahedrons() - 1;
		if(lastTet >= 0)
		{
			const Triangulator::Tetrahedron& tet = s_triangulator->GetTetrahedron(lastTet);
			Vec3 v0Pos = s_grid->GetPos(tet.v0);
			Vec3 v1Pos = s_grid->GetPos(tet.v1);
			Vec3 v2Pos = s_grid->GetPos(tet.v2);
			Vec3 v3Pos = s_grid->GetPos(tet.v3);

			g_centerTarget = (v0Pos + v1Pos + v2Pos + v3Pos) / 4.f;
		}
	}
}

void on_display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	// camera

	float cos_pitch = cos(g_pitch);
	float eyeX = g_eyeDist * cos_pitch * cos(g_yaw);
	float eyeY = g_eyeDist * sin(g_pitch);
	float eyeZ = g_eyeDist * cos_pitch * sin(g_yaw);

	glLoadIdentity();
	gluLookAt(eyeX + g_center.x, eyeY + g_center.y, eyeZ + g_center.z, g_center.x, g_center.y, g_center.z, 0, 1, 0);

	// point cloud

	if(g_state == STATE_DisplayPoints)
	{
		glColor3f(1,1,1);
		glBegin(GL_POINTS);
		for(int i = 0, c = g_numPoints; i < c; ++i)
			glVertex3f(g_points[i].x, g_points[i].y, g_points[i].z);
		glEnd();
	}
	else if(g_state == STATE_Triangulating)
	{
		for(int i = 0, c = s_grid->GetNumPoints(); i < c; ++i)
		{
			if(!s_grid->IsValidPoint(i))
			{
				Vec3 pos = s_grid->GetPos(i);
				DebugDrawPoint(pos, 0.5f, 0.5f, 0.5f);
			}
		}
		
		glBegin(GL_POINTS);
		glColor3f(1,1,1);
		glPointSize(1.f);
		for(int i = 0, c = s_grid->GetNumPoints(); i < c; ++i)
		{
			if(s_grid->IsValidPoint(i))
			{
				Vec3 pos = s_grid->GetPos(i);
				glVertex3fv(&pos.x);
			}
		}
		glEnd();

		glColor3f(0.8f, 0.8f, 0.8f);
		for(int pass = 0; pass < 2; ++pass)
		{
			if(pass == 1)
			{
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				glLineWidth(3.f);
				glEnable(GL_POLYGON_OFFSET_LINE);
				glPolygonOffset(1.f, -10.f);
				glColor3f(0.2f, 0.2f, 0.2f);
			}

			glBegin(GL_TRIANGLES);
			for(int i = 0, c = s_triangulator->GetNumTetrahedrons(); i < c; ++i)
			{
				const Triangulator::Tetrahedron& tet = s_triangulator->GetTetrahedron(i);
				Vec3 v0Pos = s_grid->GetPos(tet.v0);
				Vec3 v1Pos = s_grid->GetPos(tet.v1);
				Vec3 v2Pos = s_grid->GetPos(tet.v2);
				Vec3 v3Pos = s_grid->GetPos(tet.v3);

				// Bottom
				glVertex3fv(&v0Pos.x);
				glVertex3fv(&v1Pos.x);
				glVertex3fv(&v2Pos.x);

				// Side 0
				glVertex3fv(&v0Pos.x);
				glVertex3fv(&v2Pos.x);
				glVertex3fv(&v3Pos.x);

				// Side 1
				glVertex3fv(&v1Pos.x);
				glVertex3fv(&v3Pos.x);
				glVertex3fv(&v2Pos.x);

				// Side 2
				glVertex3fv(&v0Pos.x);
				glVertex3fv(&v3Pos.x);
				glVertex3fv(&v1Pos.x);
			}
			glEnd();
		}
		glDisable(GL_POLYGON_OFFSET_LINE);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		glPolygonOffset(0.f, 0.f);
		glLineWidth(1.f);
	}
	//glEnable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);
	RenderDebugDraw();
	glDisable(GL_CULL_FACE);

	glutSwapBuffers();
}

void on_keyboard(unsigned char key, int x, int y)
{
	if(key == ' ')
	{
		s_bStepTriangulator = true;
	}
}

void on_special_keyboard(int key, int x, int y)
{
	if(key == GLUT_KEY_UP)
	{
		if(g_eyeDistTarget < 10.f)
			g_eyeDistTarget -= 1.f;
		else 
			g_eyeDistTarget -= 10.f;
	}
	else if(key == GLUT_KEY_DOWN)
	{	
		if(g_eyeDistTarget < 10.f)
			g_eyeDistTarget += 1.f;
		else 
			g_eyeDistTarget += 10.f;
	}

	g_eyeDistTarget = Max(1.f, g_eyeDistTarget);
}

static int g_lastx = -1, g_lasty = -1;

void on_passive_motion(int x, int y)
{
	g_lastx = x;
	g_lasty = y;
}

void on_motion(int x, int y)
{
	if(g_lastx < 0) g_lastx = x;
	if(g_lasty < 0) g_lasty = y;

	int dx = x - g_lastx;
	int dy = y - g_lasty;

	g_lastx = x;
	g_lasty = y;

	g_pitch += 5.f * (dy / float(g_height)) / M_PI;
	g_yaw += 5.f * (dx / float(g_width)) / M_PI;

	glutPostRedisplay();
}

////////////////////////////////////////////////////////////////////////////////
// Main
int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutCreateWindow("delaunay");
	g_width = 800; g_height = 800;
	glutInitWindowSize( g_width, g_height );

	glutIdleFunc( on_idle );
	glutReshapeFunc( on_reshape );
	glutDisplayFunc( on_display );
	glutKeyboardFunc( on_keyboard );
	glutSpecialFunc( on_special_keyboard );
	glutMotionFunc( on_motion );
	glutPassiveMotionFunc( on_passive_motion );

	setup_gl();

	clock_gettime(CLOCK_MONOTONIC, &g_last_time);
	glutMainLoop();
}

