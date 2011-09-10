#include <cstdio>
#include <cmath>
#include <cstdlib>
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
// Global state

static int g_state = STATE_Init;
static int g_width, g_height;
static float g_eyeDist = 310.f, g_pitch = M_PI / 4.f, g_yaw = M_PI / 4.f;
static Vec3 g_center(0,0,0);
static Vec3 *g_points;
static int g_numPoints = 1000;
static unsigned int g_seed = 12345U;
static SparsePointGrid *s_grid;
static Triangulator *s_triangulator;
static bool s_bStepTriangulator = false;

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
	}
}

void on_display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// camera

	float cos_pitch = cos(g_pitch);
	float eyeX = g_eyeDist * cos_pitch * cos(g_yaw);
	float eyeY = g_eyeDist * sin(g_pitch);
	float eyeZ = g_eyeDist * cos_pitch * sin(g_yaw);

	glLoadIdentity();
	gluLookAt(eyeX, eyeY, eyeZ, g_center.x, g_center.y, g_center.z, 0, 1, 0);

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
		glColor3f(1,1,1);
		glBegin(GL_POINTS);
		for(int i = 0, c = g_numPoints; i < c; ++i)
			glVertex3f(g_points[i].x, g_points[i].y, g_points[i].z);
		glEnd();
	}
	RenderDebugDraw();

	glutSwapBuffers();
}

void on_keyboard(unsigned char key, int x, int y)
{
	if(key == ' ')
	{
		s_bStepTriangulator = true;
	}
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
	glutMotionFunc( on_motion );
	glutPassiveMotionFunc( on_passive_motion );

	setup_gl();

	// in loop

	// 1. generate random points
	// 2. generate delaunay tri one piece at a time (for vis) using dewall
	// 3. when finished export to file optionally

	glutMainLoop();
}

