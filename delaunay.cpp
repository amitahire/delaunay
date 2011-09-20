#include "common.hh"
#include <cmath>
#include <ctime>
#include <vector>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include "debugdraw.hh"
#include "draw.hh"
#include "math/math.hh"
#include "sparsegrid.hh"
#include "triangulator.hh"
#include "cmdhelper.hh"

enum AppState
{
	STATE_Init,
	STATE_Triangulating,
	STATE_DisplayVolumeMesh,
};

////////////////////////////////////////////////////////////////////////////////
// Global state (file scope)

static int g_state = STATE_Init;
static int g_width, g_height;
static float g_eyeDist = 310.f, g_pitch = M_PI / 4.f, g_yaw = M_PI / 4.f;
static float g_eyeDistTarget = 310.f;
static Vec3 g_center(0,0,0);
static Vec3 g_centerTarget(0.f, 0.f, 0.f);
static int g_numPoints = 1000;
static unsigned int g_seed = 12345U;
static Triangulator *s_triangulator;
static SparsePointGrid *s_grid;
static bool s_bStepTriangulator = false;
static int s_stepTriangulatorCount = 1;
static timespec g_last_time;
static bool g_bAuto = false;
static bool g_bDebugRender = true;
static bool g_bEnableCutaway = false;
static bool g_bEnableAnimCutaway = false;
static bool g_bOneStep = false;
static int g_gridDims = 64;
static float g_cutawayParam = 0.f;
static float g_cutawayParamTarget = 1.f;
static int g_cutawayDir = 0;
static bool g_bPaused = false;
static const char* g_szPointFile;
static const char* g_szMeshFile;

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

void generate_points(std::vector<Vec3>& points)
{
	points.clear();
	points.resize(g_numPoints);

	unsigned int seed = g_seed;
	const float inv_randmax = 1.f/RAND_MAX;
	for(int i = 0, c = g_numPoints; i < c; ++i)
	{
		int ix = rand_r(&seed);
		int iy = rand_r(&seed);
		int iz = rand_r(&seed);

		Vec3 &pt = points[i];
		pt.x = ((ix * inv_randmax) * 2.f - 1.f) * 100.f;
		pt.y = ((iy * inv_randmax) * 2.f - 1.f) * 100.f;
		pt.z = ((iz * inv_randmax) * 2.f - 1.f) * 100.f;
	}
}

void load_points(const char* szPointFile, std::vector<Vec3>& points)
{
	points.clear();
	printf("Loading points from %s\n", szPointFile);

	FILE *fp = fopen(szPointFile, "rb");
	if(!fp)
	{
		printf("Failed to load point file %s\n", szPointFile);
		return;
	}

	Vec3 pt;
	char line[256];
	char *read;
	do
	{
		read = fgets(line, ARRAY_SIZE(line), fp);
		if(read)
		{	
			int numRead = sscanf(read, "%f %f %f\n", &pt.x, &pt.y, &pt.z);
			if(numRead == 3)
			{
				points.push_back(pt);
			}
		}
	} while(read);
	fclose(fp);

	printf("Read %d points.\n", int(points.size()));
}

void write_volume_mesh(const SparsePointGrid *grid, const Triangulator* triangulator, const char *szMeshFile)
{
	FILE* fp = fopen(szMeshFile, "wb");
	if(!fp)
	{
		printf("Failed to open %s for writing.\n", szMeshFile);
		return;
	}

	Vec3 tetPos[4];
	for(int i = 0, c = s_triangulator->GetNumTetrahedrons(); i < c; ++i)
	{
		const Triangulator::Tetrahedron& tet = s_triangulator->GetTetrahedron(i);
		tetPos[0] = s_grid->GetPos(tet.v0);
		tetPos[1] = s_grid->GetPos(tet.v1);
		tetPos[2] = s_grid->GetPos(tet.v2);
		tetPos[3] = s_grid->GetPos(tet.v3);

		for(int j = 0; j < 4; ++j)
			fprintf(fp, "%f %f %f ", tetPos[j].x, tetPos[j].y, tetPos[j].z);
		fprintf(fp, "\n");
	}
	fclose(fp);
	printf("Wrote %d tetrahedrons to %s.\n", s_triangulator->GetNumTetrahedrons(), szMeshFile);
}

void add_outer_points(std::vector<Vec3>& points, const AABB& aabb)
{
	Vec3 bounds[] = { aabb.m_min + Vec3(EPSILON, EPSILON, EPSILON), 
		aabb.m_max - Vec3(EPSILON, EPSILON, EPSILON)};
	for(int i = 0; i < 8; ++i)
	{
		int zSel = (i >> 2) & 1;
		int ySel = (i >> 1) & 1;
		int xSel = i & 1;

		Vec3 pt(bounds[xSel][0], bounds[ySel][1], bounds[zSel][2]);
		points.push_back(pt);
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


	if(dt > 1/30.f)
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
		std::vector<Vec3> points;

		if(g_szPointFile)
			load_points(g_szPointFile, points);
		else
			generate_points(points);

		if(points.empty())
		{
			exit(1);
		}
		add_outer_points(points, AABB(Vec3(-128, -128, -128), Vec3(128, 128, 128)));

		printf("building grid...\n");
		SparsePointGrid * grid = new SparsePointGrid(256.f, g_gridDims);
		grid->InsertPoints(&points[0], points.size());
		++g_state;
		glutPostRedisplay();

		printf("Triangulating...\n");
		s_grid = grid;
		s_triangulator = new Triangulator(grid);
	}
	else if(g_state == STATE_Triangulating)
	{
		if(g_bOneStep)
		{
			ClearDebugDraw();
			DisableDebugDraw();
		
			timespec start_time;
			timespec end_time;

			int stepCount = 0;
			printf("*\r");
			char progress[] = { '/','-','\\', '|' };
			clock_gettime(CLOCK_MONOTONIC, &start_time);
			while(!s_triangulator->IsDone())
			{
				s_triangulator->Step();
				++stepCount;
				int idx = (stepCount / 100) % sizeof(progress);
				printf("%c (%d)\r", progress[idx], stepCount);
			}
			clock_gettime(CLOCK_MONOTONIC, &end_time);

			time_t diffSec = end_time.tv_sec - start_time.tv_sec;
			long diffNSec = end_time.tv_nsec - start_time.tv_nsec;

			long timeElapsedMs = diffSec * 1000 + diffNSec / (1000 * 1000);

			EnableDebugDraw();
			printf("\rDone (Single Step in %ds %dms)!\n", int(timeElapsedMs/1000), int(timeElapsedMs % 1000));

			write_volume_mesh(s_grid, s_triangulator, g_szMeshFile);
			++g_state;
			glutPostRedisplay();
		}
		else 
		{
			if(s_triangulator->IsDone())
			{
				ClearDebugDraw();
				printf("Done!\n");
				write_volume_mesh(s_grid, s_triangulator, g_szMeshFile);
				++g_state;
				glutPostRedisplay();
			}
			else if(!g_bPaused)
			{
				if(s_bStepTriangulator || g_bAuto)
				{
					ClearDebugDraw();
					DisableDebugDraw();
					while(s_stepTriangulatorCount-- > 1)
					{
						s_triangulator->Step();
					}
					EnableDebugDraw();
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
	else if(g_state == STATE_DisplayVolumeMesh)
	{
		g_centerTarget = Vec3(0,0,0);
		if(g_bEnableAnimCutaway && dt > 1/30.f)
		{
			float delta = 0.1f * ((g_cutawayParamTarget == 0.f) ? -dt : dt);
			g_cutawayParam = Clamp(g_cutawayParam + delta, 0.f, 1.f);
			if(fabs(g_cutawayParam - g_cutawayParamTarget) < EPSILON)
			{
			//	if(g_cutawayParamTarget == 0.f) {
			//		g_cutawayDir = (g_cutawayDir + 1) % 3;
			//	}
				g_cutawayParamTarget = (1.f - g_cutawayParamTarget);
			}
			glutPostRedisplay();
		}
	}
}

void on_display(void)
{
	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	float cos_pitch = cos(g_pitch);
	float eyeX = g_eyeDist * cos_pitch * cos(g_yaw);
	float eyeY = g_eyeDist * sin(g_pitch);
	float eyeZ = g_eyeDist * cos_pitch * sin(g_yaw);

	glLoadIdentity();
	gluLookAt(eyeX + g_center.x, eyeY + g_center.y, eyeZ + g_center.z, g_center.x, g_center.y, g_center.z, 0, 1, 0);

	if(g_state == STATE_Triangulating)
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
				glLineWidth(1.f);
				glEnable(GL_POLYGON_OFFSET_FILL);
				glPolygonOffset(0.f, 0.01f);
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
	else if(g_state == STATE_DisplayVolumeMesh)
	{
		Plane cutaway;
		cutaway.m_normal = Vec3(g_cutawayDir == 0, g_cutawayDir == 1, g_cutawayDir == 2);
		float cutaway_t = sin( g_cutawayParam * M_PI / 2.f ) ;
		const AABB &allPointsAABB = s_grid->GetAllPointsAABB();
		cutaway.m_d = dot((1.f - cutaway_t) * 1.1f * allPointsAABB.m_min +
			cutaway_t * 1.1f * allPointsAABB.m_max, cutaway.m_normal);

		glEnable(GL_CULL_FACE);
		glColor4f(0.8f, 0.8f, 0.8f, 1.f);
		for(int pass = 0; pass < 2; ++pass)
		{
			if(pass == 1)
			{
				glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
				glLineWidth(1.f);
				glEnable(GL_POLYGON_OFFSET_FILL);
				glPolygonOffset(0.f, 0.001f);
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
				
				if(g_bEnableCutaway)
				{
					if(dot(cutaway.m_normal, v0Pos) - cutaway.m_d > 0.f) continue;
					if(dot(cutaway.m_normal, v1Pos) - cutaway.m_d > 0.f) continue;
					if(dot(cutaway.m_normal, v2Pos) - cutaway.m_d > 0.f) continue;
					if(dot(cutaway.m_normal, v3Pos) - cutaway.m_d > 0.f) continue;
				}

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
		
		if(g_bEnableCutaway) DrawPlane(allPointsAABB, cutaway);
	}

	if(g_bDebugRender)
	{
		glDisable(GL_DEPTH_TEST);
		RenderDebugDraw();
	}

	glutSwapBuffers();
}

void on_keyboard(unsigned char key, int x, int y)
{
	if(key == ' ')
	{
		if(g_bAuto)
			g_bPaused = !g_bPaused;
		else
		{
			s_bStepTriangulator = true;
			s_stepTriangulatorCount = 1;
		}

	}
	else if(key == 'S')
	{
		s_bStepTriangulator = true;
		s_stepTriangulatorCount = 100;
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
// Command processing
void CmdAuto(int, char**)
{
	g_bAuto = true;
}

void CmdNoDebug(int, char**)
{
	g_bDebugRender = false;
}

void CmdCutaway(int, char**)
{
	g_bEnableCutaway = true;
}

void CmdAnimCutaway(int, char**)
{
	g_bEnableAnimCutaway = true;
	g_bEnableCutaway = true;
}

void CmdSkip(int, char**)
{
	g_bOneStep = true;
}

void CmdNumPoints(int argc, char** argv)
{
	g_numPoints = Max(atoi(argv[1]), 3);
	printf("Setting num points to %d\n", g_numPoints);
}

void CmdCellCount(int argc, char **argv)
{
	g_gridDims = Max(4, atoi(argv[1]));
	printf("Setting Uniform grid to %dx%dx%d\n", g_gridDims, g_gridDims, g_gridDims);
}

void CmdPause(int, char **)
{
	g_bPaused = true;
}

void CmdPoints(int argc, char** argv)
{
	g_szPointFile = argv[1];
}

void CmdOutput(int argc, char** argv)
{
	g_szMeshFile = argv[1];
}

void CmdHelp(int, char**);
static CmdOption g_options[] = 
{
	{ &CmdAuto, "--auto", NULL, 0, "Automatically step." }, 
	{ &CmdNoDebug, "--nodebug", NULL, 0, "Hide debug rendering primitives." },
	{ &CmdCutaway, "--cutaway", NULL, 0, "Use a cutaway plane when rendering final results." },
	{ &CmdAnimCutaway, "--animcutaway", NULL, 0, "Use an animated cutaway plane when rendering final results." },
	{ &CmdSkip, "--skip", NULL, 0, "Skip visualization and jump to the end." },
	{ &CmdNumPoints, "--numpoints", NULL, 1, "Set the number of points to generate." },
	{ &CmdCellCount, "--cellcount", NULL, 1, "Set number of grid subdivisions to use." },
	{ &CmdPause, "--pause", NULL, 0, "Start paused." },
	{ &CmdPoints, "--points", NULL, 1, "File to read points from." },
	{ &CmdOutput, "--output", "-o", 1, "File where we will write the volume mesh." },
	{ &CmdHelp, "--help", "-h", 0, "Display help." },
};

bool g_bHelped = false;
void CmdHelp(int argc, char **argv)
{	
	printf("Usage: delaunay [options]\n"
	"options may be:\n\n");
	for(int i = 0, c = ARRAY_SIZE(g_options); i < c; ++i)
	{
		CmdOption &opt = g_options[i];
		printf("%s%s%-10s\t%s\n",
			opt.szOptionLong,
			(opt.szOptionShort ? ", " : ""),
			(opt.szOptionShort ? opt.szOptionShort : ""),
			opt.szDesc);
	}
	g_bHelped = true;
}


////////////////////////////////////////////////////////////////////////////////
// Main
int main(int argc, char** argv)
{
	ProcessArguments(g_options, ARRAY_SIZE(g_options), argc, argv);
	if(g_bHelped)
		return 0;

	if(g_bAuto)
		printf("Auto stepping...\n");
	if(!g_bDebugRender)
		printf("Debug rendering disabled.\n");
	if(g_bEnableCutaway)
		printf("Using cutaway plane for final display.\n");
	if(g_bEnableAnimCutaway)
		printf("Aanimating cutaway on completion.\n");
	if(g_bOneStep)
		printf("Skipping to the end of triangulatization.\n");
	if(g_bPaused)
		printf("Starting off paused.\n");
	printf("\n");
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
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

