#include "common.hh"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <ctime>
#include "math/math.hh"
#include "ply.hh"
#include "cmdhelper.hh"
#include "trisoup.hh"
#include "debugdraw.hh"
#include "sgndist.hh"

enum AppState
{
	APPSTATE_LOAD,
	APPSTATE_DISPLAY
};

////////////////////////////////////////////////////////////////////////////////
// File Globals
static const char * g_volMeshFilename;
static const char * g_triMeshFilename;
static bool g_renderingEnabled = false;
static PlyData * g_ply;
static TriSoup * g_triMesh;
static timespec g_lastTime;					// last time, used for calculating dt
static int g_width, g_height;
static float g_clipMeshScale = 1.f;
static bool g_visualizeMode = false;
static AppState g_appState = APPSTATE_DISPLAY;
static bool g_stepAllowed = false;
static int g_currentFace = -1;
static int g_currentFaceVerts[3];
static bool g_justShow = true;
static float g_eyeDistTarget = 310.f;			
static Vec3 g_centerTarget(0.f, 0.f, 0.f);		// camera center to lerp to
static SignedDistanceField* g_distField;
static bool g_showMesh = true;
static bool g_showDistField = false;

////////////////////////////////////////////////////////////////////////////////
// Command line setup
void CmdVolMesh(int, char**);
void CmdTriMesh(int, char**);
void CmdGraphics(int, char**);
void CmdHelp(int, char**);
void CmdScale(int, char**);
void CmdVisMode(int, char**);

static CmdOption g_options[] =
{
	{ &CmdVolMesh, "--volmesh", "-v", 1, "File containing the volume mesh dump." },
	{ &CmdTriMesh, "--mesh", "-m", 1, "File containing the triangle mesh to clip against (PLY)." },
	{ &CmdGraphics, "--graphics", "-g", 0, "Interactive render of results." },
	{ &CmdScale, "--scale", "-s", 1, "Amount to scale clip mesh by." },
	{ &CmdVisMode, "--vis", 0, 0, "Vis mode with stepping." },
	{ &CmdHelp, "--help", "-h", 0, "Display help." },
};

void CmdVolMesh(int, char** argv)
{
	g_volMeshFilename = argv[1];
}

void CmdTriMesh(int, char** argv)
{
	g_triMeshFilename = argv[1];
}

void CmdGraphics(int, char**)
{
	g_renderingEnabled = true;
}

void CmdScale(int, char** argv)
{
	g_clipMeshScale = atof(argv[1]);
}

void CmdVisMode(int, char**)
{
	g_visualizeMode = true;
	g_renderingEnabled = true;
}

void CmdHelp(int, char**)
{
	printf("Usage: tetclip [options]\n"
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
}

////////////////////////////////////////////////////////////////////////////////
struct VolMesh;

TriSoup * ReadMesh(const char* filename);
bool ReadMeshStep(PlyData& ply);
VolMesh * ReadVolumeMesh(const char* filename);

void SetupGL();
void OnIdle(void);
void OnReshape(int width, int height);
void OnDisplay(void);
void OnKeyboard(unsigned char key, int x, int y);
void OnSpecialKeyboard(int key, int x, int y);
void OnPassiveMotion(int x, int y);
void OnMotion(int x, int y);
void FindFloating(bool forward);

int main(int argc, char **argv)
{
	ProcessArguments(g_options, ARRAY_SIZE(g_options), argc, argv);

	if(!g_volMeshFilename)
	{
		printf("No volume mesh specified.\n");
		CmdHelp(0, NULL);
		return 0;
	}

	if(!g_triMeshFilename)
	{
		printf("No triangle mesh specified.\n");
		CmdHelp(0, NULL);
		return 0;
	}

//	VolMesh * volMesh = ReadVolumeMesh(g_volMeshFilename);
//	if(!volMesh)
//	{
//		printf("Failed to read volume mesh.\n");
//		return 1;
//	}
	TriSoup * clipMesh = 0;
	if(g_visualizeMode)
	{
		g_appState = APPSTATE_LOAD;
		FILE* fp = fopen(g_triMeshFilename, "rb");
		if(fp == 0)
		{
			printf("Failed to open file %s\n", g_triMeshFilename);
			return 0;
		}
		g_ply = new PlyData();
		if(!g_ply->Read(fp))
		{
			delete g_ply;
			g_ply = 0;
		}

		fclose(fp);
	}
	else
	{
		clipMesh = ReadMesh(g_triMeshFilename);
		if(!clipMesh)
		{
			printf("Failed to read clipping mesh.\n");
			return 1;
		}
		g_triMesh = clipMesh;
		g_distField = new SignedDistanceField(*g_triMesh, 0.25f);
	}

	if(g_renderingEnabled)
	{
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
		glutCreateWindow(argv[0]);
		g_width = 800; g_height = 800;
		glutInitWindowSize( g_width, g_height );

		glutIdleFunc( OnIdle );
		glutReshapeFunc( OnReshape );
		glutDisplayFunc( OnDisplay );
		glutKeyboardFunc( OnKeyboard );
		glutSpecialFunc( OnSpecialKeyboard );
		glutMotionFunc( OnMotion );
		glutPassiveMotionFunc( OnPassiveMotion );

		SetupGL();

		clock_gettime(CLOCK_MONOTONIC, &g_lastTime);
		glutMainLoop();
	}

	delete clipMesh;

	return 0;
}

bool ReadMeshStep(PlyData& ply)
{
	if(!g_triMesh)
	{
		g_triMesh = new TriSoup();

		if(g_triMesh == 0)
			return true;

		PlyData::ElementReader vertexReader = ply.GetElement("vertex");
		if(vertexReader.Valid())
		{
			int xId = vertexReader.GetPropertyId("x");
			int yId = vertexReader.GetPropertyId("y");
			int zId = vertexReader.GetPropertyId("z");
			if(xId >= 0 && yId >= 0 && zId >= 0)
			{
				for(int i = 0, c = vertexReader.GetCount(); i < c; ++i)
				{
					float x = vertexReader.GetPropertyValue<float>(xId, i), 
						  y = vertexReader.GetPropertyValue<float>(yId, i), 
						  z = vertexReader.GetPropertyValue<float>(zId, i);

					g_triMesh->AddVertex( g_clipMeshScale * Vec3(x,y,z) );
				}
			}
			else printf("Couldn't find x y or z properties in vertex\n");
		}

		g_currentFace = 0;
	}

	PlyData::ElementReader faceReader = ply.GetElement("face");
	if(faceReader.Valid() && g_currentFace < faceReader.GetCount())
	{
		int idxId = faceReader.GetPropertyId("vertex_indices");
		if(idxId >= 0)
		{
			int i = g_currentFace;
			int count = faceReader.GetListSize(idxId, i);
			if(count == 3)
			{
				int v0 = faceReader.GetPropertyListValue<int>(idxId, i, 0);
				int v1 = faceReader.GetPropertyListValue<int>(idxId, i, 1);
				int v2 = faceReader.GetPropertyListValue<int>(idxId, i, 2);
				int verts[] = { v0, v1, v2 };

				memcpy(g_currentFaceVerts, verts, sizeof(int) * ARRAY_SIZE(g_currentFaceVerts));

				g_justShow = !g_justShow;
				if(!g_justShow)
					return true;

				g_triMesh->AddFace(v0, v1, v2);

				g_centerTarget = g_triMesh->GetVertexPos(g_currentFaceVerts[0]);
				g_eyeDistTarget = 10.f;
			}
			else printf("WARNING: primitive with %d verts not supported.\n", count);
			++g_currentFace;
			return g_currentFace < faceReader.GetCount() ;
		}
		else printf("Couldn't find 'vertex_indices\n");
	}

	return false;
}

TriSoup* ReadMesh(const char* filename)
{
	FILE* fp = fopen(filename, "rb");
	
	if(fp == 0)
	{
		printf("Failed to open file %s\n", filename);
		return 0;
	}

	PlyData ply;
	TriSoup* result = 0;
	if(ply.Read(fp))
	{
		result = new TriSoup();
		PlyData::ElementReader vertexReader = ply.GetElement("vertex");
		if(vertexReader.Valid())
		{
			int xId = vertexReader.GetPropertyId("x");
			int yId = vertexReader.GetPropertyId("y");
			int zId = vertexReader.GetPropertyId("z");
			if(xId >= 0 && yId >= 0 && zId >= 0)
			{
				for(int i = 0, c = vertexReader.GetCount(); i < c; ++i)
				{
					float x = vertexReader.GetPropertyValue<float>(xId, i), 
						  y = vertexReader.GetPropertyValue<float>(yId, i), 
						  z = vertexReader.GetPropertyValue<float>(zId, i);

					result->AddVertex( g_clipMeshScale * Vec3(x,y,z) );
				}
			}
			else printf("Couldn't find x y or z properties in vertex\n");
		}

		PlyData::ElementReader faceReader = ply.GetElement("face");
		if(faceReader.Valid())
		{
			int idxId = faceReader.GetPropertyId("vertex_indices");
			if(idxId >= 0)
			{
				for(int i = 0, c = faceReader.GetCount(); i < c; ++i)
				{
					int count = faceReader.GetListSize(idxId, i);
					if(count == 3)
					{
						int v0 = faceReader.GetPropertyListValue<int>(idxId, i, 0);
						int v1 = faceReader.GetPropertyListValue<int>(idxId, i, 1);
						int v2 = faceReader.GetPropertyListValue<int>(idxId, i, 2);

						result->AddFace(v0, v1, v2);
					}
					else printf("WARNING: primitive with %d verts not supported.\n", count);

				}
			}
			else printf("Couldn't find 'vertex_indices\n");
		}
	}
	else
	{
		printf("failed to parse %s as PLY file\n", filename);
	}
	fclose(fp);


	return result;
}

VolMesh * ReadVolumeMesh(const char*)
{
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// OpenGL and GLUT stuff
static float g_eyeDist = 310.f, 				// Camera parameters.
	g_pitch = M_PI / 4.f, 
	g_yaw = M_PI / 4.f;
static Vec3 g_center(0,0,0);
static int g_lastx = -1, g_lasty = -1;
static bool g_showBackface = true;
static bool g_showProblemFaces = false;
static int g_currentFloating = -1;

void SetupGL()
{
	glClearColor(0.1f, 0.1f, 0.1f, 1.f);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glShadeModel(GL_FLAT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	OnReshape(800, 800);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	

	if(g_triMesh)
	{
		Vec3 avgPoint(0,0,0);
		AABB bounds;
		float avgFactor = 1.f / g_triMesh->NumVertices();

		for(int i = 0, c = g_triMesh->NumVertices(); i < c; ++i)
		{
			const Vec3& pt = g_triMesh->GetVertexPos(i);
			bounds.Extend(pt);
			avgPoint += avgFactor * pt;
		}

		g_center = g_centerTarget = avgPoint;
		g_eyeDist = g_eyeDistTarget = magnitude(bounds.m_max - bounds.m_min);
		printf("Bounds: <%f, %f, %f> to <%f, %f, %f>\n", 
			bounds.m_min.x, bounds.m_min.y, bounds.m_min.z, 
			bounds.m_max.x, bounds.m_max.y, bounds.m_max.z);

	}
}

void OnIdle(void)
{
	timespec currentTime;
	clock_gettime(CLOCK_MONOTONIC, &currentTime);

	time_t diffSec = currentTime.tv_sec - g_lastTime.tv_sec;
	long diffNsec = currentTime.tv_nsec - g_lastTime.tv_nsec;

	long diffMsec = diffSec * 1000 + diffNsec / (1000 * 1000);
	float dt = diffMsec / 1000.f;


	if(dt > 1/30.f)
	{
		g_lastTime = currentTime;
		float t = 0.9f * dt;

		g_center = t * g_centerTarget + (1.f - t) * g_center;
		g_eyeDist = t * g_eyeDistTarget + (1.f - t) * g_eyeDist;
		if(magnitude_squared(g_center - g_centerTarget) > EPSILON_SQ ||
			(fabs(g_eyeDistTarget - g_eyeDist) > EPSILON))
		{
			glutPostRedisplay();
		}
	}

	switch(g_appState)
	{
	case APPSTATE_LOAD:
		{
			if(g_stepAllowed)
			{
				g_stepAllowed = false;
				ClearDebugDraw();
				if(g_ply == 0 || !ReadMeshStep(*g_ply))
				{
					ClearDebugDraw();
					g_appState = AppState(int(g_appState) + 1);
				}
				glutPostRedisplay();
			}
		}
		break;

	case APPSTATE_DISPLAY:
		break;
	default:
		break;
	}
}

void OnReshape(int width, int height)
{
	g_width = width;
	g_height = height;
	glViewport(0, 0, g_width, g_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.f, g_width / (float)g_height, 1.f, 1000.f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glutPostRedisplay();
}

void OnDisplay(void)
{
	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if(g_triMesh == 0)
	{	
		glutSwapBuffers();
		return;
	}

	float cos_pitch = cos(g_pitch);
	float eyeX = g_eyeDist * cos_pitch * cos(g_yaw);
	float eyeY = g_eyeDist * sin(g_pitch);
	float eyeZ = g_eyeDist * cos_pitch * sin(g_yaw);

	glLoadIdentity();
	gluLookAt(eyeX + g_center.x, eyeY + g_center.y, eyeZ + g_center.z, g_center.x, g_center.y, g_center.z, 0, 1, 0);
		
	glEnable(GL_CULL_FACE);

	if(g_showMesh)
	{
		glBegin(GL_TRIANGLES);
		for(int i = 0, c = g_triMesh->NumFaces(); i < c; ++i)
		{
			float r = 0.8f, g = 0.8f, b = 0.8f, a = 1.f;
			glColor4f(r,g,b,a);

			int verts[3];
			g_triMesh->GetFace(i, verts);
			const Vec3& pt0 = g_triMesh->GetVertexPos(verts[0]);
			const Vec3& pt1 = g_triMesh->GetVertexPos(verts[1]);
			const Vec3& pt2 = g_triMesh->GetVertexPos(verts[2]);
			Vec3 normal = normalize(cross(pt1 - pt0, pt2 - pt0));
			glNormal3fv(&normal.x);
			glVertex3fv(&pt0.x);
			glVertex3fv(&pt1.x);
			glVertex3fv(&pt2.x);
		}

		if(g_showBackface)
		{
			for(int i = 0, c = g_triMesh->NumFaces(); i < c; ++i)
			{
				float r = 0.8f, g = 0.8f, b = 0.8f, a = 1.f;
				glColor4f(r,g,b,a);
				int verts[3];
				g_triMesh->GetFace(i, verts);
				const Vec3& pt0 = g_triMesh->GetVertexPos(verts[0]);
				const Vec3& pt1 = g_triMesh->GetVertexPos(verts[1]);
				const Vec3& pt2 = g_triMesh->GetVertexPos(verts[2]);
				Vec3 normal = normalize(cross(pt2 - pt0, pt1 - pt0));
				glNormal3fv(&normal.x);
				glVertex3fv(&pt0.x);
				glVertex3fv(&pt2.x);
				glVertex3fv(&pt1.x);
			}
		}

		glEnd();

		if(g_appState == APPSTATE_LOAD && g_currentFace >= 0)
		{
			glLineWidth(3.f);
			glDisable(GL_LIGHTING);
			glDisable(GL_DEPTH_TEST);
			glBegin(GL_LINES);
			glColor4f(1, 0, 0, 0.2f);
			for(int i = 0; i < (int)ARRAY_SIZE(g_currentFaceVerts); ++i)
			{
				int next = (i+1) % ARRAY_SIZE(g_currentFaceVerts);
				const Vec3& pt0 = g_triMesh->GetVertexPos(g_currentFaceVerts[i]);
				const Vec3& pt1 = g_triMesh->GetVertexPos(g_currentFaceVerts[next]);
				glVertex3fv(&pt0.x);
				glVertex3fv(&pt1.x);
			}
			glEnd();

			glEnable(GL_LIGHTING);
			glEnable(GL_DEPTH_TEST);
		}
	}

	if(g_showDistField && g_distField)
	{
		SignedDistanceField::Iterator iter = g_distField->GetFirst();
		glDisable(GL_LIGHTING);
		glPointSize(3.f);
		glBegin(GL_POINTS);
		while(iter.Valid())
		{
			const Vec3& pt = iter.GetCenter();
			float dist = iter.GetDistance();
			if(dist != FLT_MAX && dist != -FLT_MAX)
			{
				float intensity = Clamp(((fabsf(dist) / 25.f) + 1.f) / 2.f, 0.f, 1.f);
				glColor3f(dist > 0.f ? intensity : 0.f, 0.f, dist < 0.f ? intensity : 0.f);
				glVertex3fv(&pt.x);
			}
			
			iter.NextVoxel();
		}
		glEnd();
		glPointSize(1.f);
		glEnable(GL_LIGHTING);
	}

	RenderDebugDraw();
	glutSwapBuffers();
}

void OnKeyboard(unsigned char key, int x, int y)
{
	(void)x; (void)y;
	if(key == ' ')
	{
		g_showBackface = !g_showBackface;
		glutPostRedisplay();
	}
	//else if(key == 'n')
	//{
	//	FindFloating(true);
	//}
	//else if(key == 'p')
	//{
	//	FindFloating(false);
	//}
	else if(key == 'f')
	{
		g_showProblemFaces = !g_showProblemFaces;
		glutPostRedisplay();
	}
	else if(key == 's')
	{
		g_stepAllowed = true;
	}
	else if(key == 'h')
	{
		g_showMesh = !g_showMesh;
		glutPostRedisplay();
	}
	else if(key == 'd')
	{
		g_showDistField = !g_showDistField;
		glutPostRedisplay();
	}
}

void OnSpecialKeyboard(int key, int x, int y)
{
	(void)x; (void)y;
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

void OnPassiveMotion(int x, int y)
{
	g_lastx = x;
	g_lasty = y;
}

void OnMotion(int x, int y)
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
void FindFloating(bool forward)
{
	if(!g_triMesh) return;
	int count = g_triMesh->NumFaces();
	int cur = g_currentFloating;
	if(cur == -1) cur = 0;
	else cur = (forward ? (cur + 1) % count : (cur + count - 1) % count);
	int i;
	for(i = 0; i < count; ++i)
	{
		const char * data = g_triMesh->GetFaceData(cur);
		if(data)
		{
			const bool * boolData = reinterpret_cast<const bool*>(data);
			if(*boolData) {
				g_currentFloating = cur;
				break;
			}
		}

		cur = (forward ? (cur + 1) % count : (cur + count - 1) % count);
	}
	if(i == count)
	{
		g_currentFloating = -1;
		Vec3 avgPoint(0,0,0);
		float avgFactor = 1.f / g_triMesh->NumVertices();

		for(int i = 0, c = g_triMesh->NumVertices(); i < c; ++i)
		{
			const Vec3& pt = g_triMesh->GetVertexPos(i);
			avgPoint += avgFactor * pt;
		}
		g_centerTarget = avgPoint;
	}
	else
	{
		int verts[3];
		g_triMesh->GetFace(g_currentFloating, verts);
		float factor = 1.f / 3.f;
		g_centerTarget = Vec3(0,0,0);
		for(int i = 0; i < 3; ++i)
		{
			g_centerTarget += factor * g_triMesh->GetVertexPos(verts[i]);
		}	
	}
}

