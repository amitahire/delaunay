#include "common.hh"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <ctime>
#include "math/math.hh"
#include "ply.hh"
#include "cmdhelper.hh"
#include "he_trimesh.hh"

////////////////////////////////////////////////////////////////////////////////
// File Globals
static const char * g_volMeshFilename;
static const char * g_triMeshFilename;
static bool g_renderingEnabled = false;
static HETriMesh * g_triMesh;
static timespec g_lastTime;					// last time, used for calculating dt
static int g_width, g_height;
static float g_clipMeshScale = 1.f;

////////////////////////////////////////////////////////////////////////////////
// Command line setup
void CmdVolMesh(int, char**);
void CmdTriMesh(int, char**);
void CmdGraphics(int, char**);
void CmdHelp(int, char**);
void CmdScale(int, char**);

static CmdOption g_options[] =
{
	{ &CmdVolMesh, "--volmesh", "-v", 1, "File containing the volume mesh dump." },
	{ &CmdTriMesh, "--mesh", "-m", 1, "File containing the triangle mesh to clip against (PLY)." },
	{ &CmdGraphics, "--graphics", "-g", 0, "Interactive render of results." },
	{ &CmdScale, "--scale", "-s", 1, "Amount to scale clip mesh by." },
	{ &CmdHelp, "--help", "-h", 0, "Display help." },
};

void CmdVolMesh(int argc, char** argv)
{
	g_volMeshFilename = argv[1];
}

void CmdTriMesh(int argc, char** argv)
{
	g_triMeshFilename = argv[1];
}

void CmdGraphics(int, char**)
{
	g_renderingEnabled = true;
}

void CmdScale(int argc, char** argv)
{
	g_clipMeshScale = atof(argv[1]);
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

HETriMesh * ReadMesh(const char* filename);
VolMesh * ReadVolumeMesh(const char* filename);

void SetupGL();
void OnIdle(void);
void OnReshape(int width, int height);
void OnDisplay(void);
void OnKeyboard(unsigned char key, int x, int y);
void OnSpecialKeyboard(int key, int x, int y);
void OnPassiveMotion(int x, int y);
void OnMotion(int x, int y);

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
	HETriMesh * clipMesh = ReadMesh(g_triMeshFilename);
	if(!clipMesh)
	{
		printf("Failed to read clipping mesh.\n");
		return 1;
	}

	if(g_renderingEnabled)
	{
		g_triMesh = clipMesh;
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

HETriMesh* ReadMesh(const char* filename)
{
	FILE* fp = fopen(filename, "rb");
	
	if(fp == 0)
	{
		printf("Failed to open file %s\n", filename);
		return 0;
	}

	PlyData ply;
	HETriMesh * result = 0;
	if(ply.Read(fp))
	{
		result = new HETriMesh(HETriMesh::OPT_TRYMATCH);
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
				int totalFloatingTris = 0;
				for(int i = 0, c = faceReader.GetCount(); i < c; ++i)
				{
					int count = faceReader.GetListSize(idxId, i);
					if(count == 3)
					{
						int v0 = faceReader.GetPropertyListValue<int>(idxId, i, 0);
						int v1 = faceReader.GetPropertyListValue<int>(idxId, i, 1);
						int v2 = faceReader.GetPropertyListValue<int>(idxId, i, 2);

						if(result->AddFace(v0, v1, v2) < 0)
						{
							// create a floating triangle
							v0 = result->AddVertex( result->GetVertexPos(v0) );
							v1 = result->AddVertex( result->GetVertexPos(v1) );
							v2 = result->AddVertex( result->GetVertexPos(v2) );
							if(result->AddFace(v0, v1, v2) < 0)
							{
								printf("Failed to create floating triangle.\n");
							}

							++totalFloatingTris;
						}
					}
					else printf("WARNING: primitive with %d verts not supported.\n", count);

				}
				if(totalFloatingTris > 0)
					printf("created %d floating faces.\n", totalFloatingTris);
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

VolMesh * ReadVolumeMesh(const char* filename)
{
	return 0;
}

////////////////////////////////////////////////////////////////////////////////
// OpenGL and GLUT stuff
static float g_eyeDist = 310.f, 				// Camera parameters.
	g_pitch = M_PI / 4.f, 
	g_yaw = M_PI / 4.f;
static float g_eyeDistTarget = 310.f;			
static Vec3 g_center(0,0,0);
static Vec3 g_centerTarget(0.f, 0.f, 0.f);		// camera center to lerp to
static int g_lastx = -1, g_lasty = -1;
static bool g_showBackface = true;

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
		return;

	float cos_pitch = cos(g_pitch);
	float eyeX = g_eyeDist * cos_pitch * cos(g_yaw);
	float eyeY = g_eyeDist * sin(g_pitch);
	float eyeZ = g_eyeDist * cos_pitch * sin(g_yaw);

	glLoadIdentity();
	gluLookAt(eyeX + g_center.x, eyeY + g_center.y, eyeZ + g_center.z, g_center.x, g_center.y, g_center.z, 0, 1, 0);
		
	glEnable(GL_CULL_FACE);
	glColor4f(0.8f, 0.8f, 0.8f, 1.f);

	glBegin(GL_TRIANGLES);
	for(int i = 0, c = g_triMesh->NumFaces(); i < c; ++i)
	{
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
	
	glutSwapBuffers();
}

void OnKeyboard(unsigned char key, int x, int y)
{
	if(key == ' ')
	{
		g_showBackface = !g_showBackface;
		glutPostRedisplay();
	}
}

void OnSpecialKeyboard(int key, int x, int y)
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


