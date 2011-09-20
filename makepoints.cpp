#include "common.hh"
#include <strings.h>
#include "cmdhelper.hh"
#include "math/math.hh"
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// Global state 
enum GenType
{
	GENTYPE_UNIFORM,
};

static GenType g_genType = GENTYPE_UNIFORM;
static int g_numPoints;
static float g_rangeMinMax;
static int g_seed;
static char *g_szOutputFile;

////////////////////////////////////////////////////////////////////////////////
// Generation functions
void GenerateUniformPoints(std::vector<Vec3>& points, int num, float rangeMinMax)
{
	points.clear();
	points.resize(num);

	unsigned int seed = g_seed;
	const float inv_randmax = 1.f/RAND_MAX;
	for(int i = 0, c = num; i < c; ++i)
	{
		int ix = rand_r(&seed);
		int iy = rand_r(&seed);
		int iz = rand_r(&seed);

		Vec3 &pt = points[i];
		pt.x = ((ix * inv_randmax) * 2.f - 1.f) * rangeMinMax;
		pt.y = ((iy * inv_randmax) * 2.f - 1.f) * rangeMinMax;
		pt.z = ((iz * inv_randmax) * 2.f - 1.f) * rangeMinMax;
	}
}

////////////////////////////////////////////////////////////////////////////////
// Command line setup
void CmdGenType(int, char**);
void CmdSeed(int, char**);
void CmdNumPoints(int, char**);
void CmdRangeMinMax(int, char**);
void CmdOutputFile(int, char**);
void CmdHelp(int, char**);
static CmdOption g_options[] =
{
	{ &CmdGenType, "--gentype", "-t", 1, "Type of point cloud to generate: [uniform]" },
	{ &CmdSeed, "--seed", NULL, 1, "Random number seed." },
	{ &CmdNumPoints, "--num", "-n", 1, "Number of points to generate." },
	{ &CmdRangeMinMax, "--range", NULL, 1, "range to generate points (from -value to value)." },
	{ &CmdOutputFile, "--output", "-o", 1, "File to write points to." },
	{ &CmdHelp, "--help", "-h", 0, "Display help." },
};

void CmdSeed(int argc, char** argv)
{
	g_seed = atoi(argv[1]);
}

void CmdNumPoints(int argc, char** argv)
{
	g_numPoints = atoi(argv[1]);
}

void CmdRangeMinMax(int argc, char** argv)
{
	g_rangeMinMax = atof(argv[1]);
}

void CmdGenType(int argc, char** argv)
{
	for(int i = 1; i < argc; ++i)
	{
		if(strcasecmp(argv[i], "uniform") == 0)
		{
			g_genType = GENTYPE_UNIFORM;
		}
		else
		{
			printf("Unrecognized gen type: %s\n", argv[i]);
		}
	}
}

void CmdOutputFile(int argc, char **argv)
{
	g_szOutputFile = argv[1];
}

void CmdHelp(int argc, char **argv)
{	
	printf("Usage: makepoints [options]\n"
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
// Main
int main(int argc, char** argv)
{
	ProcessArguments(g_options, ARRAY_SIZE(g_options), argc, argv);

	if(g_numPoints == 0)
	{
		CmdHelp(0, NULL);
		return 0;
	}

	if(g_szOutputFile == 0)
	{
		printf("No output file specified.\n");
		CmdHelp(0, NULL);
		return 0;
	}

	std::vector<Vec3> points;
	switch(g_genType)
	{
	case GENTYPE_UNIFORM:
		GenerateUniformPoints(points, g_numPoints, g_rangeMinMax);
		break;
	default:
		break;
	}

	FILE* fp = fopen(g_szOutputFile, "wb");
	if(!fp)
	{
		printf("Could not open %s\n", g_szOutputFile);
		return 1;
	}

	for(int i = 0, c = points.size(); i < c; ++i)
	{
		Vec3& pt = points[i];
		fprintf(fp, "%f %f %f\n", pt.x, pt.y, pt.z);
	}

	fclose(fp);

	printf("Wrote %d points to %s.\n", int(points.size()), g_szOutputFile);
	
	return 0;
}

