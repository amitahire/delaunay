#pragma once

struct CmdOption
{
	void (*parseFunc)(int argc, char** argv);
	const char * szOptionLong;
	const char * szOptionShort;
	int numArgs;
	const char * szDesc;
};

void ProcessArguments(CmdOption *opts, int numOpts, int argc, char **argv);


