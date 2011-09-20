#include "cmdhelper.hh"
#include <cstdio>
#include <strings.h>

void ProcessArguments(CmdOption *opts, int numOpts, int argc, char **argv)
{
	for(int i = 1; i < argc; ++i)
	{
		const char *cmd = argv[i];
		bool found = false;
		for(int iOpt = 0; iOpt < numOpts; ++iOpt)
		{
			CmdOption &opt = opts[iOpt];
			if(strcasecmp(opt.szOptionLong, cmd) == 0 ||
				(opt.szOptionShort && strcasecmp(opt.szOptionShort, cmd) == 0))
			{
				found = true;
				int numArgs = i+1;
				while(numArgs < argc && argv[numArgs][0] != '-') ++numArgs;
				numArgs -= i;
				
				if(opt.numArgs < numArgs)
				{
					(opt.parseFunc)(numArgs, &argv[i]);
				}
				else
				{
					printf("Not enough arguments for option %s. It requires %d parameters.\n",
						opt.szOptionLong, opt.numArgs);
				}
				i += numArgs - 1;

				if(found)
					break;
			}
		}
		if(!found)
		{
			printf("WARN: Ignoring unrecognized option: %s\n", cmd);
		}
	}
}
