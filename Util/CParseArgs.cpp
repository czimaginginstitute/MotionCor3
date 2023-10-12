#include "CUtilInc.h"
#include <memory.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>

using namespace MotionCor2::Util;

CParseArgs::CParseArgs(void)
{
}

CParseArgs::~CParseArgs(void)
{
}

void CParseArgs::Set(int argc, char* argv[])
{
	m_argc = argc;
	m_argv = argv;
}

bool CParseArgs::FindVals(const char* pcTag, int aiRange[2])
{
	aiRange[0] = -1;
	aiRange[1] = 0;
	for(int i=1; i<m_argc; i++)
	{	if(strcasecmp(m_argv[i], pcTag) != 0) continue;
		aiRange[0] = i + 1;
		break;
	}
	if(aiRange[0] == -1) return false;
	//--------------------------------
	char acBuf[256] = {0};
	for(int i=aiRange[0]; i<m_argc; i++)
	{	char* argv = m_argv[i];
		if(argv[0] == '-' && strlen(argv) >= 2)
		{	if(isalpha(argv[1]) != 0) break;
		}
		strcpy(acBuf, argv);
		char* pcTok = strtok(acBuf, ", ");
		while(pcTok != 0L)
		{	aiRange[1] += 1;
			pcTok = strtok(0L, ", ");
		}
	}
	//----------------------
	if(aiRange[1] > 0) return true;
	else return false;
}

void CParseArgs::GetVals(int aiRange[2], float* pfVals)
{
	if(aiRange[0] == -1) return;
	else if(aiRange[1] == 0) return;
	//------------------------------
	for(int i=0; i<aiRange[1]; i++)
	{	int j = aiRange[0] + i;
		if(j >= m_argc) return;
		sscanf(m_argv[j], "%f", pfVals + i);
	}
}

void CParseArgs::GetVals(int aiRange[2], int* piVals)
{
	if(aiRange[0] == -1) return;
        else if(aiRange[1] == 0) return;
        //------------------------------
        int iCount = 0;
	char acBuf[256];
	for(int i=0; i<aiRange[1]; i++)
	{	int j = aiRange[0] + i;
		if(j >= m_argc) return;
		//---------------------
		strcpy(acBuf, m_argv[j]);
		char* pcTok = strtok(acBuf, ", ");
		while(pcTok != 0L)
		{	sscanf(pcTok, "%d", piVals + iCount);
			iCount++;
			if(iCount == aiRange[1]) return;
			pcTok = strtok(0L, ", ");
		}
	}
}

void CParseArgs::GetVals(int aiRange[2], char** ppcVals)
{
	if(aiRange[0] == -1) return;
        else if(aiRange[1] == 0) return;
        //------------------------------
	for(int i=0; i<aiRange[1]; i++)
	{	int j = aiRange[0] + i;
		if(j >= m_argc) return;
		strcpy(ppcVals[i], m_argv[j]);
	}
}

void CParseArgs::GetVal(int iArg, char* pcVal)
{
	if(iArg <= 0 || iArg >= m_argc) return;
	strcpy(pcVal, m_argv[iArg]);
}
