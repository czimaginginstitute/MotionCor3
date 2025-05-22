#include "CDataUtilInc.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>

using namespace MotionCor2;
using namespace MotionCor2::DataUtil;

static pthread_mutex_t s_mutex;

CSaveMovieDone* CSaveMovieDone::m_pInstance = 0L;

CSaveMovieDone* CSaveMovieDone::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CSaveMovieDone;
	return m_pInstance;
}

void CSaveMovieDone::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

//------------------------------------------------------------------------------
// m_aMutex: It is initialized in Util_Thread. Do not init here
//------------------------------------------------------------------------------
CSaveMovieDone::CSaveMovieDone(void)
{
	m_pLogFile = 0L;
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iSerial == 0) return;
	//---------------------------
	char acLogFile[256] = {'\0'};
	strcpy(acLogFile, pInput->m_acOutMrcFile);
	char* pcSlash = strrchr(acLogFile, '/');
	if(pcSlash != 0L)
	{	strcpy(pcSlash, "/MoviesDone.txt");
	}
	else
	{	strcpy(acLogFile, "./MoviesDone.txt");
	}
	m_pLogFile = fopen(acLogFile, "wt");
	//-----------------
	pthread_mutex_init(&s_mutex, 0L);
}

//------------------------------------------------------------------------------
// m_aMutex: It is destroyed in Util_Thread. Do not destroy here.
//------------------------------------------------------------------------------
CSaveMovieDone::~CSaveMovieDone(void)
{
	pthread_mutex_destroy(&s_mutex);
	if(m_pLogFile != 0L) fclose(m_pLogFile);
}

void CSaveMovieDone::DoStart(const char* pcMovieFile)
{
	mDoIt(pcMovieFile, "start");
}

void CSaveMovieDone::DoEnd(const char* pcMovieFile)
{
	mDoIt(pcMovieFile, "done");
}

void CSaveMovieDone::mDoIt(const char* pcMovieFile, const char* pcStatus)
{
	if(m_pLogFile == 0L) return;
	pthread_mutex_lock(&s_mutex);
	//-----------------
	const char* pcMainFile = strrchr(pcMovieFile, '/');
	if(pcMainFile != 0L)
	{	int iSize = strlen(pcMainFile);
		if(iSize > 1) 
		{	fprintf(m_pLogFile, "%s: %s\n", 
			   &pcMainFile[1], pcStatus);
		}
	}
	else
	{	fprintf(m_pLogFile, "%s: %s\n", pcMovieFile, pcStatus);
	}
	fflush(m_pLogFile);
	//-----------------
	pthread_mutex_unlock(&s_mutex);
}
