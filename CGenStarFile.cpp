#include "CMainInc.h"
#include "Align/CAlignInc.h"
#include "Util/CUtilInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <errno.h>
#include <Util/Util_Time.h>

using namespace MotionCor2;

CGenStarFile* CGenStarFile::m_pInstance = 0L;

CGenStarFile* CGenStarFile::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CGenStarFile;
	return m_pInstance;
}

void CGenStarFile::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CGenStarFile::CGenStarFile(void)
{
	m_pFile = 0L;
	pthread_mutex_init(&m_aMutex, NULL);
}

CGenStarFile::~CGenStarFile(void)
{
	this->CloseFile();
	pthread_mutex_destroy(&m_aMutex);
}

void CGenStarFile::CloseFile(void)
{
	if(m_pFile == 0L) return;
	fclose(m_pFile);
	m_pFile = 0L;
	pthread_mutex_unlock(&m_aMutex);
}

void CGenStarFile::OpenFile(char* pcInFile)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iOutStarFile == 0) return;
	//-------------------------------------
	pthread_mutex_lock(&m_aMutex);
	char acStarName[256] = {'\0'};
	strcpy(acStarName, pInput->m_acOutMrcFile);
	char* pcStarMain = strrchr(acStarName, '/');
	if(pcStarMain == 0L) pcStarMain = acStarName;
	else pcStarMain += 1;
	//-------------------
	char* pcInMain = strrchr(pcInFile, '/');
	if(pcInMain == 0L) strcpy(m_acInMain, pcInFile);
	else strcpy(m_acInMain, pcInMain+1);
	//----------------------------------
	strcpy(pcStarMain, m_acInMain);
	char* pcDot = strrchr(pcStarMain, '.');
	if(pcDot == 0L) strcat(pcStarMain, ".star");
	else strcpy(pcDot, ".star");
	//--------------------------
	m_pFile = fopen(acStarName, "wt");
	if(m_pFile == 0L) return;
	//-----------------------
	fprintf(m_pFile, "\n# version 30001\n\n");
	fprintf(m_pFile, "data_general\n\n");
};

void CGenStarFile::SetStackSize(int* piStkSize)
{
	if(m_pFile == 0L) return;
	fprintf(m_pFile, "%40s %-16d\n", "_rlnImageSizeX", piStkSize[0]);
	fprintf(m_pFile, "%40s %-16d\n", "_rlnImageSizeY", piStkSize[1]);
	fprintf(m_pFile, "%40s %-16d\n", "_rlnImageSizeZ", piStkSize[2]);
	fprintf(m_pFile, "%40s %s\n", "_rlnMicrographMovieName", m_acInMain);
	//-------------------------------------------------------------------
	CInput* pInput = CInput::GetInstance();
	char* pcGainFile = 0L;
	if(strlen(pInput->m_acGainMrc) != 0)
	{	pcGainFile = strrchr(pInput->m_acGainMrc, '/');
		if(pcGainFile != 0L) pcGainFile += 1;
		else pcGainFile = pInput->m_acGainMrc;
	}
	if(pcGainFile == 0L) 
	{	fprintf(m_pFile, "%40s\n", "_rlnMicrographGainName");
	}
	else
	{	fprintf(m_pFile, "%40s %s\n", "_rlnMicrographGainName",
		   pcGainFile);
	}
	//---------------------
	fprintf(m_pFile, "%40s %-16.6f\n", "_rlnMicrographOriginalPixelSize",
	   pInput->m_fPixelSize);
	fprintf(m_pFile, "%40s %-16.6f\n", "rlnMicrographBinning",
	   pInput->m_fFourierBin);
	fprintf(m_pFile, "%40s %-16.6f\n", "_rlnMicrographDoseRate",
	   pInput->m_fFmDose);
	fprintf(m_pFile, "%40s %-16.6f\n", "_rlnMicrographPreExposure", 0.0f);
	fprintf(m_pFile, "%40s %-16.6f\n", "_rlnVoltage",
	   (float)pInput->m_iKv);
	fprintf(m_pFile, "%40s %-16d\n","_rlnMicrographStartFrame",
	   pInput->m_aiThrow[0] + 1);
	fprintf(m_pFile, "%40s %-16d\n", "_rlnMotionModelVersion", 0);
	fprintf(m_pFile, "\n");		
}

void CGenStarFile::SetGlobalShifts(float* pfShifts, int iSize)
{
	if(m_pFile == 0L) return;
	fprintf(m_pFile, "# version 30001\n\n");
	fprintf(m_pFile, "data_global_shift\n\n");
	fprintf(m_pFile, "loop\n");
	fprintf(m_pFile, "_rlnMicrographFrameNumber #1\n");
	fprintf(m_pFile, "_rlnMicrographShiftX #2\n");
	fprintf(m_pFile, "_rlnMicrographShiftY #3\n");
	//--------------------------------------------
	float* pfShiftYs = pfShifts + iSize;
	float fRefX = pfShifts[0];
	float fRefY = pfShiftYs[0];
	//-------------------------
	for(int i=0; i<iSize; i++)
	{	float fSx = pfShifts[i] - pfShifts[0];
		float fSy = pfShiftYs[i] - pfShiftYs[0];
		fprintf(m_pFile, "%13d %13.6f %13.6f\n", i+1, fSx, fSy);
	}
	fprintf(m_pFile, "\n");
}

void CGenStarFile::SetHotPixels
(	unsigned char* pucBadMap, 
	int* piMapSize,
	bool bPadded
)
{	if(m_pFile == 0L) return;
	fprintf(m_pFile, "data_hot_pixels\n\n");
	fprintf(m_pFile, "loop\n");
	fprintf(m_pFile, "_rlnCoordinateX #1\n");
	fprintf(m_pFile, "_rlnCoordinateY #2\n");
	int iSizeX = bPadded ? (piMapSize[0] / 2 - 1) * 2 : piMapSize[0];
	for(int y=0; y<piMapSize[1]; y++)
	{	int i = y * piMapSize[0];
		int iY = piMapSize[1] - y; // Daniel
		for(int x=0; x<iSizeX; x++)
		{	if(pucBadMap[i+x] == 0) continue;
			fprintf(m_pFile, "%13.6f  %13.6f\n", 
			   x + 1.0f, iY);
		}
	}
	fprintf(m_pFile, "\n\n");
}		 
