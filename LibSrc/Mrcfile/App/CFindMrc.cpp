#include "../Include/LoadMainHeader.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <math.h>

using namespace Mrc;

CFindMrc::CFindMrc()
{
	m_pcFileName = new char[256];
	m_iNumSections = 0;
}

CFindMrc::~CFindMrc()
{
	if(m_pcFileName != NULL) delete[] m_pcFileName;
}

void CFindMrc::SetFileName(char* pcFileName)
{
	strcpy(m_pcFileName, "");
	m_iNumSections = 0;
	if(pcFileName == NULL) return;
	else strcpy(m_pcFileName, pcFileName);
	//------------------------------------
	int iFile = mOpenFile(m_cFileName);
	if(iFile == -1) return;
	//---------------------
	CLoadMainHeader aLoadMain;
	aLoadMain.DoIt(iFile);
	m_iNumSections = aLoadMain.GetSizeZ();
	close(iFile);
}

int CFindMrc::FindFromStage(float fStageX, float fStageY)
{
	int iFile = mOpenFile();
	if(iFile == -1) return -1;
	//------------------------
	CLoadExtHeader aLoadExt;
	aLoadExt.SetFile(iFile);
	//----------------------
	int iSection = -1;
	double dMin = 1e30;
	float afStage[2] = {0.0f};
	//------------------------
	for(int i=0; i<m_iNumSections; i++)
	{	aLoadExt.DoIt(i);
		aLoadExt.GetStage(afStage, 2);
		float fDistX = afStage[0] - fStageX;
		float fDistY = afStage[1] - fStageY;
		double dDist = sqrt(fDistX * fDistX + fDistY * fDistY);
		if(i == 0 || dMin < dDist)
		{	dMin = dDist;
			iSection = i;
		}
	}
	//---------------------------
	m_fDist = (float)dMin;
	close(iFile);
	return iSection;
}

int CFindMrc::FindFromShift(float fShiftX, float fShiftY)
{
	int iFile = mOpenFile();
	if(iFile == -1) return -1;
	//------------------------
	CLoadExtHeader aLoadExt;
	aLoadExt.SetFile(iFile);
	//----------------------
	double dMin = 1e30;
	int iSection;
	float afShift[2] = {0.0f};
	//------------------------
	for(int i=0; i<m_iNumSections; i++)
	{	aLoadExt.DoIt(i);
		aLoadExt.GetShift(afShift, 2);
		float fDistX = afShift[0] - fShiftX;
		float fDistY = afShift[1] - fShiftY;
		double dDist = sqrt(fDistX * fDistX + fDistY * fDistY);
		if(i == 0 || dMin < dDist)
		{	dMin = dDist;
			iSection = i;
		}
	}
	//---------------------------
	m_fDist = (float)dMin;
	close(iFile);
	return iSection;
}

int CFindMrc::mOpenFile(char* pcFileName)
{
	if(strlen(m_pcFileName) == 0) return -1;
	else return open(m_pcFileName, O_RDWR); 
}
