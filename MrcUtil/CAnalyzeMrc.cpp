#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <memory.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

using namespace MotionCor2::MrcUtil;

CAnalyzeMrc::CAnalyzeMrc(void)
{
}

CAnalyzeMrc::~CAnalyzeMrc(void)
{
}

bool CAnalyzeMrc::IsMrc(char* pcMrcFile)
{
	Mrc::CLoadMrc aLoadMrc;
        bool bOpen = aLoadMrc.OpenFile(pcMrcFile);
	if(!bOpen) return false;
	else return true;
}

bool CAnalyzeMrc::IsTomoMrc(char* pcMrcFile)
{
	Mrc::CLoadMrc aLoadMrc;
	if(!aLoadMrc.OpenFile(pcMrcFile)) return false;
	int iNumFloats = aLoadMrc.m_pLoadMain->GetNumFloats();
	if(iNumFloats == 0) return false;
	//-------------------------------
	int iNumImgs = aLoadMrc.m_pLoadMain->GetSizeZ();
	float afTilt[] = {0.0f, 0.0f};
	aLoadMrc.m_pLoadExt->DoIt(0);
	aLoadMrc.m_pLoadExt->GetTilt(afTilt+0, 1);
	aLoadMrc.m_pLoadExt->DoIt(iNumImgs-1);
	aLoadMrc.m_pLoadExt->GetTilt(afTilt+1, 1);
	//----------------------------------------
	float fRange = fabs(afTilt[0] - afTilt[1]);
	if(fabs(afTilt[0]) > 80) return false;
	else if(fabs(afTilt[1]) > 80) return false;
	else if(fRange <= 2) return false;
	else return true;
}

bool CAnalyzeMrc::IsSerialTomoMrc(char* pcMrcFile)
{
	Mrc::CLoadMrc aLoadMrc;
        bool bOpen = aLoadMrc.OpenFile(pcMrcFile);
        if(!bOpen) return false;
	//----------------------
	int iNumFloats = aLoadMrc.m_pLoadMain->GetNumFloats();
	if(iNumFloats <= 0) return false;
	//-------------------------------
	int iSizeZ = aLoadMrc.m_pLoadMain->GetSizeZ();
	char acSerialFile[256];
	for(int i=0; i<iSizeZ; i++)
	{	memset(acSerialFile, 0, sizeof(acSerialFile));
		sprintf(acSerialFile, "%s%d", pcMrcFile, i);
		bOpen = aLoadMrc.OpenFile(acSerialFile);
		if(bOpen) return true;
	}
	return false;
}

void CAnalyzeMrc::GetMrcSize
(	char* pcMrcFile, 
	int* piMrcSize, 
	int* piNumStacks
)
{	memset(piMrcSize, 0, sizeof(int) * 3);
	*piNumStacks = 0;
	//---------------
	Mrc::CLoadMrc aLoadMrc;
	if(!aLoadMrc.OpenFile(pcMrcFile)) return;
	if(!IsTomoMrc(pcMrcFile)) return;
	//-------------------------------
	piMrcSize[0] = aLoadMrc.m_pLoadMain->GetSizeX();
	piMrcSize[1] = aLoadMrc.m_pLoadMain->GetSizeY();
	piMrcSize[2] = aLoadMrc.m_pLoadMain->GetSizeZ();
	//-------------------------------------------
	int iStackZ = aLoadMrc.m_pLoadMain->GetStackZ();
	if(iStackZ > 0) *piNumStacks = piMrcSize[2] / iStackZ;
}
