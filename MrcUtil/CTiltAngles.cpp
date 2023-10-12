#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <memory.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

CTiltAngles::CTiltAngles(void)
{
	m_fStartTilt = 0.0f;
	m_fTiltStep = 2.0f;
	m_iNumTilts = 0;
}

CTiltAngles::~CTiltAngles(void)
{
}

void CTiltAngles::Setup
(	float fFirstTilt, 
	float fTiltStep
)
{	m_fStartTilt = fFirstTilt;
	m_fTiltStep = fTiltStep;
}

bool CTiltAngles::Read(char* pcMrcFile)
{
	m_fMaxTilt = -180.0f;
	m_fMinTilt = 180.0f;
	m_iNumTilts = 0;
	if(fabs(m_fTiltStep) <= 0.1) return false;
	//----------------------------------------
	Mrc::CLoadMrc aLoadMrc;
	if(!aLoadMrc.OpenFile(pcMrcFile)) return false;
	//---------------------------------------------
	int iSizeZ = aLoadMrc.m_pLoadMain->GetSizeZ();
	int iStackZ = aLoadMrc.m_pLoadMain->GetStackZ();
	if(iStackZ < 1) iStackZ = 1;
	else if(iStackZ >= iSizeZ) iStackZ = 1;
	//-------------------------------------
	float fTilt = 0.0f;
	int iNumStarts = 0;
	//-----------------
	for(int i=0; i<iSizeZ; i+=iStackZ)
	{	aLoadMrc.m_pLoadExt->DoIt(i);
		aLoadMrc.m_pLoadExt->GetTilt(&fTilt, 1);
		//--------------------------------------
		if(fTilt > m_fMaxTilt) m_fMaxTilt = fTilt;
		else if(fTilt < m_fMinTilt) m_fMinTilt = fTilt;
		//---------------------------------------------
		if(fabs(fTilt - m_fStartTilt) >= 0.2) continue;
		else iNumStarts += 1;
	}
	//---------------------------
	float fNumTilts = (m_fMaxTilt - m_fMinTilt) / m_fTiltStep;
        m_iNumTilts = (int)(fabs(fNumTilts) + 0.5f) + 1;
	if(iNumStarts == 2) m_iNumTilts += 1;
	//-----------------------------------
	float fEndTilt1 = (m_fTiltStep > 0) ? m_fMaxTilt : m_fMinTilt;
	float fHalf1 = (fEndTilt1 - m_fStartTilt) / m_fTiltStep;
	m_iHalf1 = (int)(fHalf1 + 0.5f) + 1;
	return true;
}

int CTiltAngles::GetAcqIndex(float fTilt, float* pfShift)
{
	if(m_iNumTilts <= 0) return 0;
	//----------------------------
	double dShift = 0;
	if(pfShift != 0L) dShift = fabs(pfShift[0]) + fabs(pfShift[1]);
	//-------------------------------------------------------------
	int iSign = (m_fTiltStep > 0) ? 1 : -1;
	float fIndex = (fTilt - m_fStartTilt) / m_fTiltStep;
	int iIndex = (int)(fIndex + iSign * 0.5f);
	//----------------------------------------
	if(iIndex == 0 && dShift == 0) return 0;
	else if(iIndex > 0) return iIndex;
	else return m_iHalf1 - iIndex;
	
}

