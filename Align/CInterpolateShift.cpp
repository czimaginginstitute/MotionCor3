#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <Util/Util_LinEqs.h>
#include <memory.h>
#include <stdio.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CInterpolateShift::CInterpolateShift(void)
{
}

CInterpolateShift::~CInterpolateShift(void)
{
}

CStackShift* CInterpolateShift::DoIt
(	CStackShift* pGroupShift, 
	DU::CFmGroupParam* pFmGroupParam,
	DU::CFmIntegrateParam* pFmIntParam,
	bool bNearest
)
{	CStackShift* pStackShift = new CStackShift;
	pStackShift->Setup(pFmGroupParam->m_iNumIntFms);
	this->DoIt(pGroupShift, pStackShift, pFmGroupParam,
	   pFmIntParam, bNearest);
	return pStackShift;
}

void CInterpolateShift::DoIt
(	CStackShift* pGroupShift,
	CStackShift* pOutShift,
	DU::CFmGroupParam* pFmGroupParam,
	DU::CFmIntegrateParam* pFmIntParam,
	bool bNearest
)
{	m_iNumFrames = pOutShift->m_iNumFrames;
	m_iNumGroups = pGroupShift->m_iNumFrames;
	//--------------------------------------------------------
	// This is the case that the group size is 1 and thus no
	// interpolation is needed. Just copy the data over.
	//--------------------------------------------------------
	if(m_iNumFrames == m_iNumGroups)
	{	float afShift[2] = {0.0f};
		for(int i=0; i<m_iNumGroups; i++)
		{	pGroupShift->GetShift(i, afShift);
			pOutShift->SetShift(i, afShift);
		}
		return;	
	}
	//----------------------------------------------
	// Use the near point instead od interpolation.
	//----------------------------------------------
	if(bNearest)
	{	float afShift[2] = {0.0f};
		for(int g=0; g<m_iNumGroups; g++)
		{	int iGpStart = pFmGroupParam->GetGroupStart(g);
			int iGpSize = pFmGroupParam->GetGroupSize(g);
			pGroupShift->GetShift(g, afShift);
			for(int i=0; i<iGpSize; i++)
			{	pOutShift->SetShift(i+iGpStart, afShift);
			}
		}
		return;
	}
	//--------------------------------------------------
	// Linear interpolation based on the group centers.
	//--------------------------------------------------
	float* pfGroupCents = new float[m_iNumGroups];
	for(int i=0; i<m_iNumGroups; i++)
	{	pfGroupCents[i] = pFmGroupParam->GetGroupCenter(i);
	}
	//---------------------------------------------------------
	float* pfGroupShifts = new float[m_iNumGroups * 2];
	float* pfGroupShiftYs = pfGroupShifts + m_iNumGroups;
	for(int i=0; i<m_iNumGroups; i++)
	{	float afShift[2] = {0.0f};
		pGroupShift->GetShift(i, afShift);
		pfGroupShifts[i] = afShift[0];
		pfGroupShiftYs[i] = afShift[1];
	}
	//-------------------------------------
	float* pfFmCents = new float[m_iNumFrames];
	//-------------------------------------------------------
	float* pfFmShifts = new float[m_iNumFrames * 2];
	mInterpolate(pfGroupCents, pFmIntParam->m_pfIntFmCents, 
	   pfGroupShifts, pfFmShifts); 
	//----------------------------
	float* pfFmShiftYs = pfFmShifts + m_iNumFrames;
	for(int i=0; i<m_iNumFrames; i++)
	{	float afShift[2] = {0.0f};
		afShift[0] = pfFmShifts[i];
		afShift[1] = pfFmShiftYs[i];
		pOutShift->SetShift(i, afShift);
	}
	//--------------------------------------
	memcpy(pOutShift->m_afCenter, pGroupShift->m_afCenter, 
	   sizeof(float) * 3);
	pOutShift->m_bConverged = pGroupShift->m_bConverged;
	//--------------------------------------------------
	if(pfGroupCents != 0L) delete[] pfGroupCents;
	if(pfGroupShifts != 0L) delete[] pfGroupShifts;
	if(pfFmCents != 0L) delete[] pfFmCents;
	if(pfFmShifts != 0L) delete[] pfFmShifts;
}	

void CInterpolateShift::mInterpolate
(	float* pfGpCents,
	float* pfFmCents,
	float* pfGpShifts,
	float* pfFmShifts
)
{	float* pfGpShiftYs = pfGpShifts + m_iNumGroups;
	float* pfFmShiftYs = pfFmShifts + m_iNumFrames;
	float fSlopeX, fSlopeY, fCentG, fDifG, fDifF;
	//-------------------------------------------
	for(int i=0; i<m_iNumFrames; i++)
	{	int g = mFindGroup(pfGpCents, pfFmCents[i]);
		int f = g - 1;
		fDifG = pfGpCents[g] - pfGpCents[f];
		fDifF = pfFmCents[i] - pfGpCents[f];
		fSlopeX = (pfGpShifts[g] - pfGpShifts[f]) / fDifG;
		fSlopeY = (pfGpShiftYs[g] - pfGpShiftYs[f]) / fDifG;
		pfFmShifts[i] = pfGpShifts[f] + fSlopeX * fDifF;
		pfFmShiftYs[i] = pfGpShiftYs[f] + fSlopeY * fDifF;
	}
}

int CInterpolateShift::mFindGroup(float* pfGpCents, float fCent)
{
	if(fCent <= pfGpCents[0]) return 1;
	for(int g=1; g<m_iNumGroups; g++)
	{	if(fCent < pfGpCents[g]) return g;
	}
	return m_iNumGroups - 1;
}

