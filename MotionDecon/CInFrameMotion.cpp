#include "CMotionDeconInc.h"
#include <memory.h>
#include <stdio.h>
#include <math.h>

using namespace MotionCor2;
using namespace MotionCor2::MotionDecon;

CInFrameMotion::CInFrameMotion(void)
{
	m_iNumFrames = 0;
	m_iNumPatches = 0;
	m_pFullShift = 0L;
	m_pPatchShifts = 0L;
}

CInFrameMotion::~CInFrameMotion(void)
{
}

void CInFrameMotion::SetFullShift
(	Align::CStackShift* pStackShift
)
{	m_pFullShift = pStackShift;
	m_iNumFrames = m_pFullShift->m_iNumFrames;
}

void CInFrameMotion::SetPatchShifts
(	Align::CPatchShifts* pPatchShifts
)
{	m_pPatchShifts = pPatchShifts;
	m_iNumFrames = m_pPatchShifts->m_aiFullSize[2];
	m_iNumPatches = m_pPatchShifts->m_iNumPatches;
}

void CInFrameMotion::DoFullMotion
(	int iFrame,
	cufftComplex* gCmpFrm,
	int* piCmpSize,
        cudaStream_t stream
)
{	float afMotion[2] = {0.0f};
	this->GetFullMotion(iFrame, afMotion);
	GMotionWeight aGMotionWeight;
	aGMotionWeight.DirWeight(afMotion, gCmpFrm, piCmpSize, stream);
}

void CInFrameMotion::DoLocalMotion
(	int iFrame,
	cufftComplex* gCmpFrm,
	int* piCmpSize,
        cudaStream_t stream
)
{	if(m_iNumPatches <= 0) return;
	//----------------------------
	float afMotion[2];
	double dMeanX = 0, dMeanY = 0;
	double dVarX = 0, dVarY = 0;
	for(int i=0; i<m_iNumPatches; i++)
	{	this->GetLocalMotion(iFrame, i, afMotion);
		dMeanX += afMotion[0];
		dMeanY == afMotion[1];
		dVarX += afMotion[0] * afMotion[0];
		dVarY += afMotion[1] * afMotion[1];
	}
	dMeanX /= m_iNumPatches;
	dMeanY /= m_iNumPatches;
	dVarX = dVarX / m_iNumPatches - dMeanX * dMeanX;
	dVarY = dVarY / m_iNumPatches - dMeanY * dMeanY;
	double dMotion = dVarX + dVarY;
	if(dMotion <= 0) return;
	//----------------------
	float fMotion = (float)sqrtf(dMotion);
	GMotionWeight aGMotionWeight;
	aGMotionWeight.Weight(fMotion, gCmpFrm, piCmpSize, stream);
}	

void CInFrameMotion::GetFullMotion
(	int iFrame, 
	float* pfMotion
)
{	pfMotion[0] = 0.0f;
	pfMotion[1] = 0.0f;
	if(iFrame < 0 || iFrame >= m_iNumFrames) return;
	//----------------------------------------------
	float afShift1[2], afShift2[2];
	int iFrame1 = iFrame;
	int iFrame2 = iFrame + 1;
	if(iFrame == (m_iNumFrames - 1))
	{	iFrame1 = iFrame - 1;
		iFrame2 = iFrame;
	}
	//-----------------------
	m_pFullShift->GetShift(iFrame1, afShift1);
	m_pFullShift->GetShift(iFrame2, afShift2);
	pfMotion[0] = afShift2[0] - afShift1[0];
	pfMotion[1] = afShift2[1] - afShift1[1];	
} 

void CInFrameMotion::GetLocalMotion
(	int iFrame,
	int iPatch,
	float* pfMotion
)
{	pfMotion[0] = 0.0f;
	pfMotion[1] = 0.0f;
	if(iFrame < 0 || iFrame >= m_iNumFrames) return;
	if(iPatch < 0 || iPatch >= m_iNumPatches) return;
	//---------------------------------------------------
	float afShift1[2], afShift2[2];
	int iFrame1 = iFrame;
	int iFrame2 = iFrame + 1;
	if(iFrame == (m_iNumFrames - 1))
	{	iFrame1 = iFrame - 1;
		iFrame2 = iFrame;
	}
	//-----------------------
	bool bRawShift = true;
	m_pPatchShifts->GetLocalShift(iFrame1, iPatch, afShift1);
	m_pPatchShifts->GetLocalShift(iFrame2, iPatch, afShift2);
	pfMotion[0] = afShift2[0] - afShift1[0];
	pfMotion[1] = afShift2[1] - afShift1[1];	
}
