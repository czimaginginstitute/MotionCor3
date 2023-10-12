#include "CFindCtfInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::FindCtf;

CFindCtf1D::CFindCtf1D(void)
{
	m_pFindDefocus1D = 0L;
	m_gfRadialAvg = 0L;
}

CFindCtf1D::~CFindCtf1D(void)
{
	this->Clean();
}

void CFindCtf1D::Clean(void)
{
	if(m_pFindDefocus1D != 0L) 
	{	delete m_pFindDefocus1D;
		m_pFindDefocus1D = 0L;
	}
	if(m_gfRadialAvg != 0L)
	{	cudaFree(m_gfRadialAvg);
		m_gfRadialAvg = 0L;
	}
}

void CFindCtf1D::Setup1(int* piSpectSize)
{
	this->Clean();
	CFindCtfBase::Setup1(piSpectSize);
	//--------------------------------
	cudaMalloc(&m_gfRadialAvg, sizeof(float) * m_aiSpectSize[0]);
	//---------------------------------------------------------
	m_pFindDefocus1D = new CFindDefocus1D;
	m_pFindDefocus1D->Setup1(m_aiSpectSize[0]);
}

void CFindCtf1D::Setup2(CCtfParam* pCtfParam)
{
	CFindCtfBase::Setup2(pCtfParam);
	//------------------------------
	m_pFindDefocus1D->Setup2(m_pCtfParam0);
	m_pFindDefocus1D->SetResRange(m_afResRange);
}

void CFindCtf1D::DoIt(float* gfSpect)
{	
	CFindCtfBase::DoIt(gfSpect);
	mCalcRadialAverage();
	//-------------------
	bool bAngstrom = true, bDegree = true;
	float fInitDf = m_pCtfParam0->GetDfMin(bAngstrom);
	float fInitPhase = m_pCtfParam0->GetExtPhase(bDegree);
	//----------------------------------------------------
	float afDfRange[2] = {0.0f};
	mGetDfRange(fInitDf, m_fDfRange, afDfRange);
	float afPhaseRange[2] = {0.0f};
	mGetPhaseRange(fInitPhase, m_fPhaseRange, afPhaseRange);
	mFindDefocus(afDfRange, afPhaseRange);
	//------------------------------------
	fInitDf = m_pCtfParamN->GetDfMin(bAngstrom);
	fInitPhase = m_pCtfParamN->GetExtPhase(bDegree);
	mGetDfRange(fInitDf, 0.3f * m_fDfRange, afDfRange);
	mGetPhaseRange(fInitPhase, 0.3f * m_fPhaseRange, afPhaseRange);
	mFindDefocus(afDfRange, afPhaseRange);
}

void CFindCtf1D::mFindDefocus(float* pfDfRange, float* pfPhaseRange)
{
	m_pFindDefocus1D->DoIt(pfDfRange, pfPhaseRange, m_gfRadialAvg);
	//-------------------------------------------------------------
	bool bDegree = true, bAngstrom = true;
	m_pCtfParamN->SetExtPhase(m_pFindDefocus1D->m_fBestPhase, bDegree);
	m_pCtfParamN->SetDfs(m_pFindDefocus1D->m_fBestDf, 
	   m_pFindDefocus1D->m_fBestDf, bAngstrom);
	m_pCtfParamN->m_fScore = m_pFindDefocus1D->m_fMaxCC;
}

void CFindCtf1D::mCalcRadialAverage(void)
{
	GRadialAvg aGRadialAvg;
	aGRadialAvg.DoIt(m_gfSpect, m_gfRadialAvg, m_aiSpectSize);
}

