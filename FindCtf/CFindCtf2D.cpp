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

CFindCtf2D::CFindCtf2D(void)
{
	m_pFindDefocus2D = 0L;
}

CFindCtf2D::~CFindCtf2D(void)
{
	this->Clean();
}

void CFindCtf2D::Clean(void)
{
	if(m_pFindDefocus2D != 0L) 
	{	delete m_pFindDefocus2D;
		m_pFindDefocus2D = 0L;
	}
	CFindCtf1D::Clean();
}

void CFindCtf2D::Setup1(int* piSpectSize)
{
	this->Clean();
	CFindCtf1D::Setup1(piSpectSize);
	m_pFindDefocus2D = new CFindDefocus2D;
	m_pFindDefocus2D->Setup1(piSpectSize);
}

void CFindCtf2D::Setup2(CCtfParam* pCtfParam)
{
	CFindCtf1D::Setup2(pCtfParam);
	//----------------------------
	m_pFindDefocus2D->Setup2(m_pCtfParam0);
	m_pFindDefocus2D->Setup3(m_afResRange);
}

void CFindCtf2D::DoIt(float* gfSpect)
{	
	CFindCtf1D::DoIt(gfSpect);
	//------------------------
	bool bAngstrom = true, bDegree = true;
	float fDfMean = m_pCtfParamN->GetDfMin(bAngstrom);
	float fPhase = m_pCtfParamN->GetExtPhase(bDegree);
	//------------------------------------------------
	m_pFindDefocus2D->Setup3(m_afResRange);
	m_pFindDefocus2D->Setup4(fDfMean, 0.0f, 0.0f, fPhase);
	m_pFindDefocus2D->DoIt(m_gfSpect, m_fPhaseRange);
	mGetResults();
}

void CFindCtf2D::Refine
(	float afDfMean[2],
	float afAstRatio[2],
	float afAstAngle[2],
	float afExtPhase[2]
)
{	m_pFindDefocus2D->Setup4(afDfMean[0], afAstRatio[0],
	   afAstAngle[0], afExtPhase[0]);
	m_pFindDefocus2D->Refine(m_gfSpect, afDfMean[1],
	   afAstRatio[1], afAstAngle[1], afExtPhase[1]);
	mGetResults();
}

void CFindCtf2D::mGetResults(void)
{
	bool bAngstrom = true, bDegree = true;
	m_pCtfParamN->SetDfMin(m_pFindDefocus2D->GetDfMin(), bAngstrom);
	m_pCtfParamN->SetDfMax(m_pFindDefocus2D->GetDfMax(), bAngstrom);
	m_pCtfParamN->SetAstAngle(m_pFindDefocus2D->GetAngle(), bDegree);
	m_pCtfParamN->SetExtPhase(m_pFindDefocus2D->GetExtPhase(), bDegree);
	m_pCtfParamN->m_fScore = m_pFindDefocus2D->GetScore();	
}
