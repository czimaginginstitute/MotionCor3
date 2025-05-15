#include "CFindCtfInc.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::FindCtf;

CFindCtfBase::CFindCtfBase(void)
{
	m_fPhaseRange = 0.0f;
	m_pCtfParam0 = new CCtfParam; // initial values
	m_pCtfParamN = new CCtfParam; // final values
}

CFindCtfBase::~CFindCtfBase(void)
{
	if(m_pCtfParam0 != 0L) delete m_pCtfParam0;
	if(m_pCtfParamN != 0L) delete m_pCtfParamN;
	m_pCtfParam0 = 0L;
	m_pCtfParamN = 0L;
}

void CFindCtfBase::Setup1(int* piSpectSize)
{
	m_aiSpectSize[0] = piSpectSize[0];
	m_aiSpectSize[1] = piSpectSize[1];
}

void CFindCtfBase::Setup2(CCtfParam* pCtfParam)
{
	m_pCtfParam0->Setup(pCtfParam);
	m_pCtfParamN->Setup(pCtfParam);
	//-----------------------------------------------
	// 1) m_afResRange decides the range of Fourier
	// components are involved in CTF estimation.
	// 2) see CFindCtf1D for using m_afResRange.
	//-----------------------------------------------
	float fPixSize = m_pCtfParam0->GetPixSize();
	m_afResRange[0] = 20.0f * fPixSize;
	m_afResRange[1] = (2.0f * fPixSize) / 0.8f;
	if(m_afResRange[1] < 3.5f) m_afResRange[1] = 3.5f;
}

void CFindCtfBase::SetDfRange(float fDfRange)
{	
	m_fDfRange = fDfRange;
}

void CFindCtfBase::SetAstRange(float fAstRange)
{
	m_fAstRange = fAstRange;
}

void CFindCtfBase::SetAngRange(float fAngRange)
{
	m_fAngRange = fAngRange;
}

void CFindCtfBase::SetPhaseRange(float fPhaseRange)
{
	m_fPhaseRange = fPhaseRange;
}

void CFindCtfBase::DoIt(float* gfSpect)
{
	m_gfSpect = gfSpect;
}

CCtfParam* CFindCtfBase::GetResult(void)
{
	CCtfParam* pRetParam = new CCtfParam;
	pRetParam->Setup(m_pCtfParamN);
	return pRetParam;
}

void CFindCtfBase::GetResRange(float* pfResRange)
{
	pfResRange[0] = m_afResRange[0];
	pfResRange[1] = m_afResRange[1];
}

void CFindCtfBase::mGetDfRange(float fInitDf, float fDfRange, float* pfDfRange)
{
	float fPixSize = m_pCtfParam0->GetPixSize();
	float fPixSize2 = fPixSize * fPixSize;
	float fDfMin = 1000.0f * fPixSize2;
	float fDfMax = 30000.0f * fPixSize2;
	if(fDfMin < 1000.0f) fDfMin = 1000.0f;
	//----------------------------------
	pfDfRange[0] = fmaxf(fInitDf - 0.5f * fDfRange, fDfMin);
	pfDfRange[1] = fminf(pfDfRange[0] + fDfRange, fDfMax);
}

void CFindCtfBase::mGetPhaseRange
(	float fInitPhase,
	float fPhaseRange,
	float* pfPhaseRange
)
{	pfPhaseRange[0] = fmaxf(fInitPhase - 0.5f * fPhaseRange, 0.0f);
	pfPhaseRange[1] = fmin(pfPhaseRange[0] + fPhaseRange, 180.0f);
}

