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

static float s_fD2R = 0.01745329f;

CFindDefocus2D::CFindDefocus2D(void)
{
	m_gfCtf2D = 0L;
	m_pGCtfCC2D = 0L;
	m_fAstRatio = 0.0f; // (m_fDfMean - fMinDf) / m_fDfMean;
	m_fAstAngle = 0.0f; // degree
}

CFindDefocus2D::~CFindDefocus2D(void)
{
	this->Clean();
}

void CFindDefocus2D::Clean(void)
{
	if(m_gfCtf2D != 0L) 
	{	cudaFree(m_gfCtf2D);
		m_gfCtf2D = 0L;
	}
	if(m_pGCtfCC2D != 0L)
	{	delete m_pGCtfCC2D;
		m_pGCtfCC2D = 0L;
	}
}

float CFindDefocus2D::GetDfMin(void)
{
	float fDfMin = m_fDfMean * (1.0f - m_fAstRatio);
	return fDfMin;
}

float CFindDefocus2D::GetDfMax(void)
{
	float fDfMax = m_fDfMean * (1.0f + m_fAstRatio);
	return fDfMax;
}

float CFindDefocus2D::GetAngle(void)
{
	return m_fAstAngle;
}

float CFindDefocus2D::GetExtPhase(void)
{
	return m_fExtPhase;
}

float CFindDefocus2D::GetScore(void)
{
	return m_fCCMax;
}

void CFindDefocus2D::Setup1(int* piSpectSize)
{
	this->Clean();
	//------------
	m_aiSpectSize[0] = piSpectSize[0];
	m_aiSpectSize[1] = piSpectSize[1];
	//--------------------------------
	cudaMalloc(&m_gfCtf2D, sizeof(float) 
	   * m_aiSpectSize[0] * m_aiSpectSize[1]);
	//----------------------------------------
	m_pGCtfCC2D = new GCtfCC2D;
	m_pGCtfCC2D->SetSize(m_aiSpectSize);	
}

void CFindDefocus2D::Setup2(CCtfParam* pCtfParam)
{
	m_pCtfParam = pCtfParam;
	m_aGCalcCtf2D.SetParam(m_pCtfParam);
}

void CFindDefocus2D::Setup3(float afResRange[2])
{
	float fRes1 = m_aiSpectSize[1] * m_pCtfParam->m_fPixSize;
	float fMinFreq = fRes1 / afResRange[0];
	float fMaxFreq = fRes1 / afResRange[1];
	m_pGCtfCC2D->Setup(fMinFreq, fMaxFreq, 81.0f);
}

void CFindDefocus2D::Setup4
(	float fDfMean,
	float fAstRatio,
	float fAstAngle,
	float fExtPhase
)
{	m_fDfMean = fDfMean;
	m_fAstRatio = fAstRatio;
	m_fAstAngle = fAstAngle;
	m_fExtPhase = fExtPhase;
}

void CFindDefocus2D::DoIt(float* gfSpect, float fPhaseRange)
{	
	m_gfSpect = gfSpect;
	m_fCCMax = mCorrelate(m_fAstRatio, m_fAstAngle, m_fExtPhase);
	//-----------------------------------------------------------
	float afAstRanges[] = {0.1f, 180.0f};
	float fDfMeanRange = 0.2f * m_fDfMean;
	mIterate(afAstRanges, fDfMeanRange, fPhaseRange, 2);
	//--------------------------------------------------
	afAstRanges[0] = 0.05f;
	afAstRanges[1] = 18.0f;
	fPhaseRange = 0.5f * fPhaseRange;
	mIterate(afAstRanges, fDfMeanRange, fPhaseRange, 20);
}

void CFindDefocus2D::Refine
(	float* gfSpect,
	float fDfMeanRange,
	float fAstRange,
	float fAngRange,
	float fPhaseRange
)
{	m_gfSpect = gfSpect;
	m_fCCMax = mCorrelate(m_fAstRatio, m_fAstAngle, m_fExtPhase);
	//-----------------------------------------------------------
	float afAstRanges[] = {fAstRange, fAngRange};
	mIterate(afAstRanges, fDfMeanRange, fPhaseRange, 30);
}

float CFindDefocus2D::mIterate
(	float afAstRanges[2], 
	float fDfMeanRange, 
	float fPhaseRange,
	int iIterations
)
{	float fAstErr, fDfMeanErr, fPhaseErr, fSumErr;
	for(int i=0; i<iIterations; i++)
	{	float fCCMaxOld = m_fCCMax;
		float fScale = fmaxf(1.0f - i * 0.03f, 0.1f);
		if(i < 5)
		{	fAstErr = mGridSearch(afAstRanges[0] * fScale, 
		   	   afAstRanges[1] * fScale);
		}
		else
		{	fAstErr = mGridSearch(afAstRanges[0] * fScale, 0.0f);
			fAstErr = mGridSearch(0.0f, afAstRanges[1] * fScale);
		}
		fDfMeanErr = mRefineDfMean(fDfMeanRange * fScale);
		fPhaseErr = mRefinePhase(fPhaseRange * fScale);
		fSumErr = fAstErr + fDfMeanErr + fPhaseErr;
		//printf("Iter: %3d  %9.5e  %9.5e\n", i+1, m_fCCMax, fSumErr);
	}
	return fSumErr;
}

float CFindDefocus2D::mGridSearch(float fRatRange, float fAngRange)
{	
	float fTiny = (float)1e-20;
	if(fRatRange < fTiny  && fAngRange < fTiny) return 0.0f;
	//------------------------------------------------------
	int iRatSteps = (int)(fRatRange / 0.001);
	if(iRatSteps > 51) iRatSteps = 51;
	else if(iRatSteps < 1) iRatSteps = 1;
	float fRat0 = fmaxf(m_fAstRatio - fRatRange * 0.5f, 0.001f);
	float fRatStep = fRatRange / iRatSteps;
	//-------------------------------------
	int iAngSteps = (int)(fAngRange / 0.1f);
	if(iAngSteps > 51) iAngSteps = 51;
	else if(iAngSteps < 1) iAngSteps = 1;
	float fAng0 = fmaxf(m_fAstAngle - fAngRange * 0.5f, -90.0f);
	float fAngN = fAng0 + fAngRange;
	if(fAngN > 90.0f) fAng0 = 90.0f - fAngRange;
	float fAngStep = fAngRange / iAngSteps;
	//-------------------------------------
	float fAngMax, fRatMax, fCCMax = (float)-1e20;
	//--------------------------------------------
	for(int j=0; j<iAngSteps; j++)
	{	float fAng = fAng0 + j * fAngStep;
		for(int i=0; i<iRatSteps; i++)
		{	float fRat = fRat0 + i * fRatStep;
			float fCC = mCorrelate(fRat, fAng, m_fExtPhase);
			if(fCC <= fCCMax) continue;
			//-------------------------
			fCCMax = fCC;
			fAngMax = fAng;
			fRatMax = fRat;
		}
	}
	//--------------------------------
	if(fCCMax <= m_fCCMax) return 0.0f;
	float fErr = fabsf((m_fAstRatio - fRatMax) / (m_fAstRatio + fTiny))
	   + fabs((m_fAstAngle - fAngMax) / (m_fAstAngle + fTiny));
	//---------------------------------------------------------
	m_fAstRatio = fRatMax;
	m_fAstAngle = fAngMax;
	m_fCCMax = fCCMax;
	return fErr;
}

float CFindDefocus2D::mRefineDfMean(float fDfRange)
{
	float fTiny = (float)1e-30;
	if(fDfRange < fTiny) return 0.0f;
	//--------------------------------
	float fDfMeanOld = m_fDfMean;
	int iNumSteps = 101;
	float fDfMean0 = fmaxf(m_fDfMean - fDfRange * 0.5f, 1000.0f);
	float fDfMeanN = m_fDfMean + fDfRange * 0.5f;
	float fStep = fDfRange / (iNumSteps - 1);
	//-----------------
	float fDfMeanMax = 0.0f, fCCMax = (float)-1e20;
	for(int i=0; i<iNumSteps; i++)
	{	m_fDfMean = fDfMean0 + i * fStep;
		float fCC = mCorrelate(m_fAstRatio, m_fAstAngle, m_fExtPhase);
		if(fCC <= fCCMax) continue;
		//-------------------------
		fDfMeanMax = m_fDfMean;
		fCCMax = fCC;
	}
	//-------------------
	if(fCCMax <= m_fCCMax)
	{	m_fDfMean = fDfMeanOld;
		return 0.0f;
	}
	//-----------------------------
	float fErr = fabsf((m_fDfMean - fDfMeanMax) / (m_fDfMean + fTiny));
	m_fDfMean = fDfMeanMax;
	m_fCCMax = fCCMax;
	return fErr;
}

float CFindDefocus2D::mRefinePhase(float fPhaseRange)
{
	float fTiny = (float)1e-30;
	if(fPhaseRange < fTiny) return 0.0f;
	//----------------------------------
	int iNumSteps = 61;
	float fPhase0 = fmaxf(m_fExtPhase - 0.5f * fPhaseRange, 0.0f);
	float fPhaseN = fminf(m_fExtPhase + 0.5f * fPhaseRange, 180.0f);
	float fStep = (fPhaseN - fPhase0) / (iNumSteps - 1);
	//-----------------
	float fCCMax = (float)-1e20, fPhaseMax = 0.0f;
	for(int i=0; i<iNumSteps; i++)
	{	float fExtPhase = fPhase0 + i * fStep;
		float fCC = mCorrelate(m_fAstRatio, m_fAstAngle, fExtPhase);
		if(fCC <= fCCMax) continue;
		//----------------
		fPhaseMax = fExtPhase; 
		fCCMax = fCC;
	}
	if(fCCMax <= m_fCCMax) return 0.0f;
	//---------------------------------
	float fErr = fabsf((m_fExtPhase - fPhaseMax) / (m_fExtPhase + fTiny));
	m_fExtPhase = fPhaseMax;
	m_fCCMax = fCCMax;
	return fErr;
}

float CFindDefocus2D::mCorrelate
(	float fAstRatio, 
	float fAstAngle, 
	float fExtPhase
)
{	float fExtPhaseRad = fExtPhase * s_fD2R;
	float fAstRad = fAstAngle * s_fD2R;
	//---------------------------------
	float fDfMin = CFindCtfHelp::CalcDfMin(m_fDfMean, fAstRatio)
	   / m_pCtfParam->m_fPixSize;
	float fDfMax = CFindCtfHelp::CalcDfMax(m_fDfMean, fAstRatio)
	   / m_pCtfParam->m_fPixSize;
	//-----------------------------
	m_aGCalcCtf2D.DoIt(fDfMin, fDfMax, fAstRad, fExtPhaseRad, 
	   m_gfCtf2D, m_aiSpectSize);
	float fCC = m_pGCtfCC2D->DoIt(m_gfCtf2D, m_gfSpect);
	return fCC;
}
