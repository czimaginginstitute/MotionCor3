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

float CFindDefocus2D::GetCtfRes(void)
{
	return m_fCtfRes;
}

void CFindDefocus2D::Setup1(CCtfParam* pCtfParam, int* piCmpSize)
{
	this->Clean();
	//------------
	m_pCtfParam = pCtfParam;
	memcpy(m_aiCmpSize, piCmpSize, sizeof(int) * 2);
	//----------------------------------------------
	m_aGCalcCtf2D.SetParam(m_pCtfParam);
	//----------------------------------
	cudaMalloc(&m_gfCtf2D, sizeof(float) 
	   * m_aiCmpSize[0] * m_aiCmpSize[1]);
	//------------------------------------
	m_pGCtfCC2D = new GCtfCC2D;
	m_pGCtfCC2D->SetSize(m_aiCmpSize);	
}

void CFindDefocus2D::Setup2(float afResRange[2])
{
	float fRes1 = m_aiCmpSize[1] * m_pCtfParam->m_fPixSize;
	float fMinFreq = fRes1 / afResRange[0];
	float fMaxFreq = fRes1 / afResRange[1];
	m_pGCtfCC2D->Setup(fMinFreq, fMaxFreq, 40.0f);
}

//--------------------------------------------------------------------
// 1. DoIt() should be called after CFindDefocus1D::DoIt(), which
//    generates an estimate of m_fDfMean.
//--------------------------------------------------------------------
void CFindDefocus2D::Setup3
(	float fDfMean,
	float fAstRatio,
	float fAstAngle,
	float fExtPhase
)
{	m_fDfMean = fDfMean;
	m_fAstRatio = fAstRatio;
	m_fAstAngle = fAstAngle;
	m_afPhaseRange[0] = fExtPhase;
}

void CFindDefocus2D::DoIt(float* gfSpect, float fPhaseRange)
{
	m_gfSpect = gfSpect;
	m_afPhaseRange[1] = fPhaseRange;
        //-----------------
	m_afDfRange[0] = m_fDfMean * 0.9f;
	m_afDfRange[1] = m_fDfMean * 1.1f;
	m_afAstRange[0] = 0.0f;
	m_afAstRange[1] = 0.06f;
	m_afAngRange[0] = -90.0f;
	m_afAngRange[1] = 90.0f;
	//-----------------
	mIterate();
	mCalcCtfRes();	
}

void CFindDefocus2D::Refine
(	float* gfSpect,
	float fDfRange,
	float fAstRange,
	float fAngRange,
	float fPhaseRange
)
{	m_gfSpect = gfSpect;
	//-----------------
	float fHalfR = 0.5f * fDfRange;
	m_afDfRange[0] = fmaxf(m_fDfMean - fHalfR, 3000.0f);
	m_afDfRange[1] = m_fDfMean + fHalfR;
	//-----------------
	fHalfR = 0.5f * fAstRange;
	m_afAstRange[0] = fmaxf(m_fAstRatio - fHalfR, 0.0f);
	m_afAstRange[1] = fminf(m_fAstRatio + fHalfR, 0.02f);
	//-----------------
	fHalfR = 0.5f * fAngRange;
	m_afAngRange[0] = fmaxf(m_fAstAngle - fHalfR, -90.0f);
	m_afAngRange[1] = fminf(m_fAstAngle + fHalfR, 90.0f);
	//-----------------
	m_afPhaseRange[1] = fPhaseRange;
	//-----------------
	mIterate();
	mCalcCtfRes();
}

void CFindDefocus2D::mIterate(void)
{
	m_fCCMax = mCorrelate(m_fAstRatio, m_fAstAngle, m_fExtPhase);
	float fOldCC = m_fCCMax;
	//-----------------
	mFindAstig(m_afAstRange, m_afAngRange);
	if(fOldCC > m_fCCMax) return;
	//-----------------
	int iIterations = 20;
	m_fExtPhase = m_afPhaseRange[0];
	float fDfRange = m_afDfRange[1] - m_afDfRange[0];
        float fAstRange = m_afAstRange[1] - m_afAstRange[0];
        float fAngRange = m_afAngRange[1] - m_afAngRange[0];
	for(int i=1; i<iIterations; i++)
        {       float fScale = 1.0f - i * 0.5f / iIterations;
                //----------------
                float fRange = fScale * fAstRange;
		mRefineAstMag(fRange);
		if(fOldCC > m_fCCMax) return;
		//----------------
		m_fCCMax = fOldCC;
                fRange = fScale * fAngRange;
		mRefineAstAng(fRange);
                //----------------
		fRange = fScale * fDfRange;
		mRefineDfMean(fRange);
		//----------------
		fRange = fScale * m_afPhaseRange[1];
		mRefinePhase(fRange);
        }
}

float CFindDefocus2D::mFindAstig(float* pfAstRange, float* pfAngRange)
{
	float fTiny = (float)1e-20;
	float fAstRange = pfAstRange[1] - pfAstRange[0];
	float fAngRange = pfAngRange[1] - pfAngRange[0];
	if(fAstRange < fTiny  && fAngRange < fTiny) return 0.0f;
	//-----------------
	int iAstSteps = 51, iAngSteps = 51;
	float fAstStep = fAstRange / iAstSteps;
	float fAngStep = fAngRange / iAngSteps;
	//-----------------
	if(fAstStep < 1e-5) iAstSteps = 1;
	if(fAngStep < 1e-5) iAngSteps = 1;
	if(iAstSteps == 1 && iAngSteps == 1) return 0.0f;
	//-----------------
	float fAngMax, fAstMax, fCCMax = (float)-1e20;
	for(int j=0; j<iAngSteps; j++)
	{	float fAng = pfAngRange[0] + j * fAngStep;
   		for(int i=0; i<iAstSteps; i++)
		{	float fAst = pfAstRange[0] + i * fAstStep;
			float fCC = mCorrelate(fAst, fAng, m_fExtPhase);
			if(fCC <= fCCMax) continue;
			//---------------
			fCCMax = fCC;
			fAngMax = fAng;
			fAstMax = fAst;
		}
	}
	//-----------------
	if(fCCMax <= m_fCCMax) return 0.0f;
	float fErr = fabsf((m_fAstRatio - fAstMax) / (m_fAstRatio + fTiny))
	   + fabs((m_fAstAngle - fAngMax) / (m_fAstAngle + fTiny));
	//-----------------
	m_fAstRatio = fAstMax;
	m_fAstAngle = fAngMax;
	m_fCCMax = fCCMax;
	return fErr;
}

float CFindDefocus2D::mRefineAstMag(float fAstRange)
{
        float fTiny = (float)1e-20;
        if(fAstRange < fTiny) return 0.0f;
        //-----------------
        float fMin = fmax(m_fAstRatio - fAstRange * 0.5f, m_afAstRange[0]);
	float fMax = fmin(m_fAstRatio + fAstRange * 0.5f, m_afAstRange[1]);
	//-----------------
	int iSteps = 21;
	float fStep = (fMax - fMin) / (iSteps - 1);
	if(fStep < 0.001f) fStep = 0.001f;
	iSteps = (int)((fMax - fMin) / fStep) / 2 * 2 + 1;
	if(iSteps == 1) return 0.0f;	
        //-----------------
        float fAstMax, fCCMax = (float)-1e20;
	for(int i=0; i<iSteps; i++)
	{	float fAst = fMin + i * fStep;
		float fCC = mCorrelate(fAst, m_fAstAngle, m_fExtPhase);
		if(fCC <= fCCMax) continue;
		//---------------
		fCCMax = fCC;
		fAstMax = fAst;
        }
        //-----------------
        if(fCCMax <= m_fCCMax) return 0.0f;
        float fErr = fabsf((m_fAstRatio - fAstMax) / (m_fAstRatio + fTiny));
        //-----------------
        m_fAstRatio = fAstMax;
        m_fCCMax = fCCMax;
        return fErr;
}

float CFindDefocus2D::mRefineAstAng(float fAngRange)
{
        float fTiny = (float)1e-20;
	if(fAngRange < fTiny) return 0.0f;
	//-----------------
	float fMin = fmax(m_fAstAngle - fAngRange * 0.5f, m_afAngRange[0]);
        float fMax = fmin(m_fAstAngle + fAngRange * 0.5f, m_afAngRange[1]);
        //-----------------
        int iSteps = 51;
        float fStep = (fMax - fMin) / (iSteps - 1);
        if(fStep < 1.0f) fStep = 1.0f;
        iSteps = (int)((fMax - fMin) / fStep) / 2 * 2 + 1;
        if(iSteps == 1) return 0.0f;
        //-----------------
        float fAngMax, fCCMax = (float)-1e20;
        for(int i=0; i<iSteps; i++)
        {       float fAng = fMin + i * fStep;
                float fCC = mCorrelate(m_fAstRatio, fAng, m_fExtPhase);
                if(fCC <= fCCMax) continue;
                //---------------
                fCCMax = fCC;
                fAngMax = fAng;
        }
        //-----------------
        if(fCCMax <= m_fCCMax) return 0.0f;
        float fErr = fabsf((m_fAstAngle - fAngMax) / (m_fAstAngle + fTiny));
        //-----------------
        m_fAstAngle = fAngMax;
        m_fCCMax = fCCMax;
        return fErr;
}

float CFindDefocus2D::mRefineDfMean(float fDfRange)
{
	float fTiny = (float)1e-30;
	if(fDfRange < fTiny) return 0.0f;
	//-----------------
	float fDfMeanOld = m_fDfMean;
	//-----------------
	int iSteps = 31;
	float fMin = fmax(m_fDfMean - fDfRange * 0.5f, m_afDfRange[0]);
	float fMax = fmin(m_fDfMean + fDfRange * 0.5f, m_afDfRange[1]);
	float fStep = (fMax - fMin) / iSteps;
	if(fStep < 20) fStep = 20.0f;
	iSteps = (int)((fMax - fMin) / fStep) / 2 * 2 + 1;
	if(iSteps == 1) return 0.0f;
	//-----------------
	float fDfMax = 0.0f, fCCMax = (float)-1e20;
	for(int i=0; i<iSteps; i++)
	{	m_fDfMean = fMin + i * fStep;
		float fCC = mCorrelate(m_fAstRatio, m_fAstAngle, m_fExtPhase);
		if(fCC <= fCCMax) continue;
		//----------------
		fDfMax = m_fDfMean;
		fCCMax = fCC;
	}
	//-----------------
	if(fCCMax <= m_fCCMax)
	{	m_fDfMean = fDfMeanOld;
		return 0.0f;
	}
	//-----------------
	float fErr = fabsf((m_fDfMean - fDfMax) / (m_fDfMean + fTiny));
	m_fDfMean = fDfMax;
	m_fCCMax = fCCMax;
	return fErr;
}

float CFindDefocus2D::mRefinePhase(float fPhaseRange)
{
	float fTiny = (float)1e-30;
	if(fPhaseRange < fTiny) return 0.0f;
	//-----------------
	int iSteps = 61;
	float fStep = fPhaseRange / iSteps;
	if(fStep < 0.03f) return 0.0f;
	//-----------------
	if(fStep > 1.0f)
	{	fStep = 1.0f;
		iSteps = (int)(fPhaseRange / fStep);
		iSteps = iSteps / 2 * 2 + 1;
	}
	//-----------------
	float fMinPhase = m_afPhaseRange[0] - m_afPhaseRange[1] * 0.5f;
	float fMaxPhase = m_afPhaseRange[1] + m_afPhaseRange[1] * 0.5f;
	if(fMinPhase < 0) fMinPhase = 0.0f;
	if(fMaxPhase > 150) fMaxPhase = 150.0f;
	//-----------------
	float fCCMax = (float)-1e20, fPhaseMax = 0.0f, fPhase = 0.0f;
	for(int i=0; i<iSteps; i++)
	{	fPhase = m_fExtPhase + (i - iSteps / 2)  * fStep;
		if(fPhase < fMinPhase || fPhase > fMaxPhase) continue;
		//----------------
		float fCC = mCorrelate(m_fAstRatio, m_fAstAngle, fPhase);
		if(fCC <= fCCMax) continue;
		//----------------
		fPhaseMax = fPhase;
		fCCMax = fCC;
	}
	if(fCCMax <= m_fCCMax) return 0.0f;
	//-----------------
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
	   m_gfCtf2D, m_aiCmpSize);
	float fCC = m_pGCtfCC2D->DoIt(m_gfCtf2D, m_gfSpect);
	return fCC;
}

void CFindDefocus2D::mCalcCtfRes(void)
{       
	float fExtPhaseRad = m_fExtPhase * s_fD2R;
	float fAstRad = m_fAstAngle * s_fD2R;
	//-----------------
	float fDfMin = CFindCtfHelp::CalcDfMin(m_fDfMean, m_fAstRatio)
	   / m_pCtfParam->m_fPixSize;
	float fDfMax = CFindCtfHelp::CalcDfMax(m_fDfMean, m_fAstRatio)
	   / m_pCtfParam->m_fPixSize;
	//-----------------
	m_aGCalcCtf2D.DoIt(fDfMin, fDfMax, fAstRad, fExtPhaseRad,
	   m_gfCtf2D, m_aiCmpSize);
	//-----------------
	GSpectralCC2D gSpectCC;
	gSpectCC.SetSize(m_aiCmpSize);
	int iShell = gSpectCC.DoIt(m_gfCtf2D, m_gfSpect);
	//-----------------
	m_fCtfRes = m_aiCmpSize[1] * m_pCtfParam->m_fPixSize / iShell;
}

void CFindDefocus2D::mGetRange
(	float fCentVal,
	float fRange,
	float* pfMinMax,
	float* pfRange
)
{	pfRange[0] = fCentVal - fRange * 0.5f;
	pfRange[1] = fCentVal + fRange * 0.5f;
	if(pfRange[0] < pfMinMax[0]) pfRange[0] = pfMinMax[0];
	if(pfRange[1] > pfMinMax[1]) pfRange[1] = pfMinMax[1];
}

