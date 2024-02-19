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

CFullSpectrum::CFullSpectrum(void)
{
	m_gfFullSpect = 0L; // does not own it, do NOT clean
}

CFullSpectrum::~CFullSpectrum(void)
{
}

void CFullSpectrum::ToHost(float* pfFullSpect)
{
	size_t tBytes = sizeof(float) * m_aiFullSize[0] * m_aiFullSize[1];
	cudaMemcpy(pfFullSpect, m_gfFullSpect, tBytes, cudaMemcpyDefault);
}

void CFullSpectrum::Create
(	float* gfHalfSpect,
	float* gfCtfBuf,
	int* piSpectSize,
	CCtfParam* pCtfParam,
	float* pfResRange,
	float* gfFullSpect
)
{	m_aiSpectSize[0] = piSpectSize[0];
	m_aiSpectSize[1] = piSpectSize[1];
	m_afResRange[0] = pfResRange[0];
	m_afResRange[1] = pfResRange[1];
	//-----------------
	m_pCtfParam = pCtfParam;
	m_gfHalfSpect = gfHalfSpect;
	m_gfCtfBuf = gfCtfBuf;
	m_gfFullSpect = gfFullSpect;
	//-----------------
	m_aiFullSize[0] = (m_aiSpectSize[0] - 1) * 2;
	m_aiFullSize[1] = m_aiSpectSize[1];
	//-----------------
	mGenFullSpectrum();
	if(m_pCtfParam != 0L) mEmbedCTF();
}
/*
void CFullSpectrum::EmbedCTF
(	float* gfFullSpect, int* piFullSize, bool bPadded,
	CCtfParam* pCtfParam, float* pfResRange
)
{
}
*/
void CFullSpectrum::mGenFullSpectrum(void)
{
	GCalcSpectrum gCalcSpect;
	bool bFullPadded = true;
	gCalcSpect.GenFullSpect(m_gfHalfSpect, m_aiSpectSize, 
	   m_gfFullSpect, !bFullPadded);
	//-----------------
	Util::GCalcMoment2D gCalcMoment;
	bool bSync = true, bPadded = true;
	gCalcMoment.SetSize(m_aiSpectSize, !bPadded);
	m_fMean = gCalcMoment.DoIt(m_gfHalfSpect, 1, bSync);
	m_fStd = gCalcMoment.DoIt(m_gfHalfSpect, 2, bSync);
	m_fStd = m_fStd - m_fMean * m_fMean;
	if(m_fStd < 0) m_fStd = 0.0f;
	else m_fStd = sqrt(m_fStd);
}
/*
void CFullSpectrum::mEmbedCTF(void)
{
	float fPixelSize = m_pCtfParam->GetPixSize();
	float fMinFreq = fPixelSize / m_afResRange[0];
	float fMaxFreq = fPixelSize / m_afResRange[1];
	float fGain = m_fStd * 1.5f;
	//--------------------------
	GCalcCTF2D gCalcCtf2D;
	gCalcCtf2D.DoIt(m_pCtfParam, m_gfCtfBuf, m_aiSpectSize);
	gCalcCtf2D.EmbedCtf(m_gfCtfBuf, fMinFreq, fMaxFreq,
	   m_fMean, fGain, m_gfFullSpect, m_aiSpectSize);
}
*/
