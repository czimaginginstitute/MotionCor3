#include "CFindCtfInc.h"
#include <math.h>
#include <stdio.h>
#include <memory.h>

using namespace MotionCor2::FindCtf;

static float s_fD2R = 0.01745329f;

CCtfParam::CCtfParam(void)
{
	m_fPixSize = 1.0f;
	m_fDfMin = 0.0f;
	m_fDfMax = 0.0f;
	m_fAstAng = 0.0f;
	m_fExtPhase = 0.0f;
	m_fWaveLen = 0.0f;
	m_fScore = 0.0f;
	m_fCtfRes = 0.0f;
}

CCtfParam::CCtfParam(CCtfParam &ctfParam)
{
	this->Setup(&ctfParam);
}	

CCtfParam::~CCtfParam(void)
{
}

void CCtfParam::Setup
(	float fKv, float fCs, // kV, mm
	float fAmpContrast,
	float fPixSize
)
{	m_fKv = fKv;
	m_fCs = (float)(fCs * 1e7 / fPixSize);
	m_fAmpCont = fAmpContrast;
	m_fPixSize = fPixSize;
	//---------------------------
	double dWl = 12.26 / sqrt(fKv * 1000 + 0.9784 * fKv * fKv);
        m_fWaveLen = (float)(dWl / m_fPixSize);
	//---------------------------
	float fAC2 = m_fAmpCont * m_fAmpCont;
	m_fAmpPhaseShift = atan(fAmpContrast / sqrt(1 - fAC2)); 
	//---------------------------
	m_fDfMin = 0.0f;
	m_fDfMax = 0.0f;
	m_fAstAng = 0.0f;
	m_fExtPhase = 0.0f;
	m_fScore = 0.0f;
	m_fCtfRes = 0.0f;
}

void CCtfParam::Setup(CCtfParam* pCtfParam)
{
	m_fWaveLen = pCtfParam->m_fWaveLen;
	m_fCs = pCtfParam->m_fCs;
	m_fAmpCont = pCtfParam->m_fAmpCont;
	m_fExtPhase = pCtfParam->m_fExtPhase;
	m_fDfMax = pCtfParam->m_fDfMax;
	m_fDfMin = pCtfParam->m_fDfMin;
	m_fAstAng = pCtfParam->m_fAstAng;
	m_fPixSize = pCtfParam->m_fPixSize;
	m_fScore = pCtfParam->m_fScore;
	m_fCtfRes = pCtfParam->m_fCtfRes;
	m_fAmpPhaseShift = pCtfParam->m_fAmpPhaseShift;
}

void CCtfParam::SetDfMin(float fDfMin, bool bAngstrom)
{
	if(bAngstrom) m_fDfMin = fDfMin / m_fPixSize;
	else m_fDfMin = fDfMin;
}

void CCtfParam::SetDfMax(float fDfMax, bool bAngstrom)
{
	if(bAngstrom) m_fDfMax = fDfMax / m_fPixSize;
	else m_fDfMax = fDfMax;
}

void CCtfParam::SetDfs(float fDfMin, float fDfMax, bool bAngstrom)
{
	m_fDfMin = fDfMin;
	m_fDfMax = fDfMax;
	if(bAngstrom) 
	{	m_fDfMin /= m_fPixSize;
		m_fDfMax /= m_fPixSize;
	}
}

void CCtfParam::SetAstAngle(float fAstAng, bool bDegree)
{
	if(bDegree) m_fAstAng = fAstAng * s_fD2R;
	else m_fAstAng = fAstAng;
}

void CCtfParam::SetExtPhase(float fExtPhase, bool bDegree)
{
	if(bDegree) m_fExtPhase = fExtPhase * s_fD2R;
	else m_fExtPhase = fExtPhase;
}

void CCtfParam::SetPixSize(float fPixSize)
{
	float fRatio = m_fPixSize / fPixSize;
	m_fPixSize = fPixSize;
	m_fDfMin *= fRatio;
	m_fDfMax *= fRatio;
	m_fWaveLen *= fRatio;
}

float CCtfParam::GetWavelength(bool bAngstrom)
{
	if(bAngstrom) return (m_fWaveLen * m_fPixSize);
	else return m_fWaveLen;
}

float CCtfParam::GetDfMax(bool bAngstrom)
{
	if(bAngstrom) return (m_fDfMax * m_fPixSize);
	else return m_fDfMax;
}

float CCtfParam::GetDfMin(bool bAngstrom)
{
	if(bAngstrom) return (m_fDfMin * m_fPixSize);
	else return m_fDfMin;
}

float CCtfParam::GetAstAng(bool bDegree)
{
	if(bDegree) return (m_fAstAng / s_fD2R);
	else return m_fAstAng;
}

float CCtfParam::GetExtPhase(bool bDegree)
{
	if(bDegree) return (m_fExtPhase / s_fD2R);
	else return m_fExtPhase;
}

CCtfParam* CCtfParam::GetCopy(void)
{
	CCtfParam* pCopy = new CCtfParam;
	pCopy->Setup(this);
	return pCopy;
}

float CCtfParam::GetPixSize(void)
{
	return m_fPixSize;
}

CCtfTheory::CCtfTheory(void)
{
	m_fPI = (float)(4.0 * atan(1.0));
	m_pCtfParam = new CCtfParam;
}

CCtfTheory::~CCtfTheory(void)
{
	if(m_pCtfParam != 0L) delete m_pCtfParam;
}

void CCtfTheory::Setup
(	float fKv, // keV
	float fCs, // mm
	float fAmpContrast,
	float fPixelSize // A
)
{	m_pCtfParam->Setup(fKv, fCs, fAmpContrast, fPixelSize);
}

void CCtfTheory::Setup(CCtfParam* pCtfParam)
{	
	m_pCtfParam->Setup(pCtfParam);
}

float CCtfTheory::Evaluate
(	float fFreq,   // relative frquency in [-0.5, 0.5]
	float fAzimuth
)
{	float fPhaseShift = CalcPhaseShift(fFreq, fAzimuth);
	return (float)(-sin(fPhaseShift));
}

//------------------------------------------------------------------------------
// 1. Return number of extrema before the given spatial frequency.
//    Eq. 11 of Rohou & Grigoriff 2015
// 2. fFrequency is relative frequency in [-0.5, +0.5].
//------------------------------------------------------------------------------
int CCtfTheory::CalcNumExtrema
(	float fFreq,
	float fAzimuth
)
{	float fPhaseShift = CalcPhaseShift(fFreq, fAzimuth);
	int iNumExtrema = (int)(fPhaseShift / m_fPI + 0.5f);
	return iNumExtrema;
}

//-----------------------------------------------------------------------------
// 1. Return the spatial frequency of Nth zero.
//    The returned frequency is in 1/pixel.
//-----------------------------------------------------------------------------
float CCtfTheory::CalcNthZero(int iNthZero, float fAzimuth)
{
	float fPhaseShift = iNthZero * m_fPI;
	float fFreq = CalcFrequency(fPhaseShift, fAzimuth);
	return fFreq;
}

//-----------------------------------------------------------------------------
//  1. Calculate defocus in pixel at the given azumuth angle.
//  2. fDefocusMin, fDefocusMax are the min, max defocus at the major and
//     minor axis.
//  3. fAstAzimuth is the angle of the astimatism, or the major axis.
//  4. fDefocusMax must be larger than fDefocusMin.
//-----------------------------------------------------------------------------
float CCtfTheory::CalcDefocus(float fAzimuth)
{
	float fSumDf = m_pCtfParam->m_fDfMax
		+ m_pCtfParam->m_fDfMin;
	float fDifDf = m_pCtfParam->m_fDfMax
		- m_pCtfParam->m_fDfMin;
	double dCosA = cos(2.0 * (fAzimuth - m_pCtfParam->m_fAstAng));
	float fDefocus = (float)(0.5 * (fSumDf + fDifDf * dCosA));
	return fDefocus;
}

float CCtfTheory::CalcPhaseShift
(	float fFreq,
	float fAzimuth
)
{	float fS2 = fFreq * fFreq;
	float fW2 = m_pCtfParam->m_fWaveLen
		* m_pCtfParam->m_fWaveLen;
	float fDefocus = CalcDefocus(fAzimuth);
	float fPhaseShift = m_fPI * m_pCtfParam->m_fWaveLen * fS2
		* (fDefocus - 0.5f * fW2 * fS2 * m_pCtfParam->m_fCs)
		+ m_pCtfParam->m_fAmpPhaseShift
		+ m_pCtfParam->m_fExtPhase;
	return fPhaseShift;
}

//-----------------------------------------------------------------------------
// 1. Returen spatial frequency in 1/pixel given phase shift and fAzimuth
//    in radian.
//-----------------------------------------------------------------------------
float CCtfTheory::CalcFrequency
(	float fPhaseShift,
	float fAzimuth
)
{	float fDefocus = CalcDefocus(fAzimuth);
	double dW3 = pow(m_pCtfParam->m_fWaveLen, 3.0);
	double a = -0.5 * m_fPI * dW3 * m_pCtfParam->m_fCs;
	double b = m_fPI * m_pCtfParam->m_fWaveLen * fDefocus;
	double c = m_pCtfParam->m_fExtPhase
		+ m_pCtfParam->m_fAmpPhaseShift;
	double dDet = b * b - 4.0 * a * (c - fPhaseShift);
	//------------------------------------------------
	if(m_pCtfParam->m_fCs == 0)
	{	double dFreq2 = (fPhaseShift - c) / b;
		if(dFreq2 > 0) return (float)sqrt(dFreq2);
		else return 0.0f;
	}
	else if(dDet < 0.0)
	{	return 0.0f;
	}
	else
	{	double dSln1 = (-b + sqrt(dDet)) / (2 * a);
		double dSln2 = (-b - sqrt(dDet)) / (2 * a);
		if(dSln1 > 0) return (float)sqrt(dSln1);
		else if(dSln2 > 0) return (float)sqrt(dSln2);
		else return 0.0f;
	}
}

//-----------------------------------------------------------------------------
// Given acceleration voltage in keV, return the electron wavelength.
//-----------------------------------------------------------------------------
float CCtfTheory::mCalcWavelength(float fKv)
{
	double dWl = 12.26 / sqrt(fKv * 1000 + 0.9784 * fKv * fKv);
	return (float)dWl;
}

//-----------------------------------------------------------------------------
// Enforce that m_fDfMax > m_fDfMin and -90 < m_fAstAng < 90.
//-----------------------------------------------------------------------------
void CCtfTheory::mEnforce(void)
{
	m_pCtfParam->m_fAstAng -= m_fPI
	   * round(m_pCtfParam->m_fAstAng / m_fPI);
	if(m_pCtfParam->m_fDfMax < m_pCtfParam->m_fDfMin)
	{	float fTemp = m_pCtfParam->m_fDfMax;
		m_pCtfParam->m_fDfMax = m_pCtfParam->m_fDfMin;
		m_pCtfParam->m_fDfMin = fTemp;
	}
}
