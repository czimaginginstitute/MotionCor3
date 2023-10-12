#include "CUtilInc.h"
#include <Util/Util_LinEqs.h>
#include <memory.h>
#include <math.h>
#include <stdio.h>

using namespace MotionCor2::Util;

CSplineFit1D::CSplineFit1D(void)
{
	m_pfTerms = 0L;
	m_pfCoeff = 0L;
	m_pfMatrix = 0L;
}

CSplineFit1D::~CSplineFit1D(void)
{
	this->Clean();
}

void CSplineFit1D::Clean(void)
{
	if(m_pfTerms != 0L) delete[] m_pfTerms;
	if(m_pfCoeff != 0L) delete[] m_pfCoeff;
	if(m_pfMatrix != 0L) delete[] m_pfMatrix;
	m_pfTerms = 0L;
	m_pfCoeff = 0L;
	m_pfMatrix = 0L;
}

void CSplineFit1D::SetNumKnots(int iNumKnots)
{
	this->Clean();
	//------------
	m_iNumKnots = iNumKnots;
	m_iDim = m_iNumKnots + 3;
	//-----------------------
	m_pfTerms = new float[m_iDim * 2];
	m_pfCoeff = new float[m_iDim];
	m_pfMatrix = new float[m_iDim * m_iDim];
}

bool CSplineFit1D::DoIt
(	float* pfX, float* pfY, bool* pbBad, int iSize,
	float* pfKnots, float* pfFit, float fReg
)
{	m_pfKnots = pfKnots;
	//------------------
	memset(m_pfMatrix, 0, sizeof(float) * m_iDim * m_iDim);
	memset(m_pfCoeff, 0, sizeof(float) * m_iDim);
	memset(m_pfTerms, 0, sizeof(float) * m_iDim * 2);
	float* pfCurTerms = m_pfTerms + m_iDim;
	//-------------------------------------
	for(int i=0; i<iSize; i++)
	{	if(pbBad != 0L && pbBad[i]) continue;
		mCalcTerms(pfX[i]);
		if(fReg != 0.0f) mCalcCurTerms(pfX[i]);
		for(int r=0; r<m_iDim; r++)
		{	for(int c=0; c<m_iDim; c++)
			{	int k = r * m_iDim + c;
				m_pfMatrix[k] += (m_pfTerms[r] * m_pfTerms[c]
				   + fReg * pfCurTerms[r] * pfCurTerms[c]);
			}
			m_pfCoeff[r] += (m_pfTerms[r] * pfY[i]);
		}
	}
	//-----------------------------------------------------
	Util_LinEqs aLinEqs;
	bool bSuccess = aLinEqs.DoIt(m_pfMatrix, m_pfCoeff, m_iDim);
	if(!bSuccess) return false;
	//-------------------------
	for(int i=0; i<iSize; i++)
	{	pfFit[i] = mCalcFit(pfX[i]);
	}
	return true;
}

float CSplineFit1D::mCalcFit(float fX)
{
	mCalcTerms(fX);
	float fFitVal = 0.0f;
	for(int i=0; i<m_iDim; i++)
	{	fFitVal += m_pfCoeff[i] * m_pfTerms[i];
	}
	return fFitVal;
}

void CSplineFit1D::mCalcTerms(float fX)
{
	m_pfTerms[0] = 1;
	for(int i=1; i<3; i++)
	{	m_pfTerms[i] = m_pfTerms[i-1] * fX;
	}
	//-----------------------------------------
	memset(m_pfTerms+3, 0, sizeof(float) * m_iNumKnots);	
	for(int i=0; i<m_iNumKnots; i++)
	{	if(fX < m_pfKnots[i]) break;
		float fVal = fX - m_pfKnots[i];
		m_pfTerms[i+3] = fVal * fVal;
	}
}

void CSplineFit1D::mCalcCurTerms(float fX)
{
	float* pfCurTerms = m_pfTerms + m_iDim;
	memset(pfCurTerms, 0, sizeof(float) * m_iDim);
	//--------------------------------------------
	pfCurTerms[1] = 1.0f;
	pfCurTerms[2] = 2.0f * fX;
	for(int i=0; i<m_iNumKnots; i++)
	{	float fVal = fX - m_pfKnots[i];
		if(fVal > 0) break;
		pfCurTerms[i+3] = 2.0f * fVal;
	}
}
		
		

