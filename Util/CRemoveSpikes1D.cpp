#include "CUtilInc.h"
#include <Util/Util_LinEqs.h>
#include <memory.h>
#include <math.h>
#include <stdio.h>

using namespace MotionCor2::Util;

CRemoveSpikes1D::CRemoveSpikes1D(void)
{
	m_pbBad = 0L;
	m_pfFitX = 0L;
	m_pfKnots = 0L;
}

CRemoveSpikes1D::~CRemoveSpikes1D(void)
{
	this->Clean();
}

void CRemoveSpikes1D::Clean(void)
{
	if(m_pbBad != 0L) delete[] m_pbBad;
	if(m_pfFitX != 0L) delete[] m_pfFitX;
	if(m_pfKnots != 0L) delete[] m_pfKnots;
	m_pbBad = 0L;
	m_pfFitX = 0L;
	m_pfKnots = 0L;
}

void CRemoveSpikes1D::SetDataSize(int iSize)
{
	this->Clean();
	m_iSize = iSize;
	m_pbBad = new bool[m_iSize];
	m_pfFitX = new float[m_iSize * 4];
	m_pfFitY = m_pfFitX + m_iSize;
	m_pfTime = m_pfFitX + m_iSize * 2;
	m_pfBuf = m_pfFitX + m_iSize * 3;
	//-------------------------------
	float fFact = 1.0f / m_iSize;
	for(int i=0; i<m_iSize; i++) 
	{	m_pfTime[i] = (float)sqrt(i * fFact);
	}
	//-------------------------------------------
	int iKnotDist = 15;
	m_iNumKnots = (m_iSize - 2 * iKnotDist) / iKnotDist;
	if(m_iNumKnots < 0) m_iNumKnots = 0;
	else if(m_iNumKnots > 6) m_iNumKnots = 6;
	if(m_iNumKnots == 0 && m_iSize >= (2 * iKnotDist)) m_iNumKnots = 1;
	//-----------------------------------------------------------------
	if(m_iNumKnots > 0) m_pfKnots = new float[m_iNumKnots];
	for(int i=0; i<m_iNumKnots; i++)
	{	m_pfKnots[i] = m_pfTime[(i+1) * iKnotDist];
	}
	m_aSplineFit.SetNumKnots(m_iNumKnots);
}

void CRemoveSpikes1D::DoIt(float* pfShiftX, float* pfShiftY, bool bSingle)
{
	for(int i=0; i<2; i++)
	{	mRemoveSingle(pfShiftX);
		mRemoveSingle(pfShiftY);
	}
	if(bSingle) return;
	//-----------------
	memset(m_pbBad, 0, sizeof(bool) * m_iSize);
	float fReg = (float)1e-20; //(0.0001f / m_iSize / m_iSize) * m_iNumKnots;
	m_aSplineFit.DoIt(m_pfTime, pfShiftX, m_pbBad, m_iSize,
	   m_pfKnots, m_pfFitX, fReg);
	m_aSplineFit.DoIt(m_pfTime, pfShiftY, m_pbBad, m_iSize,
	   m_pfKnots, m_pfFitY, fReg);	
	//----------------------------
	memcpy(m_pfBuf, pfShiftX, sizeof(float) * m_iSize);
	for(int i=0; i<1; i++)
	{	float fTol = 3.0f - i * 3.0f;
		bool bHasBad = mFindBad(m_pfBuf, m_pfFitX, fTol);
		if(!bHasBad) break;
		m_aSplineFit.DoIt(m_pfTime, m_pfBuf, m_pbBad, 
		   m_iSize, m_pfKnots, m_pfFitX, fReg);
	}
	//---------------------------------------------
	memcpy(m_pfBuf, pfShiftY, sizeof(float) * m_iSize);
	for(int i=0; i<1; i++)
	{	float fTol = 3.0f - i * 3.0f;
		bool bHasBad = mFindBad(m_pfBuf, m_pfFitY, fTol);
		if(!bHasBad) break;
		m_aSplineFit.DoIt(m_pfTime, pfShiftY, m_pbBad, 
		   m_iSize, m_pfKnots, m_pfFitY, fReg);
	}
	//---------------------------------------------
	for(int i=0; i<m_iSize; i++)
        {       /*printf(" %4d  %8.2f  %8.2f  %8.2f  %8.2f\n", i,
                   pfShiftX[i], m_pfFitX[i],
                   pfShiftY[i], m_pfFitY[i]);*/
		//---------------------------
		float fTol = 5.0f;
		if(i > 20) fTol = 4.0f;
		if(i < 50) fTol = 3.0f;
		float fDifX = m_pfFitX[i] - pfShiftX[i];
		float fDifY = m_pfFitY[i] - pfShiftY[i];
		if(fabs(fDifX) > fTol) pfShiftX[i] = m_pfFitX[i];
		if(fabs(fDifY) > fTol) pfShiftY[i] = m_pfFitY[i];
	}
}

bool CRemoveSpikes1D::mFindBad
(	float* pfRawShift, 
	float* pfFitShift,
	float fTol
)
{	bool bHasBad = false;
	memset(m_pbBad, 0, sizeof(bool) * m_iSize);
	//-----------------------------------------
	for(int i=0; i<m_iSize; i++)
	{	if(fabs(pfFitShift[i] - pfRawShift[i]) > fTol)
		{	//m_pbBad[i] = true;
			if(fabs(pfRawShift[i]) > fabs(pfFitShift[i]))
			{	pfRawShift[i] = pfFitShift[i];
				bHasBad = true;
			}
		}
	}
	return bHasBad;
}

void CRemoveSpikes1D::mRemoveSingle(float* pfRawShift)
{
	memcpy(m_pfBuf, pfRawShift, sizeof(float) * m_iSize);
	//---------------------------------------------------
	float fTol = 5.0f;
	int iEnd = m_iSize - 1;
	for(int i=1; i<iEnd; i++)
	{	if(fabs(m_pfBuf[i] - m_pfBuf[i-1]) < fTol) continue;
		if(fabs(m_pfBuf[i] - m_pfBuf[i+1]) < fTol) continue;
		pfRawShift[i] = (m_pfBuf[i-1] + m_pfBuf[i+1]) / 2.0f;
	}
	//-----------------------------------------------------------
	fTol = 10.0f;
	if(fabs(m_pfBuf[0] - m_pfBuf[1]) > fTol &&
	   fabs(m_pfBuf[0]) > fabs(m_pfBuf[1]))
	{	pfRawShift[0] = 2.0f * m_pfBuf[1] - m_pfBuf[2];
	}
	if(fabs(m_pfBuf[iEnd] - m_pfBuf[iEnd-1]) > fTol &&
	   fabs(m_pfBuf[iEnd]) > fabs(m_pfBuf[iEnd-1]))
	{	pfRawShift[iEnd] = 2.0f * m_pfBuf[iEnd-1] - m_pfBuf[iEnd-2];
	}
} 
