#include "CAlignInc.h"
#include <stdio.h>
#include <memory.h>

using namespace MotionCor2::Align;

CPeak2D::CPeak2D(void)
{
	memset(m_afShift, 0, sizeof(m_afShift));
}

CPeak2D::~CPeak2D(void)
{
}

void CPeak2D::Mask(float* pfImg, int* piImgSize, int* piPos, int iR)
{
	int iStartX = piPos[0] - iR;
	int iEndX = piPos[0] + iR + 1;
	if(iStartX < 0) iStartX = 0;
	if(iEndX > piImgSize[0]) iEndX = piImgSize[0];
	//--------------------------------------------
	int iStartY = piPos[1] - iR;
	int iEndY = piPos[1] + iR + 1;
	if(iStartY < 0) iStartY = 0;
	if(iEndY > piImgSize[1]) iEndY = piImgSize[1];
	//--------------------------------------------
	for(int y=iStartY; y<iEndY; y++)
	{	double dY2 = (y - piPos[1]) * (y - piPos[1]);
		int i = y * piImgSize[0];
		for(int x=iStartX; x<iEndX; x++)
		{	double dX2 = (x - piPos[0]) * (y - piPos[0]);
			if(sqrt(dX2 + dY2) > iR) continue;
			pfImg[i+x] = 0.0f;
		}
	}
}

void CPeak2D::DoIt(float* pfImg, int* piImgSize)
{
	m_pfImg = pfImg;
	m_aiImgSize[0] = piImgSize[0];
	m_aiImgSize[1] = piImgSize[1];
	//----------------------------
	int aiPeak1[] = {0, 0};
	mSearchIntPeak();
	mSearchFloatPeak();
	m_afShift[0] = m_afPeak[0] - m_aiImgSize[0] / 2;
	m_afShift[1] = m_afPeak[1] - m_aiImgSize[1] / 2;
}

void CPeak2D::mSearchIntPeak(void)
{
	int iPixels = m_aiImgSize[0] * m_aiImgSize[1];
	m_fPeak = m_pfImg[0];
	int iPeak = 0;
	//------------
	for(int i=1; i<iPixels; i++)
	{	if(m_fPeak >= m_pfImg[i]) continue;
		m_fPeak = m_pfImg[i];
		iPeak = i;
	}
	//----------------
	m_aiPeak[0] = iPeak % m_aiImgSize[0];
	m_aiPeak[1] = iPeak / m_aiImgSize[0];
	//-----------------------------------
	if(m_aiPeak[0] == 0) m_aiPeak[0] = 1;
	if(m_aiPeak[1] == 0) m_aiPeak[1] = 1;
	//-----------------------------------
	int iEndX = m_aiImgSize[0] - 1;
	int iEndY = m_aiImgSize[1] - 1;
	if(m_aiPeak[0] == iEndX) m_aiPeak[0] -= 1;
	if(m_aiPeak[1] == iEndY) m_aiPeak[1] -= 1;
}

bool CPeak2D::mIsCentralPeak(void)
{
	double dX = m_aiPeak[0] - m_aiImgSize[0] * 0.5;
	double dY = m_aiPeak[1] - m_aiImgSize[1] * 0.5;
	double dR = sqrt(dX * dX + dY * dY);
	if(dR <= 3) return true;
	else return false;
}

void CPeak2D::mSearchFloatPeak(void)
{
	int ic = m_aiPeak[1] * m_aiImgSize[0] + m_aiPeak[0];
	int xp = ic + 1;
	int xm = ic - 1;
	int yp = ic + m_aiImgSize[0];
	int ym = ic - m_aiImgSize[0];
	//---------------------------
	double a = (m_pfImg[xp] + m_pfImg[xm]) * 0.5 - m_pfImg[ic];
	double b = (m_pfImg[xp] - m_pfImg[xm]) * 0.5;
	double c = (m_pfImg[yp] + m_pfImg[ym]) * 0.5f - m_pfImg[ic];
	double d = (m_pfImg[yp] - m_pfImg[ym]) * 0.5;
	double dCentX = -b / (2 * a + 1e-30);
	double dCentY = -d / (2 * c + 1e-30);
	//-----------------------------------
	if(fabs(dCentX) > 1) dCentX = 0;
	if(fabs(dCentY) > 1) dCentY = 0;
	m_afPeak[0] = (float)(m_aiPeak[0] + dCentX);
	m_afPeak[1] = (float)(m_aiPeak[1] + dCentY);
	/*
	m_fPeak =  (float)(a * dCentX * dCentX + b * dCentX
		+ c * dCentY * dCentY + d * dCentY
		+ m_pfImg[ic]);
	*/
}
