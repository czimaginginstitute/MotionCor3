#include "CFindCtfInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <math.h>
#include <stdio.h>
#include <memory.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2;
using namespace MotionCor2::FindCtf;

//------------------------------
// Debugging code
//------------------------------
//Util::CSaveTempMrc s_saveTempMrc;
//static int s_iCount = 0;

CGenAvgSpectrum::CGenAvgSpectrum(void)
{
	m_fOverlap = 0.50f;
	m_pGCalcMoment2D = new Util::GCalcMoment2D;
}

CGenAvgSpectrum::~CGenAvgSpectrum(void)
{
	delete m_pGCalcMoment2D;
	m_pGCalcMoment2D = 0L;
}

int CGenAvgSpectrum::GetSpectPixels(void)
{	
	int iPixels = m_aiSpectSize[0] * m_aiSpectSize[1];
	return iPixels;
}

void CGenAvgSpectrum::SetSizes(int* piImgSize, bool bPadded, int iTileSize)
{
	m_aiImgSize[0] = piImgSize[0];
	m_aiImgSize[1] = piImgSize[1];
	m_bPadded = bPadded;
	if(bPadded) m_aiImgSize[0] = (m_aiImgSize[0] / 2 - 1) * 2;
	//--------------------------------------------------------
	m_aiSpectSize[0] = iTileSize / 2 + 1;
	m_aiSpectSize[1] = iTileSize;
	//-----------------------------------
	m_iOverlap = (int)(iTileSize * m_fOverlap);
	m_iOverlap = m_iOverlap / 2 * 2;
	//------------------------------
	int iSize = iTileSize - m_iOverlap;
	m_aiNumTiles[0] = (m_aiImgSize[0] - m_iOverlap) / iSize;
	m_aiNumTiles[1] = (m_aiImgSize[1] - m_iOverlap) / iSize;
	//------------------------------------------------------
	m_aiOffset[0] = (m_aiImgSize[0] - m_aiNumTiles[0] * iTileSize
	   + (m_aiNumTiles[0] - 1) * m_iOverlap) / 2;
	m_aiOffset[1] = (m_aiImgSize[1] - m_aiNumTiles[1] * iTileSize
	   + (m_aiNumTiles[1] - 1) * m_iOverlap) / 2;
	//-------------------------------------------
	int aiPadSize[2] = {m_aiSpectSize[0] * 2, m_aiSpectSize[1]};
	m_pGCalcMoment2D->SetSize(aiPadSize, true);
}

void CGenAvgSpectrum::DoIt
(	float* gfImage,
	float* gfBuf,
	float* gfAvgSpect
)
{	m_gfImg = gfImage;
	m_gfPadTile = gfBuf;
	m_gfAvgSpect = gfAvgSpect;
	//------------------------
	int iPadSize = m_aiSpectSize[0] * 2 * m_aiSpectSize[1];
	m_gfTileSpect = m_gfPadTile + iPadSize;
	//-------------------------------------
	mAverage();
	mRmBackground();
}

void CGenAvgSpectrum::mAverage(void)
{
	Util::GAddFrames aGAddFrames;
	int iNumTiles = m_aiNumTiles[0] * m_aiNumTiles[1];
	float fFactor2 = 1.0f / iNumTiles;
	//---------------------------------------------------------
	// use m_gfTileSpect to hold the averaged spectrum while
	// using m_gfAvgSpect to hold the tile spectrum to avoid
	// an extra copy in mReBackground.
	//---------------------------------------------------------
	int iBytes = sizeof(float) * m_aiSpectSize[0] * m_aiSpectSize[1];
	cudaMemset(m_gfTileSpect, 0, iBytes);
	//-----------------------------------
	Util::CSaveTempMrc saveTmpMrc;
	char acMrcName[128] = {'\0'};

	for(int i=0; i<iNumTiles; i++)
	{	mCalcTileSpectrum(i);
		aGAddFrames.DoIt(m_gfTileSpect, 1.0f, m_gfAvgSpect, 
		   fFactor2, m_gfTileSpect, m_aiSpectSize);
	}
}

void CGenAvgSpectrum::mCalcTileSpectrum(int iTile)
{	
	mExtractPadTile(iTile);
	float fMean = m_pGCalcMoment2D->DoIt(m_gfPadTile, 1, true);
	float fStd = m_pGCalcMoment2D->DoIt(m_gfPadTile, 2, true);
	fStd = fStd - fMean * fMean;
	if(fStd <= 0) fStd = 1.0f;
	else fStd = sqrtf(fStd);
	//---------------------------------------------------------
	Util::GNormalize2D aGNormalize;
	int aiPadSize[] = {m_aiSpectSize[0] * 2, m_aiSpectSize[1]};
	aGNormalize.DoIt(m_gfPadTile, aiPadSize, true, fMean, fStd);
	//----------------------------------------------------------
	Util::GRoundEdge aGRoundEdge;
	float afCent[] = {m_aiSpectSize[1] * 0.5f, m_aiSpectSize[1] * 0.5f};
	float afSize[] = {m_aiSpectSize[1] * 1.0f, m_aiSpectSize[1] * 1.0f};
	aGRoundEdge.SetMask(afCent, afSize);
	aGRoundEdge.DoIt(m_gfPadTile, aiPadSize);
	//----------------------------------------------------------
	// use m_gfAvgSpect as the buffer to hold the tile spectrum 
	// while using m_gfTileSpect to hold the sum in mAverage.
	//----------------------------------------------------------  
	bool bLogrith = true;
	GCalcSpectrum aGCalcSpectrum;
	aGCalcSpectrum.DoPad(m_gfPadTile, m_gfAvgSpect, aiPadSize, !bLogrith);
}

void CGenAvgSpectrum::mExtractPadTile(int iTile)
{	
	int iTileX = iTile % m_aiNumTiles[0];
	int iTileY = iTile / m_aiNumTiles[0];
	int iStartX = m_aiOffset[0] + iTileX * (m_aiSpectSize[1] - m_iOverlap);
	int iStartY = m_aiOffset[1] + iTileY * (m_aiSpectSize[1] - m_iOverlap);
	//---------------------------------------------------------------------
	int iPadImgX = m_aiImgSize[0];
	if(m_bPadded) iPadImgX = (m_aiImgSize[0] / 2 + 1) * 2;
	float* gfSrc = m_gfImg + iStartY * iPadImgX + iStartX;
	//------------------------------------------------------
	int aiDstSize[] = {m_aiSpectSize[0] * 2, m_aiSpectSize[1]};
	int iCopyX = m_aiSpectSize[1]; // unpadded tile size
	//---------------------------------------------------------
	Util::GPartialCopy aGPartCopy;
	aGPartCopy.DoIt(gfSrc, iPadImgX, m_gfPadTile, iCopyX, aiDstSize);
}

void CGenAvgSpectrum::mRmBackground(void)
{
	float fMinFreq = 1.0f / 15.0f; // relative freq
	int iBoxSize = (int)(m_aiSpectSize[1] * fMinFreq);
	iBoxSize = iBoxSize / 2 * 2 + 1;
	if(iBoxSize < 7) iBoxSize = 7;
	//---------------------------------------------------
	// m_gfTileSpect stores the averaged spectrum before
	// the background is removed, see mAverage().
	//---------------------------------------------------
	GRmBackground2D aGRmBackground;
	aGRmBackground.DoIt(m_gfTileSpect, m_gfAvgSpect, 
	   m_aiSpectSize, iBoxSize); 
	//-----------------------------------------------------
	// calculate mean and sigma that are used to determine
	// hot and cold speckle in the spectrum.
	//-----------------------------------------------------
	Util::GCalcMoment2D calcMoment2D;
        calcMoment2D.SetSize(m_aiSpectSize, false);
        float fMean = calcMoment2D.DoIt(m_gfAvgSpect, 1, true);
        float fStd = calcMoment2D.DoIt(m_gfAvgSpect, 2, true);
        fStd = fStd - fMean * fMean;
        if(fStd < 1.0f) return;
	//-----------------
        fStd = (float)sqrtf(fStd);
	float fMin = fMean - 1.0f * fStd;
	float fMax = fMean + 1.0f * fStd;
	Util::GThreshold2D aGThreshold;
	aGThreshold.DoIt(m_gfAvgSpect, m_aiSpectSize, false, fMin, fMax);
}
