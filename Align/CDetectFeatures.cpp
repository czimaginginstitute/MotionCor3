#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Correct/CCorrectInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <Util/Util_Time.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CDetectFeatures* CDetectFeatures::m_pInstance = 0L;

CDetectFeatures* CDetectFeatures::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CDetectFeatures;
	return m_pInstance;
}

void CDetectFeatures::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CDetectFeatures::CDetectFeatures(void)
{
	m_aiBinnedSize[0] = 128;
	m_aiBinnedSize[1] = 128;
	m_pbFeatures = 0L;
	m_pbUsed = 0L;
	m_pfCenters = 0L; // patch centers
}

CDetectFeatures::~CDetectFeatures(void)
{
	mClean();
}

void CDetectFeatures::mClean(void)
{
	if(m_pbFeatures != 0L) delete[] m_pbFeatures;
	if(m_pbUsed != 0L) delete[] m_pbUsed;
	if(m_pfCenters != 0L) delete[] m_pfCenters;
	m_pbFeatures = 0L;
	m_pbUsed = 0L;
	m_pfCenters = 0L;
}


void CDetectFeatures::DoIt(CStackShift* pXcfStackShift, int* piNumPatches)
{
	mClean();
	int iBinnedSize = m_aiBinnedSize[0] * m_aiBinnedSize[1];
	m_pbFeatures = new bool[iBinnedSize];
	m_pbUsed = new bool[iBinnedSize];
	//-------------------------------
	int iNumPatches = piNumPatches[0] * piNumPatches[1];
	m_pfCenters = new float[iBinnedSize * 3];
	//---------------------------------------
	m_aiNumPatches[0] = piNumPatches[0];
	m_aiNumPatches[1] = piNumPatches[1];
	m_afPatSize[0] = m_aiBinnedSize[0] * 1.0f / m_aiNumPatches[0];
	m_afPatSize[1] = m_aiBinnedSize[1] * 1.0f / m_aiNumPatches[1];
	m_aiSeaRange[0] = (int)(m_afPatSize[0] * 0.5f - 0.5f);
	if(m_aiSeaRange[0] < 4) m_aiSeaRange[0] = 4;
	m_aiSeaRange[1] = m_aiBinnedSize[0] - m_aiSeaRange[0];
	m_aiSeaRange[2] = (int)(m_afPatSize[1] * 0.5f - 0.5f);
	if(m_aiSeaRange[2] < 4) m_aiSeaRange[2] = 4;
	m_aiSeaRange[3] = m_aiBinnedSize[1] - m_aiSeaRange[2];
	//----------------------------------------------------
	int aiSumRange[2] = {0, pXcfStackShift->m_iNumFrames};
	CAlignedSum::DoIt(EBuffer::xcf, pXcfStackShift, aiSumRange);
	//----------------------------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
	//-------------------------------------------------
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	cufftComplex* gCmpBinnedSum = pTmpBuffer->GetFrame(0, 0);
	//-------------------------------------------------------
	CStackBuffer* pXcfBuffer = pBufferPool->GetBuffer(EBuffer::xcf);
	int aiOutCmpSize[] = {m_aiBinnedSize[0] / 2 + 1, m_aiBinnedSize[1]};
	Util::GFourierResize2D aGFourierResize;
	aGFourierResize.DoIt(gCmpSum, pXcfBuffer->m_aiCmpSize,
	   gCmpBinnedSum, aiOutCmpSize, false, 0);
	//----------------------------------------
	Util::CCufft2D* pInverseFFT = pBufferPool->GetInverseFFT(0);
	pInverseFFT->CreateInversePlan(aiOutCmpSize, true);
	pInverseFFT->Inverse(gCmpBinnedSum, (cudaStream_t)0);
	float* gfBinnedSum = reinterpret_cast<float*>(gCmpBinnedSum);
	//-----------------------------------------------------------
	int aiOutPadSize[] = { aiOutCmpSize[0] * 2, aiOutCmpSize[1] };
	Util::GFindMinMax2D gFindMinMax2D;
	gFindMinMax2D.SetSize(aiOutPadSize, true);
	float fSumMin = gFindMinMax2D.DoMin(gfBinnedSum, true, 0);
	//--------------------------------------------------------
	Util::GPositivity2D gPositivity;
	gPositivity.AddVal(gfBinnedSum, aiOutPadSize, 1.0f - fSumMin);
	//------------------------------------------------------------
	int aiWinSize[] = {5, 5};
	GNormByStd2D gNormByStd;
	gNormByStd.DoIt(gfBinnedSum, aiOutPadSize, true, aiWinSize);	
	//----------------------------------------------------------
	Util::GCalcMoment2D gCalcMoment;
	gCalcMoment.SetSize(aiOutPadSize, true);
	float fMean = gCalcMoment.DoIt(gfBinnedSum, 1, true);
	float fStd = gCalcMoment.DoIt(gfBinnedSum, 2, true);
	fStd = fStd - fMean * fMean;
	if(fStd < 0) fStd = 0.0f;
	else fStd = (float)sqrtf(fStd);
	//-----------------------------
	float* pfBinnedSum = new float[iBinnedSize];
	int iBytes = sizeof(float) * m_aiBinnedSize[0];
	int iPadSize = aiOutCmpSize[0] * 2;
	for(int y=0; y<m_aiBinnedSize[1]; y++)
	{	cudaMemcpy(pfBinnedSum + y * m_aiBinnedSize[0],
		   gfBinnedSum + y * iPadSize, iBytes, cudaMemcpyDefault);
	}
	//----------------------------------------------------------------
	/*
	Util::CSaveTempMrc saveTempMrc;
	saveTempMrc.SetFile("/home/szheng/Temp/Test_Sum", ".mrc");
	saveTempMrc.DoIt(pfBinnedSum, 2, m_aiBinnedSize);
	*/
	//-----------------------------------------------
	float fMin = fMean - 4.0f * fStd;
	float fMax = fMean + 4.0f * fStd;
	//---------------------------------------
	double dMean = 0, dStd = 0;
	int iCount = 0;
	for(int i=0; i<iBinnedSize; i++)
	{	if(pfBinnedSum[i] < fMin) continue;
		else if(pfBinnedSum[i] > fMax) continue;
		dMean += pfBinnedSum[i];
		dStd += (pfBinnedSum[i] * pfBinnedSum[i]);
		iCount += 1;
	}
	dMean /= (iCount + 1e-30);
	dStd = dStd / (iCount + 1e-30) - dMean * dMean;
	if(dStd < 0) dStd = 0;
	else dStd = sqrtf(dStd);
	fMin = (float)(dMean - 4.0 * dStd);
	float fScale = (piNumPatches[0] + piNumPatches[1]) / 4.0f;
	fScale = fScale * fScale * 0.05f;
	fMax = (float)(dMean - 0.08f * dStd);
	//------------------------------------
	for(int i=0; i<iBinnedSize; i++)
	{	if(pfBinnedSum[i] < fMin) m_pbFeatures[i] = false;
		else if(pfBinnedSum[i] > fMax) m_pbFeatures[i] = false;
		else m_pbFeatures[i] = true;
	}
	delete[] pfBinnedSum;
	//-------------------
	memset(m_pbUsed, 0, sizeof(bool) * iBinnedSize);
	mFindCenters();
	/*
	printf("# Feature map\n");
	float fBinX = (pXcfBuffer->m_aiCmpSize[0] - 1) * 2.0f
	   / m_aiBinnedSize[0];
	float fBinY = pXcfBuffer->m_aiCmpSize[1] * 1.0f
	   / m_aiBinnedSize[1];
	for(int y=0; y<m_aiBinnedSize[1]; y+=5)
	{	int i = y * m_aiBinnedSize[0];
		float fY = (y + 0.5f) * fBinY;
		for(int x=0; x<m_aiBinnedSize[0]; x+=5)
		{	if(!m_pbFeatures[i+x]) continue;
			float fX = (x + 0.5f) * fBinX;
			printf("%6d  %9.2f  %9.2f\n", i+x, fX, fY);
		}
	}
	printf("\n");
	*/
}

void CDetectFeatures::GetCenter(int iPatch, int* piImgSize, float* pfCent)
{
	float fBinX = piImgSize[0] * 1.0f / m_aiBinnedSize[0];
	float fBinY = piImgSize[1] * 1.0f / m_aiBinnedSize[1];
	int i = 3 * iPatch;
	pfCent[0] = m_pfCenters[i] * fBinX;
	pfCent[1] = m_pfCenters[i+1] * fBinY;
}

void CDetectFeatures::FindNearest(float* pfLoc, 
	int* piImgSize, int* piPatSize, float* pfNewLoc)
{
	float fBinX = piImgSize[0] * 1.0f / m_aiBinnedSize[0];
	float fBinY = piImgSize[1] * 1.0f / m_aiBinnedSize[1];
	//----------------------------------------------------
	int iPatX = (int)(piPatSize[0] / fBinX);
	int iPatY = (int)(piPatSize[1] / fBinY);
	int iImgStartX = (int)(iPatX / 2);
	int iImgStartY = (int)(iPatY / 2);
	int iImgEndX = m_aiBinnedSize[0] - iImgStartX;
	int iImgEndY = m_aiBinnedSize[1] - iImgStartY;	
	//--------------------------------------------
	float fLocX = pfLoc[0] / fBinX;
	float fLocY = pfLoc[1] / fBinY;
	//-----------------------------
	int iWinX = m_aiBinnedSize[0] / 4;
	int iWinY = m_aiBinnedSize[1] / 4;
	int iStartX = (int)(fLocX - iWinX * 0.5f);
	if(iStartX < iImgStartX) iStartX = iImgStartX;
	else if((iStartX + iWinX) > iImgEndX) 
	{	iStartX = iImgEndX - iWinX;
	}
	int iStartY = (int)(fLocY - iWinY * 0.5f);
	if(iStartY < iImgStartY) iStartY = iImgStartY;
	else if((iStartY + iWinY) > iImgEndY)
	{	iStartY = iImgEndY - iWinY;
	}
	//------------------------------------------
	double dMin = 1e20;
	int iFound = -1;
	for(int j=0; j<iWinY; j++)
	{	int y = j + iStartY;
		for(int i=0; i<iWinX; i++)
		{	int x = i + iStartX;
			int k = y * m_aiBinnedSize[0] + x;
			if(!m_pbFeatures[k]) continue;
			else if(m_pbUsed[k]) continue;
			//----------------------------
			float fDx = x + 0.5f - fLocX;
			float fDy = y + 0.5f - fLocY;
			double dR = sqrtf(fDx * fDx + fDy * fDy);
			if(dR < dMin)
			{	pfNewLoc[0] = x;
				pfNewLoc[1] = y;
				dMin = dR;
				iFound = k;
			}
		}
	}
	//---------------------------------
	iWinX = iPatX * 3 / 4;
	iWinY = iPatY * 3 / 4;
	iStartX = (int)(pfNewLoc[0] + 0.01f) - iWinX / 2;
	if(iStartX < 0) iStartX = 0;
	else if((iStartX + iWinX) >= m_aiBinnedSize[0])
	{	iStartX = m_aiBinnedSize[0] - iWinX;
	}
	iStartY = (int)(pfNewLoc[1] + 0.01f) - iWinY / 2;
	if(iStartY < 0) iStartY = 0;
	else if((iStartY + iWinY) >= m_aiBinnedSize[1])
	{	iStartY = m_aiBinnedSize[1] - iWinY;
	}
	for(int j=0; j<iWinY; j++)
	{	for(int i=0; i<iWinX; i++)
		{	m_pbUsed[(j+iStartY) * m_aiBinnedSize[0]
			   + (i+iStartX)] = true;
		}
	}
	//---------------------------------------
	if(iFound < 0 || dMin < 3)
	{	pfNewLoc[0] = pfLoc[0];
		pfNewLoc[1] = pfLoc[1];
	}
	else
	{	pfNewLoc[0] = (pfNewLoc[0] + 0.5f) * fBinX;
		pfNewLoc[1] = (pfNewLoc[1] + 0.5f) * fBinY;
		m_pbUsed[iFound] = true;
	}
}

void CDetectFeatures::mFindCenters(void)
{
	int iNumPatches = m_aiNumPatches[0] * m_aiNumPatches[1];
	memset(m_pfCenters, 0, sizeof(float) * 3 * iNumPatches);
	//------------------------------------------------------
	int iNoFeatures = 0;
	for(int i=0; i<iNumPatches; i++)
	{	int x = i % m_aiNumPatches[0];
		int y = i / m_aiNumPatches[0];
		float fCentX = (x + 0.5f) * m_afPatSize[0];
		float fCentY = (y + 0.5f) * m_afPatSize[1];
		bool bHasFeature = mCheckFeature(fCentX, fCentY);
		//-----------------------------------------------
		int j = 3 * i;
		m_pfCenters[j] = fCentX;
		m_pfCenters[j+1] = fCentY;
		if(bHasFeature) 
		{	m_pfCenters[j+2] = 1.0f;
			mSetUsed(fCentX, fCentY);
		}
		else 
		{	m_pfCenters[j+2] = -1.0f;
			iNoFeatures += 1;
		}
	}
	if(iNoFeatures = 0) return;
	//-------------------------
	for(int i=0; i<iNumPatches; i++)
	{	float* pfCenter = m_pfCenters + 3 * i;
		if(pfCenter[2] > 0) continue;
		mFindCenter(pfCenter);
	}
	/*
	for(int i=0; i<iNumPatches; i++)
	{	int j = 3 * i;
		if(m_pfCenters[j+2] < 0) continue;
		printf("%3d %8.2f %8.2f\n", i+1,
		   m_pfCenters[j], m_pfCenters[j+1]);
	}
	*/
}

bool CDetectFeatures::mCheckFeature(float fCentX, float fCentY)
{
	int iCentX = (int)fCentX;
	int iCentY = (int)fCentY;
	if(iCentX < m_aiSeaRange[0]) return false;
	else if(iCentX > m_aiSeaRange[1]) return false;
	//---------------------------------------------
	if(iCentY < m_aiSeaRange[2]) return false;
	else if(iCentY > m_aiSeaRange[3]) return false;
	//---------------------------------------------
	int i = iCentY * m_aiBinnedSize[0] + iCentX;
	return m_pbFeatures[i];
}

void CDetectFeatures::mFindCenter(float* pfCenter)
{
	int iWinX = m_aiBinnedSize[0] / 3;
	int iWinY = m_aiBinnedSize[1] / 3;
	int iStartX = (int)(pfCenter[0] - iWinX * 0.5f + 0.5f);
	int iStartY = (int)(pfCenter[1] - iWinY * 0.5f + 0.5f);
	iStartX = mCheckRange(iStartX, iWinX, &m_aiSeaRange[0]);
	iStartY = mCheckRange(iStartY, iWinY, &m_aiSeaRange[2]);
	//------------------------------------------------------
	float fMinR = (float)1e20;
	float afCent[2] = {-1.0f};
	int iOffset = iStartY * m_aiBinnedSize[0] + iStartX;
	bool* pbFeatures = &m_pbFeatures[iOffset];
	bool* pbUsed = &m_pbUsed[iOffset];
	//--------------------------------
	float fX, fY, fDx, fDy, fR;
	for(int y=0; y<iWinY; y++)
	{	int i = y * m_aiBinnedSize[0];
		for(int x=0; x<iWinX; x++)
		{	int j = i + x;
			if(pbUsed[j]) continue;
			if(!pbFeatures[j]) continue;
			//----------------------------
			fX = x + iStartX + 0.5f;
			fY = y + iStartY + 0.5f;
			fDx = fX - pfCenter[0];
			fDy = fY - pfCenter[1]; 
			fR = (float)sqrtf(fDx * fDx + fDy * fDy);
			if(fR >= fMinR) continue;
			//-----------------------
			fMinR = fR;
			afCent[0] = fX;
			afCent[1] = fY;
		}
	}
	//-----------------------------
	if(afCent[0] > 0)
	{	pfCenter[0] = afCent[0];
		pfCenter[1] = afCent[1];
	}
	mSetUsed(pfCenter[0], pfCenter[1]);
}


void CDetectFeatures::mSetUsed(float fCentX, float fCentY)
{
	int iWinX = (int)(m_afPatSize[0] * 0.8f);
	int iWinY = (int)(m_afPatSize[1] * 0.8f);
	int iStartX = (int)(fCentX - iWinX / 2);
	int iStartY = (int)(fCentY - iWinY / 2);
	iStartX = mCheckRange(iStartX, iWinX, &m_aiSeaRange[0]);
	iStartY = mCheckRange(iStartY, iWinY, &m_aiSeaRange[2]);
	//------------------------------------------------------
	bool* pbUsed = &m_pbUsed[iStartY * m_aiBinnedSize[0] + iStartX];
	for(int y=0; y<iWinY; y++)
	{	int i = y * m_aiBinnedSize[0];
		for(int x=0; x<iWinX; x++)
		{	pbUsed[i + x] = true;
		}
	}
}

int CDetectFeatures::mCheckRange(int iStart, int iSize, int* piRange)
{
	if(iStart < piRange[0]) return piRange[0];
	if((iStart + iSize) > piRange[1]) return (piRange[1] - iSize);
	return iStart;
}
