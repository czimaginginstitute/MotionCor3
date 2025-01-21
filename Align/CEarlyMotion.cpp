#include "CAlignInc.h"
#include "../Correct/CCorrectInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

CEarlyMotion::CEarlyMotion(void)
{
	m_fBFactor = 500.0f;
	m_gCmpRef = 0L;
}

CEarlyMotion::~CEarlyMotion(void)
{
	int iGpuID = -1;
	cudaGetDevice(&iGpuID);
	//---------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(0);
	if(m_gCmpRef != 0L) cudaFree(m_gCmpRef);
	//--------------------------------------
	if(iGpuID >= 0) cudaSetDevice(iGpuID);
}

void CEarlyMotion::Setup(EBuffer eBuffer, float fBFactor)
{	
	m_eBuffer = eBuffer;
	m_fBFactor = fBFactor;
	//--------------------
	CInput* pInput = CInput::GetInstance();	
	if(eBuffer == EBuffer::xcf && pInput->m_aiGroup[0] == 1) return;
	if(eBuffer == EBuffer::pat && pInput->m_aiGroup[1] == 1) return;
	//--------------------------------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pStackBuffer = pBufferPool->GetBuffer(m_eBuffer);
	memcpy(m_aiCmpSize, pStackBuffer->m_aiCmpSize, sizeof(int) * 2);
	//--------------------------------------------------------------
	int iGpuID = -1;
	cudaGetDevice(&iGpuID);
	pBufferPool->SetDevice(0);
	//------------------------
	if(m_gCmpRef != 0L) cudaFree(m_gCmpRef);
	m_gCmpRef = Util::GGetCmpBuf(m_aiCmpSize, false);
	//-----------------------------------------------
	m_aiSeaSize[0] = 16; m_aiSeaSize[1] = 16;
	m_aGCorrelateSum.SetSubtract(false);
	m_aGCorrelateSum.SetFilter(m_fBFactor, false);
	m_aGCorrelateSum.SetSize(m_aiCmpSize, m_aiSeaSize);
	//-------------------------------------------------
	m_pInverseFFT = pBufferPool->GetInverseFFT(0);
	m_pInverseFFT->CreateInversePlan(m_aiCmpSize, true);
	//--------------------------------------------------
	if(iGpuID >= 0) cudaSetDevice(iGpuID);
}

void CEarlyMotion::DoIt
(	DU::CDataPackage* pPackage,
	CStackShift* pStackShift
)
{	CInput* pInput = CInput::GetInstance();
	if(m_eBuffer == EBuffer::xcf && pInput->m_aiGroup[0] == 1) return;
	if(m_eBuffer == EBuffer::pat && pInput->m_aiGroup[1] == 1) return;
	//--------------------------
	m_pPackage = pPackage;
	m_pStackShift = pStackShift; 
	//--------------------------
	bool bPatch = (m_eBuffer == EBuffer::pat);
	DU::CFmGroupParam* pFmGroupParam = &(pPackage->m_pFmGroupParams[1]);
	if(pFmGroupParam->m_iNumGroups < 2) return;
	//---------------------------
	m_aiNodeFm[0] = 0;
	m_aiNodeFm[1] = pFmGroupParam->GetGroupStart(0) +
	   pFmGroupParam->GetGroupSize(0) / 2;
	m_aiNodeFm[2] = pFmGroupParam->GetGroupStart(1) +
	   pFmGroupParam->GetGroupSize(1) / 2;
	//---------------------------
	DU::CFmIntParam* pFmIntParam = pPackage->m_pFmIntParam;
	m_afCent[0] = pFmIntParam->m_pfIntFmCents[m_aiNodeFm[0]];
	m_afCent[1] = pFmIntParam->m_pfIntFmCents[m_aiNodeFm[1]];
	m_afCent[2] = pFmIntParam->m_pfIntFmCents[m_aiNodeFm[2]];
	//---------------------------
	m_aiSumRange[0] = pFmGroupParam->GetGroupSize(0);
	m_aiSumRange[1] = m_pStackShift->m_iNumFrames * 2 / 3;
	CAlignedSum::DoIt(m_eBuffer, m_pStackShift, m_aiSumRange);
	//---------------------------
	int iGpuID = -1;
	cudaGetDevice(&iGpuID);
	//------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(0);
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
	size_t tBytes = sizeof(cufftComplex) * m_aiCmpSize[0] * m_aiCmpSize[1];
	cudaMemcpy(m_gCmpRef, gCmpSum, tBytes, cudaMemcpyDefault);
	//---------------------------------------------------------------------
	mDoIt();
	if(iGpuID >= 0) cudaSetDevice(iGpuID);
}

void CEarlyMotion::mDoIt(void)
{
	float afRefPeak[3] = {0.0f};
	mCorrelate(0, m_pStackShift);	
	mFindPeak(0, afRefPeak);
	//---------------------------
	m_iNumSteps = 40;
	m_fStepSize = 0.1f;
	CStackShift* pStackShift = m_pStackShift->GetCopy();
	float fIncX = mIterate(pStackShift, 0);
	float fIncY = mIterate(pStackShift, 1);
	//printf("Inc: %8.2f  %8.2f\n", fIncX, fIncY);
	//-------------------------------------------------
	float afOptPeak[3] = {0.0f};
	mCorrelate(0, pStackShift);
	mFindPeak(0, afOptPeak);
	if(afRefPeak[2] < afOptPeak[2]) 
	{	m_pStackShift->SetShift(pStackShift);
	}
	//-------------------------------------------
	/*printf("Ref: %8.2f  %8.2f  %10.7e\n",
	   afRefPeak[0], afRefPeak[1], afRefPeak[2]); 
	printf("Opt: %8.2f  %8.2f  %10.7e\n",
	   afOptPeak[0], afOptPeak[1], afOptPeak[2]);
	*/
	delete pStackShift;
}

float CEarlyMotion::mIterate(CStackShift* pStackShift, int iAxis)
{
	float* pfIncs = new float[m_iNumSteps];
	float* pfPeaks = new float[m_iNumSteps * 3];
	float afShifts[3] = {0.0f}, afCoeffs[3] = {0.0f};
	mGetNodeShifts(pStackShift, iAxis, afShifts);	
	//-------------------------------------------
	float* pfCoeffXs = (iAxis == 0) ? &afCoeffs[0] : 0L;
	float* pfCoeffYs = (iAxis == 1) ? &afCoeffs[0] : 0L;
	//--------------------------------------------------	
	for(int i=0; i<m_iNumSteps; i++)
        {       pfIncs[i] = 1.0f + (i - 0.5f * m_iNumSteps) * m_fStepSize;
                mCalcCoeff(pfIncs[i], afShifts, &afCoeffs[0]);
                mCalcShift(pfCoeffXs, pfCoeffYs, pStackShift);
                mCorrelate(i, pStackShift);
        }
        mFindPeaks(pfPeaks);
	//------------------
	int iOptimal = -1;
	float fPeak = (float)-1e30;
	for(int i=0; i<m_iNumSteps; i++)
	{	float* pfPeak = pfPeaks + i * 3;
		//printf("%3d  %3d  %8.2f  %8.2f  %10.7e\n", i, iOptimal,
		//   pfPeak[0], pfPeak[1], pfPeak[2]);
		if(fPeak >= pfPeak[2]) continue;
		fPeak = pfPeak[2];
		iOptimal = i;
	}
	//printf("\n");
	//---------------------------------------------
	float fInc = pfIncs[iOptimal];
	mCalcCoeff(fInc, afShifts, &afCoeffs[0]);
	mCalcShift(pfCoeffXs, pfCoeffYs, pStackShift);
	//--------------------------------------------
	float* pfExtraShift = &pfPeaks[iOptimal * 3];
	for(int i=0; i<m_aiSumRange[0]; i++)
	{	pStackShift->AddShift(i, pfExtraShift);
	}
	delete[] pfIncs;
	delete[] pfPeaks;
	return fInc;
}

void CEarlyMotion::mGetNodeShifts
(	CStackShift* pStackShift, 
	int iAxis, float* pfShift
)
{	float afShifts[6] = {0.0f};
	pStackShift->GetShift(m_aiNodeFm[0], &afShifts[0]);
	pStackShift->GetShift(m_aiNodeFm[1], &afShifts[2]);
	pStackShift->GetShift(m_aiNodeFm[2], &afShifts[4]);
	//-----------------
	pfShift[0] = afShifts[0 + iAxis];
	pfShift[1] = afShifts[2 + iAxis];
	pfShift[2] = afShifts[4 + iAxis];
}
	
void CEarlyMotion::mCalcCoeff(float fGain, float* pfShift, float* pfCoeff)
{
	pfCoeff[0] = pfShift[0] + fGain;
	//-------------------------------------------------------------
	float x1_2 = m_afCent[1] * m_afCent[1];
	float x2_2 = m_afCent[2] * m_afCent[2];
	float fDelta = m_afCent[1] * x2_2 - m_afCent[2] * x1_2;
	//-----------------------------------------------------
	pfCoeff[1] = ((pfShift[1] - pfCoeff[0]) * x2_2 - 
	   (pfShift[2] - pfCoeff[0]) * x1_2) / fDelta;
	pfCoeff[2] = ((pfShift[2] - pfCoeff[0]) * m_afCent[1] -
	   (pfShift[1] - pfCoeff[0]) * m_afCent[2]) / fDelta;
}

void CEarlyMotion::mCalcShift
(	float* pfCoeffXs,
	float* pfCoeffYs,
	CStackShift* pStackShift
)
{	float afShift[2] = {0.0f};
	DU::CFmIntParam* pFmIntParam = m_pPackage->m_pFmIntParam;
	for(int i=0; i<m_aiSumRange[0]; i++)
	{	pStackShift->GetShift(i, afShift);
		float fX = pFmIntParam->m_pfIntFmCents[i];
		float fX2 = fX * fX;
		if(pfCoeffXs != 0L)
		{	afShift[0] = pfCoeffXs[0] + pfCoeffXs[1] * fX 
			   + pfCoeffXs[2] * fX2;
		}
		if(pfCoeffYs != 0L)
		{	afShift[1] = pfCoeffYs[0] + pfCoeffYs[1] * fX
			   + pfCoeffYs[2] * fX2;
		}
		pStackShift->SetShift(i, afShift);
	}
}

void CEarlyMotion::mCorrelate(int iStep, CStackShift* pStackShift)
{
	int aiSumRange[2] = {0, 1};
	aiSumRange[1] = m_aiSumRange[0] - 1;
	CAlignedSum::DoIt(m_eBuffer, pStackShift, aiSumRange);
	//----------------------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
	//-------------------------------------------------
	int iSeaSize = m_aiSeaSize[0] * m_aiSeaSize[1];
	float* pfPinnedBuf = (float*)pBufferPool->GetPinnedBuf(0);
	float* pfXcfBuf = pfPinnedBuf + iSeaSize * iStep;
	//-----------------------------------------------
	m_aGCorrelateSum.DoIt(m_gCmpRef, gCmpSum, pfXcfBuf, m_pInverseFFT, 0);	
}

void CEarlyMotion::mFindPeaks(float* pfPeaks)
{
	cudaStreamSynchronize((cudaStream_t)0);
	//-------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	float* pfPinnedBuf = (float*)pBufferPool->GetPinnedBuf(0);
	int iSeaSize = m_aiSeaSize[0] * m_aiSeaSize[1];
	//--------------------------------------------------------
	CPeak2D peak2D;
	for(int i=0; i<m_iNumSteps; i++)
	{	float* pfXcfBuf = pfPinnedBuf + i * iSeaSize;
		peak2D.DoIt(pfXcfBuf, m_aiSeaSize);
		float* pfPeak = pfPeaks + i * 3;
		memcpy(pfPeak, peak2D.m_afShift, sizeof(float) * 2);
		pfPeak[2] = peak2D.m_fPeak;
	}
}

void CEarlyMotion::mFindPeak(int iPeak, float* pfPeak)
{
	cudaStreamSynchronize((cudaStream_t)0);
	//-------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
        float* pfPinnedBuf = (float*)pBufferPool->GetPinnedBuf(0);
        int iSeaSize = m_aiSeaSize[0] * m_aiSeaSize[1];
	float* pfXcfImg = pfPinnedBuf + iPeak * iSeaSize;
	//-----------------------------------------------
	CPeak2D peak2D;
	peak2D.DoIt(pfXcfImg, m_aiSeaSize);
	memcpy(pfPeak, peak2D.m_afShift, sizeof(float) * 2);
	pfPeak[2] = peak2D.m_fPeak;
}
