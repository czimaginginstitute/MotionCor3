#include "CCorrectInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../FindCtf/CFindCtfInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2;
using namespace MotionCor2::Correct;

Align::CStackShift* CCorrectFullShift::m_pFullShift = 0L;
DU::CDataPackage* CCorrectFullShift::m_pPackage = 0L;
int CCorrectFullShift::m_aiInCmpSize[2] = {0};
int CCorrectFullShift::m_aiInPadSize[2] = {0};
int CCorrectFullShift::m_aiOutCmpSize[2] = {0};
int CCorrectFullShift::m_aiOutPadSize[2] = {0};

CCorrectFullShift::CCorrectFullShift(void)
{
	m_pGWeightFrame = 0L;
	m_pForwardFFT = 0L;
	m_pInverseFFT = 0L;
}

CCorrectFullShift::~CCorrectFullShift(void)
{
	if(m_pGWeightFrame != 0L) delete m_pGWeightFrame;
	m_pGWeightFrame = 0L;
	m_pForwardFFT = 0L;
	m_pInverseFFT = 0L;
}

void CCorrectFullShift::DoIt
(	Align::CStackShift* pStackShift,
	DU::CDataPackage* pPackage
)
{	m_pFullShift = pStackShift;
	m_pPackage = pPackage;
	m_aiOutCmpSize[0] = m_pPackage->m_pAlnSums->m_aiStkSize[0] / 2 + 1;
	m_aiOutCmpSize[1] = m_pPackage->m_pAlnSums->m_aiStkSize[1];
	m_aiOutPadSize[0] = m_aiOutCmpSize[0] * 2;
	m_aiOutPadSize[1] = m_aiOutCmpSize[1];
	//------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	m_aiInCmpSize[0] = pFrmBuffer->m_aiCmpSize[0];
	m_aiInCmpSize[1] = pFrmBuffer->m_aiCmpSize[1];
	m_aiInPadSize[0] = m_aiInCmpSize[0] * 2;
	m_aiInPadSize[1] = m_aiInCmpSize[1];
	//----------------------------------	
	int iNumGpus = pBufferPool->m_iNumGpus;
	CCorrectFullShift* pCorrShifts = new CCorrectFullShift[iNumGpus];
	for(int i=0; i<iNumGpus; i++)
	{	pCorrShifts[i].Run(i);
	}
	for(int i=0; i<iNumGpus; i++)
	{	pCorrShifts[i].Wait();
	}
	delete[] pCorrShifts;
	//-------------------
	mSumPartialSums();
	mCorrectMag();
	mUnpadSums();
}

void CCorrectFullShift::mSumPartialSums(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(0);
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	if(pBufferPool->m_iNumGpus <= 1) return;
	//--------------------------------------
	cufftComplex* gCmpBuf = pTmpBuffer->GetFrame(0, 0);
	size_t tBytes = pFrmBuffer->m_tFmBytes;
	Util::GAddFrames addFrames;
	//-------------------------
	for(int s=0; s<pBufferPool->m_iNumSums; s++)
	{	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, s);
		for(int g=1; g<pBufferPool->m_iNumGpus; g++)
		{	cufftComplex* gPartSum = pSumBuffer->GetFrame(g, s);
			cudaMemcpy(gCmpBuf, gPartSum, tBytes, 
				cudaMemcpyDefault);
			addFrames.DoIt(gCmpSum, 1.0f, gCmpBuf, 1.0f, 
				gCmpSum, pFrmBuffer->m_aiCmpSize);
		}
	}
}

void CCorrectFullShift::mCorrectMag(void)
{
        Align::CAlignParam* pAlignParam = Align::CAlignParam::GetInstance();
        float afStretch[] = {1.0f, 0.0f};
        pAlignParam->GetMagStretch(afStretch);
        if(fabs(afStretch[0] - 1) < 1e-5) return;
	//---------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	//--------------------------------------------------------------
	Util::CCufft2D* pInverseFFT = pBufferPool->GetInverseFFT(0);
        pInverseFFT->CreateInversePlan(pFrmBuffer->m_aiCmpSize, true);
	//---------------------------------------------------------
	GStretch aGStretch;
        aGStretch.Setup(afStretch[0], afStretch[1]);
	//------------------------------------------
	Util::CCufft2D* pForwardFFT = pBufferPool->GetForwardFFT(0);
	int aiPadSize[] = {0, pFrmBuffer->m_aiCmpSize[1]};
        aiPadSize[0] = pFrmBuffer->m_aiCmpSize[0] * 2;
	pForwardFFT->CreateForwardPlan(aiPadSize, true);
	//----------------------------------------------
        for(int i=0; i<pBufferPool->m_iNumSums; i++)
        {       cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, i);
		cufftComplex* gCmpTmp = pTmpBuffer->GetFrame(0, 0);
		pInverseFFT->Inverse(gCmpSum, (float*)gCmpTmp);
		//---------------------------------------------
		float* gfPadFrm = reinterpret_cast<float*>(gCmpTmp);
                bool bPadded = true;
                aGStretch.DoIt(gfPadFrm, bPadded, aiPadSize, (float*)gCmpSum);
		pForwardFFT->Forward((float*)gCmpSum, true);
        }
}

void CCorrectFullShift::mEstimateCtf(float* gfImg, int* piImgSize)
{
	if(!FindCtf::CFindCtfMain::m_bEstCtf) return;
	//-----------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	float* gfTmpBuf = (float*)pTmpBuffer->GetFrame(0, 0);
	bool bPadded = true;
	//------------------------------------------------------------
	// gfPadSum is used as GPU buffer in CFindCtfMain. It is safe
	// since the first frame in SumBuffer has been done with all
	// needed processing.
	//------------------------------------------------------------
	FindCtf::CFindCtfMain findCtfMain;
	findCtfMain.DoIt(gfImg, piImgSize, !bPadded, gfTmpBuf, m_pPackage);
}

void CCorrectFullShift::mUnpadSums(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(0);
	//------------------------
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	//--------------------------------------------------------------
	cufftComplex* gCmpBuf = pTmpBuffer->GetFrame(0, 0);
	Util::GFourierResize2D fftResize;
	Util::GPad2D pad2D;
	//-----------------
	Util::CCufft2D* pCufft2D = pBufferPool->GetInverseFFT(0);
	pCufft2D->CreateInversePlan(m_aiOutCmpSize, true);
	//------------------------------------------------
	Util::GPositivity2D aPositivity;
	//------------------------------
	DU::CMrcStack* pAlnSums = m_pPackage->m_pAlnSums;
	for(int i=0; i<pBufferPool->m_iNumSums; i++)
	{	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, i);
		fftResize.DoIt(gCmpSum, pSumBuffer->m_aiCmpSize,
		   gCmpBuf, m_aiOutCmpSize, false);
		//---------------------------------
		pCufft2D->Inverse(gCmpBuf);
		float* gfPadBuf = reinterpret_cast<float*>(gCmpBuf);
		//--------------------------------------------------
		aPositivity.DoIt(gfPadBuf, m_aiOutPadSize);
		//-----------------------------------------
		float* gfImg = reinterpret_cast<float*>(gCmpSum);
		pad2D.Unpad(gfPadBuf, m_aiOutPadSize, gfImg);
		//----------------
		aPositivity.DoIt(gfImg, pAlnSums->m_aiStkSize);
		//----------------
		if(i == 0) mEstimateCtf(gfImg, pAlnSums->m_aiStkSize);
		//----------------
		void* pvImg = pAlnSums->GetFrame(i);
		cudaMemcpy(pvImg, gfImg, pAlnSums->m_tFmBytes,
		   cudaMemcpyDefault);
	}
}

void CCorrectFullShift::Run(int iNthGpu)
{
	m_iNthGpu = iNthGpu;
	mInit();
	mCorrectGpuFrames();
	mCorrectCpuFrames();
	cudaDeviceSynchronize();
}

void CCorrectFullShift::Wait(void)
{
	m_pFrmBuffer->SetDevice(m_iNthGpu);
	cudaDeviceSynchronize();
	cudaStreamDestroy(m_aStreams[0]);
	cudaStreamDestroy(m_aStreams[1]);
	//-------------------------------
	if(m_pGWeightFrame != 0L) delete m_pGWeightFrame;
	m_pGWeightFrame = 0L;
}

void CCorrectFullShift::mInit(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(m_iNthGpu);
	//--------------------------------
	m_pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	m_pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	m_pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	//--------------------------------------------------
	cudaStreamCreate(&m_aStreams[0]);
	cudaStreamCreate(&m_aStreams[1]);
	//-------------------------------
	mCheckDoseWeight();
	mCheckFrameCrop();
	//----------------
	size_t tBytes = m_pFrmBuffer->m_tFmBytes;
	for(int i=0; i<pBufferPool->m_iNumSums; i++)
	{	cufftComplex* gCmpSum = m_pSumBuffer->GetFrame(m_iNthGpu, i);
		cudaMemsetAsync(gCmpSum, 0, tBytes, m_aStreams[0]);
	}
}

void CCorrectFullShift::mCorrectGpuFrames(void)
{
	int iStartFrm = m_pFrmBuffer->GetStartFrame(m_iNthGpu);
        int iNumFrames = m_pFrmBuffer->GetNumFrames(m_iNthGpu);
        for(int i=0; i<iNumFrames; i++)
        {       if(!m_pFrmBuffer->IsGpuFrame(m_iNthGpu, i)) continue;
		//---------------------------------------------------
                m_iAbsFrm = iStartFrm + i;
                cufftComplex* gCmpFrm = m_pFrmBuffer->GetFrame(m_iNthGpu, i);
		mAlignFrame(gCmpFrm);
                mGenSums(gCmpFrm);
        }
}

void CCorrectFullShift::mCorrectCpuFrames(void)
{
	int iCount = 0;
	size_t tBytes = m_pFrmBuffer->m_tFmBytes;
	int iStartFrm = m_pFrmBuffer->GetStartFrame(m_iNthGpu);
	int iNumFrames = m_pFrmBuffer->GetNumFrames(m_iNthGpu);
	cufftComplex *pCmpFrm, *gCmpBuf;
	//------------------------------
	for(int i=0; i<iNumFrames; i++)
	{	if(m_pFrmBuffer->IsGpuFrame(m_iNthGpu, i)) continue;
		//--------------------------------------------------
		m_iAbsFrm = iStartFrm + i;
		int iStream = iCount % 2;
		gCmpBuf = m_pTmpBuffer->GetFrame(m_iNthGpu, iStream);
		pCmpFrm = m_pFrmBuffer->GetFrame(m_iNthGpu, i);
		//--------------------------------------------------
		if(iStream == 1 && iCount > 1) 
			cudaStreamSynchronize(m_aStreams[0]);
		cudaMemcpyAsync(gCmpBuf, pCmpFrm, tBytes, 
			cudaMemcpyDefault, m_aStreams[iStream]);
		if(iStream == 1) cudaStreamSynchronize(m_aStreams[1]);
		//----------------------------------------------------
		mAlignFrame(gCmpBuf);
		mGenSums(gCmpBuf);
		iCount += 1;
	}
}

void CCorrectFullShift::mGenSums(cufftComplex* gCmpFrm)
{	
	mMotionDecon(gCmpFrm);
	//-----------------
	int iNextSum = 0;
	mSum(gCmpFrm, iNextSum);
	iNextSum += 1;
	//-----------------
	mCropFrame(gCmpFrm);
	//------------------
	DU::CFmIntParam* pFmIntParam = m_pPackage->m_pFmIntParam;
	if(m_pGWeightFrame != 0L)
	{	mDoseWeight(gCmpFrm);
		mSum(gCmpFrm, iNextSum);
		iNextSum += 1;
		//----------------
		if(pFmIntParam->bDWSelectedSum())
		{	if(pFmIntParam->bInSumRange(m_iAbsFrm)) 
			{	mSum(gCmpFrm, iNextSum);
			}
			iNextSum += 1;
		}
	}
	//-----------------
	Align::CAlignParam* pAlnParam = Align::CAlignParam::GetInstance();
	if(pAlnParam->SplitSum())
	{	if(m_iAbsFrm % 2 == 0) mSum(gCmpFrm, iNextSum);
		else mSum(gCmpFrm, iNextSum+1);
	}
}

void CCorrectFullShift::mAlignFrame(cufftComplex* gCmpFrm)
{	
	float afShift[2] = {0};
	m_pFullShift->GetShift(m_iAbsFrm, afShift, -1.0f);
	Util::GPhaseShift2D phaseShift2D;
	phaseShift2D.DoIt(gCmpFrm, m_pFrmBuffer->m_aiCmpSize,
	   afShift, m_aStreams[0]);
}

void CCorrectFullShift::mMotionDecon(cufftComplex* gCmpFrm)
{	
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iInFmMotion == 0) return;
	//------------------------------------
	int* piCmpSize = m_pFrmBuffer->m_aiCmpSize;
	m_aInFrameMotion.SetFullShift(m_pFullShift);
	m_aInFrameMotion.DoFullMotion(m_iAbsFrm, gCmpFrm, 
		piCmpSize, m_aStreams[0]);
}

void CCorrectFullShift::mDoseWeight(cufftComplex* gCmpFrm)
{	
	if(m_pGWeightFrame == 0L) return;
	m_pGWeightFrame->DoIt(gCmpFrm, m_iAbsFrm, m_aStreams[0]);
}

void CCorrectFullShift::mSum(cufftComplex* gCmpFrm, int iNthSum)
{      
	cufftComplex* gCmpSum = m_pSumBuffer->GetFrame(m_iNthGpu, iNthSum);
	Util::GAddFrames addFrames;
	addFrames.DoIt(gCmpFrm, 1.0f, gCmpSum, 1.0f, gCmpSum,
		m_pFrmBuffer->m_aiCmpSize, m_aStreams[0]);
}

void CCorrectFullShift::mCropFrame(cufftComplex* gCmpFrm)
{
	if(m_pInverseFFT == 0L) return;
	//-----------------
	cufftComplex* gCmpBuf = m_pTmpBuffer->GetFrame(m_iNthGpu, 3);
	Util::GFourierResize2D fftResize;
	fftResize.DoIt(gCmpFrm, m_aiInCmpSize, gCmpBuf, 
	   m_aiOutCmpSize, false, m_aStreams[0]);
	//-----------------
	m_pInverseFFT->Inverse(gCmpBuf, m_aStreams[0]);
	float* gfPadBuf = reinterpret_cast<float*>(gCmpBuf);
	//-----------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	float* pfPinned = (float*)pBufferPool->GetPinnedBuf(m_iNthGpu);
	Util::GPad2D aGPad2D;
	aGPad2D.Unpad(gfPadBuf, m_aiOutPadSize, pfPinned, m_aStreams[0]);
	//-----------------
	cudaStreamSynchronize(m_aStreams[0]);
	void* pvFrm = m_pPackage->m_pAlnStack->GetFrame(m_iAbsFrm);
	memcpy(pvFrm, pfPinned, m_pPackage->m_pAlnStack->m_tFmBytes);
}

void CCorrectFullShift::mCheckDoseWeight(void)
{
	DU::CFmIntParam* pFmIntParam = m_pPackage->m_pFmIntParam; 
	if(!pFmIntParam->bDoseWeight()) 
	{	if(m_pGWeightFrame != 0L) delete m_pGWeightFrame;
		m_pGWeightFrame = 0L;
		return;
	}
	//-----------------
	CInput* pInput = CInput::GetInstance();	
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	m_pFrmBuffer->SetDevice(m_iNthGpu);
	if(m_pGWeightFrame == 0L) m_pGWeightFrame = new GWeightFrame;
	//-----------------
	cufftComplex* gCmpBuf = m_pTmpBuffer->GetFrame(m_iNthGpu, 2);
	float* gfWeightBuf = reinterpret_cast<float*>(gCmpBuf);
	m_pGWeightFrame->BuildWeight(pInput->m_fPixelSize, pInput->m_iKv,
	  pFmIntParam->m_pfAccFmDose, pBufferPool->m_aiStkSize, 
	  gfWeightBuf, m_aStreams[0]);
};

void CCorrectFullShift::mCheckFrameCrop(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_aiOutStack[0] == 0) return;
	//-----------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	m_pInverseFFT = pBufferPool->GetInverseFFT(m_iNthGpu);
	m_pInverseFFT->CreateInversePlan(m_aiOutCmpSize, true);
}

