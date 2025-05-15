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

static void mCalcSumMean(EBuffer eBuffer, bool bTransform)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pStkBuffer = pBufferPool->GetBuffer(eBuffer);
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	//--------------------------------------------------------------
	int* piCmpSize = pStkBuffer->m_aiCmpSize;
	size_t tBytes = pStkBuffer->m_tFmBytes;
	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
	cufftComplex* gCmpBuf = pTmpBuffer->GetFrame(0, 0);
	cudaMemcpy(gCmpBuf, gCmpSum, tBytes, cudaMemcpyDefault);
	//------------------------------------------------------
	if(bTransform)
	{	Util::CCufft2D cufft2D;
		cufft2D.CreateInversePlan(piCmpSize, true);
		cufft2D.Inverse(gCmpBuf);
	}
	//-------------------------------
	int aiPadSize[] = {piCmpSize[0] * 2, piCmpSize[1]};
	int iSizeX = (piCmpSize[0] - 1) * 2;
	//----------------------------------
	int iPadSize = aiPadSize[0] * aiPadSize[1];
	float* pfPadBuf = new float[iPadSize];
	cudaMemcpy(pfPadBuf, gCmpBuf, tBytes, cudaMemcpyDefault);
	//-------------------------------------------------------
	double dMean = 0;
	for(int y=0; y<aiPadSize[1]; y++)
	{	int i = y * aiPadSize[0];
		for(int x=0; x<iSizeX; x++)
		{	dMean += pfPadBuf[i+x];
		}
	}
	dMean /= (iSizeX * aiPadSize[1]);
	delete[] pfPadBuf;
	printf("dMean = %f\n", dMean);
}

static void mSumStack(EBuffer eBuffer, bool bTransform)
{
	MrcUtil::CSumFFTStack sumStack;
	sumStack.DoIt(eBuffer, false);
	mCalcSumMean(eBuffer, bTransform);
}

CFullAlign::CFullAlign(void)
{
}

CFullAlign::~CFullAlign(void)
{
}

void CFullAlign::Align(DU::CDataPackage* pPackage)
{
	nvtxRangePushA ("AlignBase Clean+DoIt");
	CAlignBase::Clean();
	CAlignBase::DoIt(pPackage);
	nvtxRangePop();
	//-------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);	
	//--------------------------------------------------------------
	bool bForward = true;
	nvtxRangePushA ("mFourierTransform(bForward)");
	mFourierTransform(bForward);
	nvtxRangePop();
	//----------------------------
	CLoadAlign* pLoadAlign = CLoadAlign::GetInstance();
	if(pLoadAlign->IsLoaded())
	{	bool bClean = true;
		m_pFullShift = pLoadAlign->GetFullShift(bClean);
	}
	else
	{	m_pFullShift = new CStackShift;
		m_pFullShift->Setup(pFrmBuffer->m_iNumFrames);
		//--------------------------------------------
		CGenXcfStack::DoIt(m_pFullShift);
		mDoAlign();
		m_pFullShift->TruncateDecimal();
	}
	m_pFullShift->DisplayShift("Full-frame alignment shift", 0);
	//-------------------------------------------------------
	CGenStarFile* pGenStarFile = CGenStarFile::GetInstance();
	pGenStarFile->SetGlobalShifts(m_pFullShift->GetShifts(),
	   m_pFullShift->m_iNumFrames);
	pGenStarFile->CloseFile();
}

void CFullAlign::DoIt(DU::CDataPackage* pPackage)
{	
	this->Align(pPackage);
	mCorrect();
	//---------
	CSaveAlign* pSaveAlign = CSaveAlign::GetInstance();
	pSaveAlign->DoGlobal(m_pFullShift);
}

void CFullAlign::mFourierTransform(bool bForward)
{
	const char* pcForward = "Forward FFT of stack";
	const char* pcInverse = "Inverse FFT of stack";
	if(bForward) printf("%s, please wait...\n", pcForward);
	else printf("%s, please wait...\n", pcInverse);
	//---------------------------------------------
	bool bNorm = true;
	CTransformStack::DoIt(EBuffer::frm, bForward, bNorm);
	printf("Fourier transform done.\n");
}

void CFullAlign::mDoAlign(void)
{
	printf("Full-frame alignment has been started.\n");
	CIterativeAlign iterAlign;
	CInput* pInput = CInput::GetInstance();
	bool bPhaseOnly = (pInput->m_iPhaseOnly == 0)? false : true;
	//----------------------------------------------------------
	iterAlign.Setup(EBuffer::xcf, pInput->m_afBFactor[0], bPhaseOnly);
	iterAlign.DoIt(m_pPackage, m_pFullShift);
	//---------------------------------------
	char* pcErrLog = iterAlign.GetErrorLog();
	printf("%s\n", pcErrLog);
	if(pcErrLog != 0L) delete[] pcErrLog;
	//----------------------------------------------------------
	// If patch align will be performed, we transform xcf buffer
	// back to real space.
	//----------------------------------------------------------
	CAlignParam* pAlignParam = CAlignParam::GetInstance();
	int iFrmRef = pAlignParam->GetFrameRef(m_pFullShift->m_iNumFrames);
	//-------------------------------
	if(pInput->m_aiNumPatches[0] > 1 && pInput->m_aiNumPatches[1] > 1)
	{	CDetectFeatures* pDetectFeatures = 
		   CDetectFeatures::GetInstance();
		pDetectFeatures->DoIt(m_pFullShift, pInput->m_aiNumPatches);
		//----------------------------------------------------------
		CPatchCenters* pPatchCenters = CPatchCenters::GetInstance();
		pPatchCenters->Calculate();
		//----------------------------------------------------------
		bool bCorrBilinear = true, bMotionDecon = true;
		bool bGenReal = true;
		Correct::CGenRealStack::DoIt(EBuffer::xcf, !bCorrBilinear, 
		   !bMotionDecon, bGenReal, m_pFullShift);
	}
	//-----------------------------------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	float* pfXcfBin = pBufferPool->m_afXcfBin;	
	m_pFullShift->Multiply(pfXcfBin[0], pfXcfBin[1]);
}

void CFullAlign::mCorrect(void)
{
	printf("Create aligned sum based upon full frame alignment.\n");
	Correct::CCorrectFullShift aCorrFullShift;
	aCorrFullShift.DoIt(m_pFullShift, m_pPackage); 
}

void CFullAlign::LogShift(char* pcLogFile)
{
	if(m_pFullShift == 0L) return;
	if(pcLogFile == 0L || strlen(pcLogFile) == 0) return;
	//---------------------------------------------------
	float afShift[2] = {0.0f};
	const char* pcFormat = "%4d  %8.2f  %8.2f\n";
	char acFile[256];
	sprintf(acFile, "%s-Full.log", pcLogFile);
	FILE* pFile = fopen(acFile, "wt");
	if(pFile == 0L) return;
	//---------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);	
	//--------------------------------------------------------------
	fprintf(pFile, "# full frame alignment\n");
	for(int i=0; i<pFrmBuffer->m_iNumFrames; i++)
	{	m_pFullShift->GetShift(i, afShift);
		fprintf(pFile, pcFormat, i+1, afShift[0], afShift[1]); 
	}
	fclose(pFile);
}

