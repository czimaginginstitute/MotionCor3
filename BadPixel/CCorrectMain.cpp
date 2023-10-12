#include "CCorrectMain.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::BadPixel;

CCorrectMain::CCorrectMain(void)
{
}

CCorrectMain::~CCorrectMain(void)
{
}

void CCorrectMain::DoIt(int iDefectSize)
{		
	nvtxRangePushA("CCorrectMain::DoIt");
	m_iDefectSize = iDefectSize;
	//--------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	m_aiPadSize[0] = pFrmBuffer->m_aiCmpSize[0] * 2;
	m_aiPadSize[1] = pFrmBuffer->m_aiCmpSize[1];
	//------------------------------------------
	mSetGpus(pFrmBuffer->m_piGpuIDs, pFrmBuffer->m_iNumGpus);
	mCreateStreams(2);
	//----------------
	for(int i=0; i<pBufferPool->m_iNumGpus; i++)
	{	mCorrectFrames(i);
	}
	mDeleteStreams();
        nvtxRangePop();
}

void CCorrectMain::mCorrectFrames(int iNthGpu)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	//--------------------------------------------------------------
	m_iCurGpu = iNthGpu;
	pBufferPool->SetDevice(m_iCurGpu);
	//--------------------------------
	void* pvBuf = pBufferPool->GetPinnedBuf(0);
	size_t tBytes = m_aiPadSize[0] * m_aiPadSize[1] * sizeof(char);
	cufftComplex* gCmpBuf = pTmpBuffer->GetFrame(m_iCurGpu, 0);
	cudaMemcpy(gCmpBuf, pvBuf, tBytes, cudaMemcpyDefault);
	//----------------------------------------------------
	int iNumFrames = pFrmBuffer->GetNumFrames(m_iCurGpu);
	for(int i=0; i<iNumFrames; i++)
	{	mCorrectFrame(i);
	}
}

void CCorrectMain::mCorrectFrame(int iFrame)
{
	int iStream = iFrame % 2;
	cudaStream_t stream = m_pStreams[m_iCurGpu * 2 + iStream];
	//--------------------------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	//--------------------------------------------------------------
	cufftComplex* gCmpFrm = pFrmBuffer->GetFrame(m_iCurGpu, iFrame);
	cufftComplex* gCmpMap = pTmpBuffer->GetFrame(m_iCurGpu, 0);
	//---------------------------------------------------------
	float* gfPadFrm = reinterpret_cast<float*>(gCmpFrm);
	unsigned char* gucMap = reinterpret_cast<unsigned char*>(gCmpMap);
	//----------------------------------------------------------------
	GCorrectBad aGCorrectBad;
	aGCorrectBad.SetWinSize(m_iDefectSize);
	aGCorrectBad.GDoIt(gfPadFrm, gucMap, m_aiPadSize, true, stream);
}

