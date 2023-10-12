#include "CDetectMain.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <nvToolsExt.h>

using namespace MotionCor2::BadPixel;

static int s_aiModSize[2] = {0};

CLocalCCMap::CLocalCCMap(void)
{
}

CLocalCCMap::~CLocalCCMap(void)
{
}

void CLocalCCMap::DoIt(int* piModSize)
{	
	nvtxRangePushA("CLocalCCMap::DoIt");
	s_aiModSize[0] = piModSize[0];
	s_aiModSize[1] = piModSize[1];
	//----------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	int iNumGpus = pBufferPool->m_iNumGpus;
	CLocalCCMap* pLocalCCMaps = new CLocalCCMap[iNumGpus];
	for(int i=0; i<iNumGpus; i++)
	{	pLocalCCMaps[i].mDoIt(i);
	}
	for(int i=0; i<iNumGpus; i++)
	{	pLocalCCMaps[i].mWait();
	}
	delete[] pLocalCCMaps;
	nvtxRangePop();
}

void CLocalCCMap::mDoIt(int iNthGpu)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(iNthGpu);
	m_iNthGpu = iNthGpu;
	//------------------
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
	m_gfPadSum = reinterpret_cast<float*>(gCmpSum);
	//---------------------------------------------
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	cufftComplex* gCmpCC = pTmpBuffer->GetFrame(0, 0);
	m_gfPadCC = reinterpret_cast<float*>(gCmpCC);
	//-------------------------------------------
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	m_aiPadSize[0] = pFrmBuffer->m_aiCmpSize[0] * 2;
	m_aiPadSize[1] = pFrmBuffer->m_aiCmpSize[1];
	//------------------------------------------
	void* pvPinnedBuf = pBufferPool->GetPinnedBuf(0);
	char* pcPinnedBuf = reinterpret_cast<char*>(pvPinnedBuf);
	char* pcMod = pcPinnedBuf + m_aiPadSize[0] * m_aiPadSize[1];
	m_pfMod = reinterpret_cast<float*>(pcMod);	
	//----------------------------------------
	CTemplate aTemplate;
        aTemplate.Create(s_aiModSize, m_pfMod);
	//-------------------------------------
	cudaStreamCreate(&m_aStream);
	//---------------------------
	int iPadSize = m_aiPadSize[0] * m_aiPadSize[1];
	int iPartSize = iPadSize / pBufferPool->m_iNumGpus;
	int iOffset = iPartSize * m_iNthGpu; 
	//----------------------------------
	GLocalCC aGLocalCC;
	aGLocalCC.SetRef(m_pfMod, s_aiModSize);
	if(iNthGpu == 0)
	{	aGLocalCC.DoIt(m_gfPadSum, m_aiPadSize, iOffset,
			iPartSize, m_gfPadCC, m_aStream);
		return;
	}
	//-------------
	cufftComplex* gCmpBuf = pTmpBuffer->GetFrame(m_iNthGpu, 0);
	float* gfPadSum = reinterpret_cast<float*>(gCmpBuf);
	cudaMemcpyAsync(gfPadSum, m_gfPadSum, iPadSize * sizeof(float), 
		cudaMemcpyDefault, m_aStream);
	//------------------------------------
	gCmpBuf = pTmpBuffer->GetFrame(m_iNthGpu, 1);
	float* gfPadCC = reinterpret_cast<float*>(gCmpBuf);
	//-------------------------------------------------
	aGLocalCC.DoIt(gfPadSum, m_aiPadSize, iOffset, iPartSize,
		gfPadCC, m_aStream);
	//--------------------------
	cudaMemcpyAsync(m_gfPadCC + iOffset, gfPadCC + iOffset, 
		sizeof(float) * iPartSize, cudaMemcpyDefault, m_aStream);
}

void CLocalCCMap::mWait(void)
{
	cudaStreamSynchronize(m_aStream);
	cudaStreamDestroy(m_aStream);
}

