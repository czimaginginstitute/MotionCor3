#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <memory.h>
#include <stdio.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

static EBuffer s_eBuffer = EBuffer::frm;
static bool s_bForward = true;
static bool s_bNorm = true;

CTransformStack::CTransformStack(void)
{
}

CTransformStack::~CTransformStack(void)
{
}

void CTransformStack::DoIt
(	EBuffer eBuffer,
	bool bForward,
	bool bNorm
)
{	s_eBuffer = eBuffer;
	s_bForward = bForward;
	s_bNorm = bNorm;
	//--------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	int iNumGpus = pBufferPool->m_iNumGpus;
	//-------------------------------------
	nvtxRangePushA("CTransformStack::DoIt");
	CTransformStack* pTransformStacks = new CTransformStack[iNumGpus];
	for(int i=0; i<iNumGpus; i++)
	{	pTransformStacks[i].Run(i);
	}
	for(int i=0; i<iNumGpus; i++)
	{	if(iNumGpus == 1) break;
		pTransformStacks[i].WaitForExit(-1.0f);
	}
	delete[] pTransformStacks;
	nvtxRangePop();
}

void CTransformStack::Run(int iNthGpu)
{
	m_iNthGpu = iNthGpu;
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	if(pBufferPool->m_iNumGpus > 1) this->Start();
	else this->ThreadMain();
}

void CTransformStack::ThreadMain(void)
{	
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(m_iNthGpu);
	//--------------------------------
	m_pFrmBuffer = pBufferPool->GetBuffer(s_eBuffer);
	m_pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	int* piCmpSize = m_pFrmBuffer->m_aiCmpSize;
	int aiPadSize[] = {piCmpSize[0] * 2, piCmpSize[1]};
	//-------------------------------------------------
	if(s_bForward) 
	{	m_pCufft2D = pBufferPool->GetForwardFFT(m_iNthGpu);
		m_pCufft2D->CreateForwardPlan(aiPadSize, true);
	}
	else 
	{	m_pCufft2D = pBufferPool->GetInverseFFT(m_iNthGpu);
		m_pCufft2D->CreateInversePlan(piCmpSize, true);
	}
	//-----------------------------------------------------
	cudaStreamCreate(&m_aStreams[0]);
	cudaStreamCreate(&m_aStreams[1]);
	//-------------------------------
	mTransformCpuFrames();
	mTransformGpuFrames();
	//--------------------
	cudaDeviceSynchronize();
	cudaStreamDestroy(m_aStreams[0]);
	cudaStreamDestroy(m_aStreams[1]);
}

void CTransformStack::mTransformGpuFrames(void)
{
	cufftComplex* gCmpFrm = 0L;
	int iNumFrames = m_pFrmBuffer->GetNumFrames(m_iNthGpu);
	for(int i=0; i<iNumFrames; i++)
	{	if(!m_pFrmBuffer->IsGpuFrame(m_iNthGpu, i)) continue;
		gCmpFrm = m_pFrmBuffer->GetFrame(m_iNthGpu, i);
		mTransformFrame(gCmpFrm);
	}
}

void CTransformStack::mTransformCpuFrames(void)
{
	int iCount = 0;
	cufftComplex *pCmpFrm, *gCmpBuf;
	size_t tBytes = m_pFrmBuffer->m_tFmBytes;
	int iNumFrames = m_pFrmBuffer->GetNumFrames(m_iNthGpu);
	//-----------------------------------------------------
	for(int i=0; i<iNumFrames; i++)
	{	if(m_pFrmBuffer->IsGpuFrame(m_iNthGpu, i)) continue;
		int iStream = iCount % 2;
		pCmpFrm = m_pFrmBuffer->GetFrame(m_iNthGpu, i);
		gCmpBuf = m_pTmpBuffer->GetFrame(m_iNthGpu, iStream);
		//---------------------------------------------------
		cudaMemcpyAsync(gCmpBuf, pCmpFrm, tBytes, 
			cudaMemcpyDefault, m_aStreams[iStream]);
		if(iStream == 1) cudaStreamSynchronize(m_aStreams[1]);
		//----------------------------------------------------
		mTransformFrame(gCmpBuf);
		if(iStream == 1) cudaStreamSynchronize(m_aStreams[0]);
		//----------------------------------------------------
		cudaMemcpyAsync(pCmpFrm, gCmpBuf, tBytes,
			cudaMemcpyDefault, m_aStreams[iStream]);
		iCount += 1;
	}
}

void CTransformStack::mTransformFrame(cufftComplex* gCmpFrm) 
{
	if(s_bForward)
	{	float* gfPadFrm = reinterpret_cast<float*>(gCmpFrm);
		m_pCufft2D->Forward(gfPadFrm, s_bNorm, m_aStreams[0]);
	}
	else
	{	m_pCufft2D->Inverse(gCmpFrm, m_aStreams[0]);
	}
}
	
