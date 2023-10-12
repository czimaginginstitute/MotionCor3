#include "CAlignInc.h"
#include "../Util/CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <memory.h>
#include <stdio.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;
static CStackShift* s_pStackShift = 0L;

CGenXcfStack::CGenXcfStack(void)
{
}

CGenXcfStack::~CGenXcfStack(void)
{
}

void CGenXcfStack::DoIt(CStackShift* pStackShift)
{	
	nvtxRangePushA ("CGenXcfStack::DoIt");
	s_pStackShift = pStackShift;
	//----------------------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	int iNumGpus = pBufferPool->m_iNumGpus;
	CGenXcfStack* pGenXcfStacks = new CGenXcfStack[iNumGpus];
	for(int i=0; i<iNumGpus; i++)
	{	pGenXcfStacks[i].mDoIt(i);
	}
	for(int i=0; i<iNumGpus; i++)
	{	pGenXcfStacks[i].mWaitStreams();
	}
	delete[] pGenXcfStacks;	
        s_pStackShift = 0L;
	nvtxRangePop();
}

void CGenXcfStack::mDoIt(int iNthGpu)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	m_iNthGpu = iNthGpu;
	pBufferPool->SetDevice(m_iNthGpu);
	//--------------------------------
	m_pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	m_pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	m_pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	m_pXcfBuffer = pBufferPool->GetBuffer(EBuffer::xcf);
	//--------------------------------------------------
	cudaStreamCreate(&m_aStreams[0]);
	cudaStreamCreate(&m_aStreams[1]);
	//-------------------------------
	int iStartFrm = m_pXcfBuffer->GetStartFrame(m_iNthGpu);
	int iNumFrames = m_pXcfBuffer->GetNumFrames(m_iNthGpu);
	for(int i=0; i<iNumFrames; i++)
	{	m_iStream = i % 2;
		mDoXcfFrame(i);
	}
}

void CGenXcfStack::mWaitStreams(void)
{
	m_pFrmBuffer->SetDevice(m_iNthGpu);
	cudaStreamSynchronize(m_aStreams[0]);
	cudaStreamSynchronize(m_aStreams[1]);
	cudaStreamDestroy(m_aStreams[0]);
	cudaStreamDestroy(m_aStreams[1]);
}

void CGenXcfStack::mDoXcfFrame(int iXcfFrm)
{
	int iStartXcf = m_pXcfBuffer->GetStartFrame(m_iNthGpu);
	int iAbsFrm = iStartXcf + iXcfFrm;
	//-----------------------------------------------------
	DataUtil::CStackFolder* pStackFolder = 
	   DataUtil::CStackFolder::GetInstance();
	//------------------------------------------------	
	size_t tBytes = m_pFrmBuffer->m_tFmBytes;
	cudaStream_t stream = m_aStreams[m_iStream];
	//------------------------------------------
	cufftComplex *gCmpXcf, *gCmpFrm, *gCmpTmp, *gCmpBuf;
	gCmpXcf = m_pXcfBuffer->GetFrame(m_iNthGpu, iXcfFrm);
	gCmpTmp = m_pTmpBuffer->GetFrame(m_iNthGpu, m_iStream);
	gCmpFrm = m_pFrmBuffer->GetFrame(iAbsFrm);
	//-----------------------------------------------------
	Util::GFourierResize2D fourierResize;
	Util::GPhaseShift2D phaseShift;
	float afShift[2] = {0.0f};
	if(s_pStackShift != 0L) 
	{	s_pStackShift->GetShift(iAbsFrm, afShift, -1.0f);
	}
	//-------------------------------------------------------
	int iFmGpu = m_pFrmBuffer->GetFrameGpu(iAbsFrm);
	if(afShift[0] == 0 && afShift[1] == 0)
	{	fourierResize.DoIt(gCmpFrm, m_pFrmBuffer->m_aiCmpSize,
		   gCmpXcf, m_pXcfBuffer->m_aiCmpSize, false, stream);
	}
	else
	{	Util::GPhaseShift2D phaseShift;
		phaseShift.DoIt(gCmpFrm, m_pFrmBuffer->m_aiCmpSize,
		   afShift, false, gCmpTmp, stream);
		fourierResize.DoIt(gCmpTmp, m_pFrmBuffer->m_aiCmpSize,
		   gCmpXcf, m_pXcfBuffer->m_aiCmpSize, false, stream);
	}
}

