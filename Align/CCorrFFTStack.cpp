#include "CAlignInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include "../Util/CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

void CCorrFFTStack::DoIt
(	MrcUtil::GFFTStack* pGFFTStack,
	CStackShift* pStackShift
)
{    Util::CNextItem nextItem;
	nextItem.Create(pGFFTStack->m_aiStkSize[2]);
	//------------------------------------------
	int iNumGpus = pGFFTStack->GetNumGpus();
	//--------------------------------------
	CCorrFFTStack* pThreads = new CCorrFFTStack[iNumGpus];
	for(int i=0; i<iNumGpus; i++)
	{	pThreads[i].Run(pGFFTStack, pStackShift, &nextItem, i);
	};
	for(int i=0; i<iNumGpus; i++)
	{	pThreads[i].WaitForExit(-1.0f);
	}
	delete[] pThreads;
}

CCorrFFTStack::CCorrFFTStack(void)
{
}

CCorrFFTStack::~CCorrFFTStack(void)
{
}

void CCorrFFTStack::Run
(    MrcUtil::GFFTStack* pGFFTStack,
     CStackShift* pStackShift,
     Util::CNextItem* pNextItem, 
     int iThreadId
)
{	m_pGFFTStack = pGFFTStack;
     m_pStackShift = pStackShift;
	m_pNextItem = pNextItem;
	m_iThreadId = iThreadId;
	m_iGpuId = m_pGFFTStack->GetGpuId(iThreadId);
	this->Start();
}

void CCorrFFTStack::ThreadMain(void)
{
	cudaSetDevice(m_iGpuId);
	//----------------------
	mCorrGpuFrames();
	mCorrCpuFrames();
}

void CCorrFFTStack::mCorrGpuFrames(void)
{
	float afShift[2] = {0};
	for(int i=0; i<m_pGFFTStack->m_aiStkSize[2]; i++)
	{	m_pStackShift->GetShift(i, afShift);
		if(afShift[0] == 0 && afShift[1] == 0) continue;
		//----------------------------------------------
		int iGpuId = m_pGFFTStack->GetFrameGpu(i);
		if(iGpuId != m_iGpuId) continue;
		//------------------------------
		m_pGFFTStack->Align(i, afShift);
	}
}

void CCorrFFTStack::mCorrCpuFrames(void)
{
	float afShift[2] = {0.0f};
	cufftComplex* gCmpBuf = (cufftComplex*)Util::GetGpuBuf
	( m_pGFFTStack->m_aiCmpSize, sizeof(cufftComplex), false
	);
	//------------------------------------------------------
	while(true)
	{	int i = m_pNextItem->GetNext();
		if(i < 0) break;
		if(m_pGFFTStack->IsGpuFrame(i)) continue;
		//---------------------------------------
		m_pStackShift->GetShift(i, afShift);
		if(afShift[0] == 0 && afShift[1] == 0) continue;
		//----------------------------------------------
		m_pGFFTStack->Align(gCmpBuf, i, afShift);
	}
	if(gCmpBuf != 0L) cudaFree(gCmpBuf);
}

