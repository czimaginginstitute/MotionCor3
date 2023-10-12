#include "CAlignInc.h"
#include "../Util/CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;

static int s_iBufferID = 0;
static Util::CGroupFrames* s_pGroupOld = 0L;
static Util::CGroupFrames* s_pGroupNew = 0L;

//-----------------------------------------------------------------------------
// 1. Divide the frames in pGFFTStack in groups given the number of frames
//    in each group and then calculate the sum of the frames in each group.
//    Return a new GFFTStack containing the group sums. 
// 2. The frames in the input GFFTStack stacks are deleted.
//-----------------------------------------------------------------------------
void CGroupFFTStack::DoIt
(	int iBufferID,
	Util::CGroupFrames* pGroupFrames	
)
{	s_iBufferID = iBufferID;
	s_pGroupOld = pGroupFrames;
	//-------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	int iNumGpus = pBufferPool->m_iNumGpus;
	int iNumGroups = s_pGroupOld->GetNumGroups();
	//-------------------------------------------
	Util::CGroupFrames groupNew;
	groupNew.DoNumGroups(iNumGroups, iNumGpus);
	s_pGroupNew = &groupNew;	
	//----------------------
	CGroupFFTStack* pThreads = new CGroupFFTStack[iNumGpus];
	for(int i=0; i<iNumGpus; i++)
	{	pThreads[i].Run(i);
	};
	for(int i=0; i<iNumGpus; i++)
	{	pThreads[i].WaitForExit(-1.0f);
	}
	//-------------------------------------
	delete[] pThreads;
}

CGroupFFTStack::CGroupFFTStack(void)
{
}

CGroupFFTStack::~CGroupFFTStack(void)
{
}

void CGroupFFTStack::Run(int iNthGpu)
{
	m_iNthGpu = iNthGpu;	
	this->Start();
}

void CGroupFFTStack::ThreadMain(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(m_iNthGpu);
	//--------------------------------
	int iNewFrames = m_pGroupNew->GetGroupSize(m_iThreadId);
	for(int i=0; i<iNumNewFrames; i++)
	{	int iOldGroup = i;
		cufftComplex* gCmpSum = mCalcGroupSum(iOldGroup);
	}
	//-------------------------------------------------------
	for(int i=iNumGpuFrames; i<iNewFrames; i++)
	{	int iNewFrm = i + iStart;
		cufftComplex* gCmpSum = mCalcGroupSum(iNewFrm);
		//---------------------------------------------
		cufftComplex* pCmpSum = Util::CCopyFrame(gCmpSum,
		   m_pGroupedStack->m_aiCmpSize);
		m_pGroupedStack->SetFrame(iNewFrm, pCmpSum, -1);
		cudaFree(gCmpSum);
	}
}

cufftComplex* CGroupFFTStack::mCalcGroupSum(int iGroup)
{
	int iOldStart = m_pGroupOld->GetGroupStart(iGroup);
	int iOldFrames = m_pGroupOld->GetGroupSize(iGroup);
        //-------------------------------------------------
        cufftComplex* gCmpSum =  

	cufftComplex* gCmpSum = mCopyOldFrame(iOldStart);
	if(iOldFrames == 1) return gCmpSum;
	//---------------------------------
	bool bClean = true;
	cufftComplex* gCmpBuf = 0L;
	//-------------------------
	for(int i=2; i<iOldFrames; i++)
	{	int iOldFrame = i + iOldStart;
		int iGpuId = m_pGFFTStack->GetFrameGpu(iOldFrame);
		if(iGpuId == m_iGpuId)
		{	cufftComplex* gCmpFrm = m_pGFFTStack->GetFrame(
			   iOldFrame, !bClean);
			mAddFrames(gCmpSum, gCmpFrm);
		}
		else
		{	if(gCmpBuf == 0L) gCmpBuf = mCopyOldFrame(iOldFrame);
			else mCopyOldFrame(iOldFrame, gCmpBuf);
			mAddFrames(gCmpSum, gCmpBuf);
		}
		if(m_bCleanFrames) m_pGFFTStack->DeleteFrame(iOldFrame);
	}
	//--------------------------------------------------------------
	if(gCmpBuf != 0L) cudaFree(gCmpBuf);
	return gCmpSum;
}

cufftComplex* CGroupFFTStack::mCopyOldFrame(int iOldFrame)
{
	bool bClean = true;
	cufftComplex* gCmpFrm = m_pGFFTStack->GetFrame(iOldFrame, !bClean);
	//----------------------------------------------------------------
	cufftComplex* gCmpCopy = Util::GCopyFrame(gCmpFrm, 
	   m_pGFFTStack->m_aiCmpSize);
}

void CGroupFFTStack::mCopyOldFrame(int iOldFrame, cufftComplex* gCmpBuf)
{
	bool bClean = true;
	cufftComplex* pCmpFrm = m_pGFFTStack->GetFrame(iOldFrame, !bClean);
	Util::CopyFrame(pCmpFrm, gCmpBuf, m_pGFFTStack->m_aiCmpSize);
}

void CGroupFFTStack::mAddFrames
(	cufftComplex* gCmpSum,
	cufftComplex* gCmpFrm)
{
	Util::GAddFrames addFrames;
	addFrames.DoIt(gCmpSum, 1.0f, gCmpFrm, 1.0f, gCmpSum,
	   m_pGFFTStack->m_aiCmpSize);
}
