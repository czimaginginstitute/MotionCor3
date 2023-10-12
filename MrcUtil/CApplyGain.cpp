#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <Util/Util_Time.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2;
using namespace MotionCor2::MrcUtil;

static CMrcStack* s_pMrcStack = 0L;
static float* s_pfGain = 0L;

CApplyGain::CApplyGain(void)
{
}

CApplyGain::~CApplyGain(void)
{
}

//-----------------------------------------------------------------------------
// 1. This is the place where GFFTStack first gets created. However, the
//    frames are padded but not yet transformed into Fourier space.
// 2. Even if there is no gain reference, i.e. pfGain is null, the original
//    frames are stilled padded and put into GFFTStack.
//-----------------------------------------------------------------------------
GFFTStack* CApplyGain::DoIt
(	CMrcStack* pMrcStack,
	float* pfGain,
	int* piGpuIds,
	int iNumGpus
)
{	int iNumFrames = pMrcStack->m_aiStkSize[2];
	if(iNumGpus > iNumFrames) iNumGpus = iNumFrames;
	//----------------------------------------------
	s_pMrcStack = pMrcStack;
	s_pfGain = pfGain;
	//----------------
	GFFTStack* pGFFTStack = new GFFTStack;
	pGFFTStack->Create(pMrcStack->m_aiStkSize);
	pGFFTStack->SetGpus(piGpuIds, iNumGpus);
	//--------------------------------------
	Util::CGroupFrames groupFrames;
	groupFrames.DoNumGroups(s_pMrcStack->m_aiStkSize[2], iNumGpus);
	//-------------------------------------------------------------
	CApplyGain* pThreads = new CApplyGain[iNumGpus];
	//----------------------------------------------
	for(int i=0; i<iNumGpus; i++)
	{	pThreads[i].Run(&groupFrames, i, pGFFTStack);
	}
	for(int i=0; i<iNumGpus; i++)
	{	pThreads[i].WaitForExit(-1.0f);
	}
	delete[] pThreads;
	//----------------
	return pGFFTStack;
}

void CApplyGain::Run
(	Util::CGroupFrames* pGroupFrames,
	int iThread,
	GFFTStack* pGFFTStack
)
{	m_pGFFTStack = pGFFTStack;
	m_pGroupFrames = pGroupFrames;
	m_iThread = iThread;
	//------------------
	this->Start();
}

void CApplyGain::ThreadMain(void)
{
	m_iGpuId = m_pGFFTStack->GetGpuId(m_iThread);
	cudaSetDevice(m_iGpuId);
	//----------------------
	m_gfGain = 0L;
	if(s_pfGain != 0L)
	{	m_gfGain = Util::GCopyFrame(s_pfGain,
		   s_pMrcStack->m_aiStkSize);
	}
	//-----------------------------------
	mCorrectFrames();
	//---------------
	if(m_gfGain != 0L) cudaFree(m_gfGain);
	m_gfGain = 0L;
}

void CApplyGain::mCorrectFrames(void)
{	
	int iNumGpuFrames = Util::CalcNumGpuFrames
	(  m_pGFFTStack->m_aiPadSize, m_iGpuId
	);
	//------------------------------------
	m_gvFrmBuf = 0L;
	m_gfPadBuf = 0L;
	//--------------
	int iStart = m_pGroupFrames->GetGroupStart(m_iThread);
	int iNumFrames = m_pGroupFrames->GetGroupSize(m_iThread);
	//-------------------------------------------------------
	for(int i=0; i<iNumFrames; i++)
	{	int j = i + iStart;
		if(i < iNumGpuFrames) mCorrectGpuFrame(j);
		else mCorrectCpuFrame(j);
	}
	//-------------------------------
	if(m_gvFrmBuf != 0L) cudaFree(m_gvFrmBuf);
	if(m_gfPadBuf != 0L) cudaFree(m_gfPadBuf);
}

void CApplyGain::mCorrectGpuFrame(int iFrame)
{
	Util_Time aTimer;
	aTimer.Measure();

	mCreateFrameBuf();
	//----------------
	mCopyFrame(iFrame, m_gvFrmBuf);
	bool bZero = true;
	float* gfPadBuf = (float*)m_pGFFTStack->GGetCmpBuf(!bZero);
	mApplyGain(m_gvFrmBuf, gfPadBuf);
	//-------------------------------
	m_pGFFTStack->SetFrame(iFrame, gfPadBuf, m_iGpuId);

	float fSec = aTimer.GetElapsedSeconds();
	printf("Gain GPU frame: %d  %e\n", iFrame+1, fSec);
}

void CApplyGain::mCorrectCpuFrame(int iFrame)
{
	Util_Time aTimer;
	aTimer.Measure();

	mCreateFrameBuf();
	mCreatePadBuf();
	//--------------
	mCopyFrame(iFrame, m_gvFrmBuf);
	mApplyGain(m_gvFrmBuf, m_gfPadBuf);
	//---------------------------------
	float* pfPadFrm = Util::CCopyFrame(m_gfPadBuf,
	   m_pGFFTStack->m_aiPadSize);
	//----------------------------
	m_pGFFTStack->SetFrame(iFrame, pfPadFrm, -1);

	float fSec = aTimer.GetElapsedSeconds();
	printf("Gain CPU frame: %d  %e\n", iFrame+1, fSec);
}

void CApplyGain::mCopyFrame(int iFrame, void* gvFrmBuf)
{
	bool bClean = true;
	size_t tBytes = s_pMrcStack->m_tFmBytes;
	//--------------------------------------
	char* pcFrame = s_pMrcStack->GetFrame(iFrame, bClean);
	cudaMemcpy(gvFrmBuf, pcFrame, tBytes, cudaMemcpyDefault);
	delete[] pcFrame;
}

void CApplyGain::mApplyGain(void* gvFrame, float* gfPadFrm)
{	
	bool bPadded = true;
	int iMode = s_pMrcStack->m_iMode;
	//-------------------------------
	GApplyGainToFrame aGApplyGainToFrame;
	aGApplyGainToFrame.SetGain(m_gfGain, s_pMrcStack->m_aiStkSize);
	aGApplyGainToFrame.DoIt(gvFrame, iMode, gfPadFrm, bPadded);
}

void CApplyGain::mCreateFrameBuf(void)
{
	if(m_gvFrmBuf != 0L) return;
	cudaMalloc(&m_gvFrmBuf, s_pMrcStack->m_tFmBytes);
}

void CApplyGain::mCreatePadBuf(void)
{
	if(m_gfPadBuf != 0L) return;
	bool bZero = true;
	m_gfPadBuf = (float*)m_pGFFTStack->GGetCmpBuf(!bZero);
}

