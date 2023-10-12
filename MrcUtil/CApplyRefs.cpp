#include "CMrcUtilInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <Util/Util_Time.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>
#include <nvToolsExt.h>

using namespace MotionCor2;
using namespace MotionCor2::MrcUtil;

static DataUtil::CMrcStack* s_pMrcStack = 0L;
static float* s_pfGain = 0L;
static float* s_pfDark = 0L;

CApplyRefs::CApplyRefs(void)
{	
	m_gfDark = 0L;
	m_gfGain = 0L;
	m_pvMrcFrames[0] = 0L;
        m_pvMrcFrames[1] = 0L;
}

CApplyRefs::~CApplyRefs(void)
{
}

//-----------------------------------------------------------------------------
// 1. This is the place where GFFTStack first gets created. However, the
//    frames are padded but not yet transformed into Fourier space.
// 2. Even if there is no gain reference, i.e. pfGain is null, the original
//    frames are stilled padded and put into GFFTStack.
//-----------------------------------------------------------------------------
void CApplyRefs::DoIt
(	DataUtil::CMrcStack* pMrcStack,
	float* pfGain,
	float* pfDark
)
{       nvtxRangePushA("CApplyRefs::DoIt");
	//---------------------------------
	s_pMrcStack = pMrcStack;
	s_pfGain = pfGain;
	s_pfDark = pfDark;
	//----------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	int iNumGpus = pBufferPool->m_iNumGpus;	
	CApplyRefs* pApplyRefs = new CApplyRefs[iNumGpus];
	//------------------------------------------------
	for(int i=0; i<iNumGpus; i++)
	{	pApplyRefs[i].Run(i);
	}
	for(int i=0; i<iNumGpus; i++)
	{	pApplyRefs[i].Wait();
	}
	//---------------------------
	delete[] pApplyRefs;
	nvtxRangePop();
}

void CApplyRefs::Run(int iNthGpu)
{
	m_iNthGpu = iNthGpu;	
	//------------------{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(m_iNthGpu);
	//--------------------------------
	cudaStreamCreate(&m_aStreams[0]);
	cudaStreamCreate(&m_aStreams[1]);
	//-------------------------------
	mCopyRefs();
	mCorrectCpuFrames();
	mCorrectGpuFrames();
}

void CApplyRefs::Wait(void)
{
	cudaStreamSynchronize(m_aStreams[0]);
	cudaStreamSynchronize(m_aStreams[1]);
	cudaStreamDestroy(m_aStreams[0]);
	cudaStreamDestroy(m_aStreams[1]);
	//-------------------------------
	if(s_pMrcStack->m_iMode == 2) cudaFreeHost(m_pvMrcFrames[1]);
        m_pvMrcFrames[0] = 0L;
        m_pvMrcFrames[1] = 0L;
	m_gfDark = 0L;
	m_gfGain = 0L;
}

void CApplyRefs::mCopyRefs(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	cufftComplex* gCmpDark = pTmpBuffer->GetFrame(m_iNthGpu, 0);
	cufftComplex* gCmpGain = pTmpBuffer->GetFrame(m_iNthGpu, 1);
	//----------------------------------------------------------
	int iFrmSizeX = (pFrmBuffer->m_aiCmpSize[0] - 1) * 2;
	int iFrmSizeY = pFrmBuffer->m_aiCmpSize[1];
	int iStartX = (s_pMrcStack->m_aiStkSize[0] - iFrmSizeX) / 2;
	int iStartY = (s_pMrcStack->m_aiStkSize[1] - iFrmSizeY) / 2;
	int iOffset = iStartY * s_pMrcStack->m_aiStkSize[0] + iStartX;
	//------------------------------------------------------------
	int aiPadSize[] = {1, pFrmBuffer->m_aiCmpSize[1]};
	aiPadSize[0] = pFrmBuffer->m_aiCmpSize[0] * 2;
	int iCpySizeX = (pFrmBuffer->m_aiCmpSize[0] - 1) * 2;
	//---------------------------------------------------
	m_aGAppRefsToFrame.SetSizes
	( s_pMrcStack->m_aiStkSize, aiPadSize, true );
	if(s_pfDark != 0L)
	{	m_gfDark = reinterpret_cast<float*>(gCmpDark);
		float* pfDark = s_pfDark + iOffset;
		Util::GPartialCopy::DoIt
		( pfDark, s_pMrcStack->m_aiStkSize[0], m_gfDark,
		  iCpySizeX, aiPadSize, m_aStreams[0] );
	}
	if(s_pfGain != 0L)
	{	m_gfGain = reinterpret_cast<float*>(gCmpGain);
		float* pfGain = s_pfGain + iOffset;
		Util::GPartialCopy::DoIt(pfGain, s_pMrcStack->m_aiStkSize[0], 
		   m_gfGain, iCpySizeX, aiPadSize, m_aStreams[0]); 
	}
	//--------------------------------------------------------
	m_pvMrcFrames[0] = pBufferPool->GetPinnedBuf(m_iNthGpu);
        if(s_pMrcStack->m_iMode != 2)
        {       char* pcFrame0 = reinterpret_cast<char*>(m_pvMrcFrames[0]);
                m_pvMrcFrames[1] = pcFrame0 + s_pMrcStack->m_tFmBytes;
        }
        else
        {       cudaMallocHost(&m_pvMrcFrames[1], s_pMrcStack->m_tFmBytes);
        }
	cudaStreamSynchronize(m_aStreams[0]);
	//-----------------------------------
	m_aGAppRefsToFrame.SetRefs(m_gfGain, m_gfDark);
}

void CApplyRefs::mCorrectGpuFrames(void)
{
        nvtxRangePushA ("CApplyRefs::mCorrectFrames");
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	//--------------------------------------------------------------
	int iNumFrames = pFrmBuffer->GetNumFrames(m_iNthGpu);
	int iStartFrame = pFrmBuffer->GetStartFrame(m_iNthGpu);
	//-----------------------------------------------------
	for(int i=0; i<iNumFrames; i++)
	{	if(!pFrmBuffer->IsGpuFrame(m_iNthGpu, i)) continue;
		m_iAbsFrame = iStartFrame + i;
		cufftComplex* gCmpFrm = pFrmBuffer->GetFrame(m_iNthGpu, i);
		int iStream = i % 2;
		mApplyRefs(gCmpFrm, iStream);
	}
        nvtxRangePop();
}

void CApplyRefs::mCorrectCpuFrames(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	CStackBuffer* pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	//--------------------------------------------------------------
	cufftComplex* pCmpFrm = 0L;
	cufftComplex* gCmpBufs[2] = {0L};
	gCmpBufs[0] = pTmpBuffer->GetFrame(m_iNthGpu, 2);
	gCmpBufs[1] = pSumBuffer->GetFrame(m_iNthGpu, 0);
	//-----------------------------------------------
	int iCount = 0;
	size_t tBytes = pFrmBuffer->m_tFmBytes;
	int iNumFrames = pFrmBuffer->GetNumFrames(m_iNthGpu);
	int iStartFrm = pFrmBuffer->GetStartFrame(m_iNthGpu);
	for(int i=0; i<iNumFrames; i++)
	{	if(pFrmBuffer->IsGpuFrame(m_iNthGpu, i)) continue;
		m_iAbsFrame = i + iStartFrm;
		int iStream = iCount % 2;
		//-----------------------
		mApplyRefs(gCmpBufs[iStream], iStream);
		pCmpFrm = pFrmBuffer->GetFrame(m_iNthGpu, i);
		cudaMemcpyAsync(pCmpFrm, gCmpBufs[iStream], tBytes,
			cudaMemcpyDefault, m_aStreams[iStream]);
		iCount += 1;
	}
}

void CApplyRefs::mApplyRefs(cufftComplex* gCmpFrm, int iStream)
{	
	void* pvRawFrm = s_pMrcStack->GetFrame(m_iAbsFrame);
	float* gfPadFrm = reinterpret_cast<float*>(gCmpFrm);
	//--------------------------------------------------
	cudaStreamSynchronize(m_aStreams[iStream]);
	/*
	if(pvRawFrm == 0L)
	{	printf("m_iAbsFrame numFrms: %d  %d\n", m_iAbsFrame,
			s_pMrcStack->m_aiStkSize[2]);
	}
	*/
	memcpy(m_pvMrcFrames[iStream], pvRawFrm, s_pMrcStack->m_tFmBytes);


	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pFrmBuffer = pBufferPool->GetBuffer(EBuffer::frm);
	int aiPadSize[] = {1, pFrmBuffer->m_aiCmpSize[1]};
	aiPadSize[0] = pFrmBuffer->m_aiCmpSize[0] * 2;
	//--------------------------------------------
	int iMode = s_pMrcStack->m_iMode;
	//-------------------------------
	m_aGAppRefsToFrame.DoIt(m_pvMrcFrames[iStream], iMode,
           gfPadFrm, m_aStreams[iStream]);
	//m_aGAppRefsToFrame.DoIt(pvRawFrm, iMode, 
	//   gfPadFrm, m_aStreams[iStream]);
}

