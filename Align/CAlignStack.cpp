#include "CAlignInc.h"
#include "../Util/CUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2;
using namespace MotionCor2::Align;


CAlignStack::CAlignStack(void)
{
	m_pbGpuGroups = 0L;
	m_iNthGpu = -1;
}

CAlignStack::~CAlignStack(void)
{
	mDestroyStreams();
	if(m_pbGpuGroups != 0L) delete[] m_pbGpuGroups;
}

void CAlignStack::Set1(int iNthGpu)
{
	if(m_iNthGpu == iNthGpu) return;
	else mDestroyStreams();
	//---------------------
	int iCurGpu = -1;
	cudaGetDevice(&iCurGpu);	
	//----------------------
	m_iNthGpu = iNthGpu;
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(m_iNthGpu);
	cudaStreamCreate(&m_aStreams[0]);
	cudaStreamCreate(&m_aStreams[1]);
	//-------------------------------
	if(iCurGpu >= 0) cudaSetDevice(iCurGpu);
}

void CAlignStack::Set2(EBuffer eBuffer, DU::CFmGroupParam* pFmGroupParam)
{
	m_pFmGroupParam = pFmGroupParam;	
	//------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	m_pFrmBuffer = pBufferPool->GetBuffer(eBuffer);
	m_pTmpBuffer = pBufferPool->GetBuffer(EBuffer::tmp);
	//--------------------------------------------------
	m_aiGpuFmRange[0] = m_pFrmBuffer->GetStartFrame(m_iNthGpu);
	m_aiGpuFmRange[1] = m_pFrmBuffer->GetNumFrames(m_iNthGpu)
	   + m_aiGpuFmRange[0] - 1;
	//----------------------------------------------------------
	int iCurGpu = -1;
	cudaGetDevice(&iCurGpu);
	m_pFrmBuffer->SetDevice(m_iNthGpu);
	//---------------------------------
	int* piCmpSize = m_pFrmBuffer->m_aiCmpSize;
	int iImgSizeX = (piCmpSize[0] - 1) * 2;
	m_aiSeaSize[0] = (64 < iImgSizeX) ? 64 : iImgSizeX;
	m_aiSeaSize[1] = (64 < piCmpSize[1]) ? 64 : piCmpSize[1];
	//-------------------------------------------------------
	m_aGCorrelateSum.SetSize(m_pFrmBuffer->m_aiCmpSize, m_aiSeaSize);
	//---------------------------------------------------------------
	m_pInverseFFT = pBufferPool->GetInverseFFT(m_iNthGpu);
	m_pInverseFFT->CreateInversePlan(m_pFrmBuffer->m_aiCmpSize, true);
	//----------------------------------------------------------------
	if(m_pbGpuGroups != 0L) delete[] m_pbGpuGroups;
	m_pbGpuGroups = new bool[m_pFmGroupParam->m_iNumGroups];
	for(int i=0; i<m_pFmGroupParam->m_iNumGroups; i++)
	{	m_pbGpuGroups[i] = mCheckGpuGroup(i);
	}
	//-------------------------------------------
	if(iCurGpu >= 0) cudaSetDevice(iCurGpu);
}

void CAlignStack::Set3(float fBFactor, bool bPhaseOnly)
{
	m_aGCorrelateSum.SetFilter(fBFactor, bPhaseOnly);
}

void CAlignStack::DoIt(CStackShift* pStackShift, CStackShift* pGroupShift)
{
	m_pStackShift = pStackShift;
	m_pGroupShift = pGroupShift;
	m_pFrmBuffer->SetDevice(m_iNthGpu);
	//-----------------------------------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	CStackBuffer* pSumBuffer = pBufferPool->GetBuffer(EBuffer::sum);
	m_gCmpSum = pSumBuffer->GetFrame(m_iNthGpu, 0);
	if(m_iNthGpu != 0)
	{	size_t tBytes = m_pFrmBuffer->m_tFmBytes;
		cufftComplex* gCmpSum = pSumBuffer->GetFrame(0, 0);
		cudaMemcpyAsync(m_gCmpSum, gCmpSum, tBytes, 
		   cudaMemcpyDefault, m_aStreams[0]);
		cudaStreamSynchronize(m_aStreams[0]);
	}
	//---------------------------------------------------
	m_fErr = 0.0f;
	mDoGroups();
}

void CAlignStack::WaitStreams(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(m_iNthGpu);
	//--------------------------------
	char acBuf[64];
	sprintf(acBuf, "CAlignStack: aaa %d", m_iNthGpu);
	cudaStreamSynchronize(m_aStreams[0]);
	Util::CheckCudaError(acBuf);
	cudaStreamSynchronize(m_aStreams[1]);
	Util::CheckCudaError("CAlignStack: bbb");
	mFindPeaks();
	Util::CheckCudaError("CAlignStack: ccc");
}

void CAlignStack::mDoGroups(void)
{
	int iStream = 0;	
	for(int i=0; i<m_pFmGroupParam->m_iNumGroups; i++)
	{	if(!m_pbGpuGroups[i]) continue;
		mDoGroup(i, iStream);
		iStream = (iStream + 1) % 2;
	}
}

//-----------------------------------------------------------------------------
// If the central frame of the group is allocated on this GPU, the group
// sum is calculated on this GPU although some other frames in this group
// may be allocated on other GPU(s);
//-----------------------------------------------------------------------------
bool CAlignStack::mCheckGpuGroup(int iGroup)
{
	int iGpStart = m_pFmGroupParam->GetGroupStart(iGroup);
	int iGpSize = m_pFmGroupParam->GetGroupSize(iGroup);
	int iGpCent = iGpStart + iGpSize / 2;
	int iFmStart = m_pFrmBuffer->GetStartFrame(m_iNthGpu);
	int iFmEnd = iFmStart + m_pFrmBuffer->GetNumFrames(m_iNthGpu) - 1;
	if(iGpCent < iFmStart) return false;
	else if(iGpCent > iFmEnd) return false;
	else return true;
}

void CAlignStack::mDoGroup(int iGroup, int iStream)
{
	int iGpStart = m_pFmGroupParam->GetGroupStart(iGroup);
	int iGpSize = m_pFmGroupParam->GetGroupSize(iGroup);
	//--------------------------------------------------
	for(int i=0; i<iGpSize; i++)
	{	m_iAbsFrm = iGpStart + i;
		bool bSum = (i > 0) ? true : false;
		mPhaseShift(iStream, bSum);
	}
	mCorrelate(iGroup, iStream);
}

void CAlignStack::mPhaseShift(int iStream, bool bSum)
{
	float afShift[2] = {0.0f};
	m_pStackShift->GetShift(m_iAbsFrm, afShift, -1.0f);
	cufftComplex* gCmpFrm = m_pFrmBuffer->GetFrame(m_iAbsFrm);
	cufftComplex* gCmpTmp = m_pTmpBuffer->GetFrame(m_iNthGpu, iStream);
	Util::GPhaseShift2D phaseShift;
	//-----------------------------
	cufftComplex* gCmpIn = gCmpFrm;
	if(m_iAbsFrm < m_aiGpuFmRange[0] ||
	   m_iAbsFrm > m_aiGpuFmRange[1])
	{	int iCmpSize = m_pFrmBuffer->m_aiCmpSize[0] *
		   m_pFrmBuffer->m_aiCmpSize[1];
		gCmpIn = gCmpTmp + iCmpSize;
		cudaMemcpyAsync(gCmpIn, gCmpFrm, m_pFrmBuffer->m_tFmBytes,
		   cudaMemcpyDefault, m_aStreams[iStream]);
	}
	//-------------------------------------------------
	phaseShift.DoIt(gCmpIn, m_pFrmBuffer->m_aiCmpSize,
	   afShift, bSum, gCmpTmp, m_aStreams[iStream]);
}

void CAlignStack::mCorrelate
(	int iGroup, 
	int iStream
)
{	int iSeaSize = m_aiSeaSize[0] * m_aiSeaSize[1];	
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	float* pfPinnedBuf = (float*)pBufferPool->GetPinnedBuf(m_iNthGpu);
	float* pfXcfBuf = pfPinnedBuf + iSeaSize * iGroup;
	//----------------------------------------------------------------
	cufftComplex* gCmpTmp = m_pTmpBuffer->GetFrame(m_iNthGpu, iStream);
	m_aGCorrelateSum.DoIt(m_gCmpSum, gCmpTmp, pfXcfBuf,
	   m_pInverseFFT, m_aStreams[iStream]);
}

void CAlignStack::mFindPeaks(void)
{
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	void* pvPinnedBuf = pBufferPool->GetPinnedBuf(m_iNthGpu);
	float* pfPinnedBuf = reinterpret_cast<float*>(pvPinnedBuf);
	int iSeaSize = m_aiSeaSize[0] * m_aiSeaSize[1];
	CPeak2D peak2D;
	//-------------
	for(int i=0; i<m_pGroupShift->m_iNumFrames; i++)
	{	if(!m_pbGpuGroups[i]) continue;
		float* pfXcfBuf = pfPinnedBuf + i * iSeaSize;
		peak2D.DoIt(pfXcfBuf, m_aiSeaSize);
		m_pGroupShift->SetShift(i, peak2D.m_afShift);
		mUpdateError(peak2D.m_afShift);
	}
}

void CAlignStack::mUpdateError(float* pfShift)
{
	double dS = sqrtf(pfShift[0] * pfShift[0] + pfShift[1] * pfShift[1]);
	if(m_fErr < dS) m_fErr = (float)dS;
}

void CAlignStack::mDestroyStreams(void)
{
	if(m_iNthGpu < 0) return;
	//-----------------------
	int iCurGpu = -1;
	cudaGetDevice(&iCurGpu);
	//----------------------
	CBufferPool* pBufferPool = CBufferPool::GetInstance();
	pBufferPool->SetDevice(m_iNthGpu);
	cudaStreamDestroy(m_aStreams[0]);
	cudaStreamDestroy(m_aStreams[1]);
	//-------------------------------
	if(iCurGpu >= 0) cudaSetDevice(iCurGpu);
}
