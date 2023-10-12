#include "CMainInc.h"
#include "Align/CAlignInc.h"
#include "DataUtil/CDataUtilInc.h"
#include "Util/CUtilInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <errno.h>
#include <Util/Util_Time.h>

using namespace MotionCor2;
namespace DU = MotionCor2::DataUtil;

CBufferPool* CBufferPool::m_pInstance = 0L;

CBufferPool* CBufferPool::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CBufferPool;
	return m_pInstance;
}

void CBufferPool::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CBufferPool::CBufferPool(void)
{
	m_pTmpBuffer = 0L;
	m_pSumBuffer = 0L;
	m_pFrmBuffer = 0L;
	m_pXcfBuffer = 0L;
	m_pPatBuffer = 0L;
	m_pvPinnedBuf = 0L;
	m_pCufft2Ds = 0L;
	//---------------
	m_piGpuIDs = 0L;
	m_iNumGpus = 0;
	memset(m_aiStkSize, 0, sizeof(m_aiStkSize));
}

CBufferPool::~CBufferPool(void)
{
	this->Clean();
}

void CBufferPool::Clean(void)
{
	if(m_pTmpBuffer != 0L) delete m_pTmpBuffer;
	if(m_pSumBuffer != 0L) delete m_pSumBuffer;
	if(m_pFrmBuffer != 0L) delete m_pFrmBuffer;
	if(m_pXcfBuffer != 0L) delete m_pXcfBuffer;
	if(m_pPatBuffer != 0L) delete m_pPatBuffer;
	if(m_pvPinnedBuf != 0L) cudaFreeHost(m_pvPinnedBuf);
	if(m_pCufft2Ds != 0L)
	{	for(int i=0; i<m_iNumGpus; i++)
		{	this->SetDevice(i);
			m_pCufft2Ds[2*i].DestroyPlan();
			m_pCufft2Ds[2*i+1].DestroyPlan();
		}
		delete[] m_pCufft2Ds;
	}
	//---------------------------
	m_pTmpBuffer = 0L;
	m_pSumBuffer = 0L;
	m_pFrmBuffer = 0L;
	m_pXcfBuffer = 0L;
	m_pPatBuffer = 0L;
	m_pvPinnedBuf = 0L;
	m_pCufft2Ds = 0L;
	memset(m_aiStkSize, 0, sizeof(m_aiStkSize));
}

void CBufferPool::SetGpus(int* piGpuIDs, int iNumGpus)
{
	this->Clean();
	//------------
	if(m_piGpuIDs != 0L) delete[] m_piGpuIDs;
	m_piGpuIDs = new int[iNumGpus];
	memcpy(m_piGpuIDs, piGpuIDs, sizeof(int) * iNumGpus);
	m_iNumGpus = iNumGpus;
}

//------------------------------------------------------------------------------
// 1. This is used for the first-time creation of the buffer. It should be
//    called only once throughout the lifetime once the program is started.
// 2. The frame size cannot be changed.
// 3. The number of frames in a movie can be changed using Adjust(...);
//------------------------------------------------------------------------------
void CBufferPool::CreateBuffers(int* piStkSize)
{
	this->Clean();
	memcpy(m_aiStkSize, piStkSize, sizeof(m_aiStkSize));
	m_pCufft2Ds = new Util::CCufft2D[2 * m_iNumGpus];
	//-----------------------------------------------
	Util_Time aTimer;
	aTimer.Measure();
	//---------------------------------------------------------------
	// If the proc queue does not have the package, this is because
	// the loading is not done and the package has not been placed
	// from the load queue to the proc queue. This happens in the
	// single processing or the first movie of the batch processing.
	//---------------------------------------------------------------
	mCreateSumBuffer();
	mCreateTmpBuffer();
	mCreateXcfBuffer();
	mCreatePatBuffer();
	mCreateFrmBuffer();
	//-----------------
	Util::CheckRUsage("Create buffers");
	float fTime = aTimer.GetElapsedSeconds();
	printf("Create buffers: %.2f seconds\n\n", fTime);	
}

void CBufferPool::AdjustBuffer(int iNumFrames)
{
	if(iNumFrames == m_aiStkSize[2]) return;
	else m_aiStkSize[2] = iNumFrames;
	//-----------------
	m_pFrmBuffer->Adjust(iNumFrames);
	m_pXcfBuffer->Adjust(iNumFrames);
	m_pPatBuffer->Adjust(iNumFrames);

}

CStackBuffer* CBufferPool::GetBuffer(EBuffer eBuf)
{
	if(eBuf == EBuffer::tmp) 
	{	if(m_pTmpBuffer == 0L) mCreateTmpBuffer();
		return m_pTmpBuffer;
	}
	if(eBuf == EBuffer::sum) 
	{	if(m_pSumBuffer == 0L) mCreateSumBuffer();
		return m_pSumBuffer;
	}
	if(eBuf == EBuffer::frm) 
	{	if(m_pFrmBuffer == 0L) mCreateFrmBuffer();
		return m_pFrmBuffer;
	}
	if(eBuf == EBuffer::xcf) 
	{	if(m_pXcfBuffer == 0L) mCreateXcfBuffer();
		return m_pXcfBuffer;
	}
	if(eBuf == EBuffer::pat) 
	{	if(m_pPatBuffer == 0L) mCreatePatBuffer();
		return m_pPatBuffer;
	}
	return 0L;
}

void* CBufferPool::GetPinnedBuf(int iNthGpu)
{
	char* pcPinnedBuf = reinterpret_cast<char*>(m_pvPinnedBuf);
	size_t tOffset = m_pTmpBuffer->m_tFmBytes * iNthGpu;
	char* pcBuf = pcPinnedBuf + tOffset;
	return pcBuf;
}

Util::CCufft2D* CBufferPool::GetForwardFFT(int iNthGpu)
{
	if(m_pCufft2Ds == 0L) return 0L;
	else return &m_pCufft2Ds[2 * iNthGpu];
}

Util::CCufft2D* CBufferPool::GetInverseFFT(int iNthGpu)
{
	if(m_pCufft2Ds == 0L) return 0L;
	else return &m_pCufft2Ds[2*iNthGpu+1];
}

void CBufferPool::SetDevice(int iNthGpu)
{
	if(iNthGpu >= m_iNumGpus) return;
	cudaSetDevice(m_piGpuIDs[iNthGpu]);
}

void CBufferPool::mCreateSumBuffer(void)
{
	m_iNumSums = 1;
	//-----------------
	Align::CAlignParam* pAlnParam = Align::CAlignParam::GetInstance();
        if(pAlnParam->SplitSum()) m_iNumSums += 2;
	//-----------------
	bool bSimpleSum = pAlnParam->SimpleSum();
	if(DU::CFmIntegrateParam::DoseWeight() && !bSimpleSum)
	{	m_iNumSums += 1;
		if(DU::CFmIntegrateParam::DWSelectedSum()) m_iNumSums += 1;
	}
	//-----------------
	int aiCmpSize[] = {m_aiStkSize[0] / 2 + 1, m_aiStkSize[1]};
	m_pSumBuffer = new CStackBuffer;
	int iNumFrames = m_iNumGpus * m_iNumSums;
	m_pSumBuffer->Create(aiCmpSize, iNumFrames, m_piGpuIDs, m_iNumGpus);
}

void CBufferPool::mCreateTmpBuffer(void)
{
	int iNumFrames = 4;
	iNumFrames *= m_iNumGpus;
	//-----------------------
	int iSizeX = (m_aiStkSize[0] > 4096) ? m_aiStkSize[0] : 4096;
	int iSizeY = (m_aiStkSize[1] > 4096) ? m_aiStkSize[1] : 4096;
	int aiCmpSize[] = {iSizeX / 2 + 1, iSizeY};
	//-----------------------------------------	
	m_pTmpBuffer = new CStackBuffer;
	m_pTmpBuffer->Create(aiCmpSize, iNumFrames, m_piGpuIDs, m_iNumGpus);
	//------------------------------------------------------------------
	size_t tBytes = m_pTmpBuffer->m_tFmBytes * m_iNumGpus;
	cudaMallocHost(&m_pvPinnedBuf, tBytes);
}

void CBufferPool::mCreateXcfBuffer(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iAlign == 0) return;
	//-------------------------------
	float fBinX = m_aiStkSize[0] / 2048.0f;
	float fBinY = m_aiStkSize[1] / 2048.0f;
	float fBin = fmin(fBinX, fBinY);
	if(fBin < 1) fBin = 1.0f;
	//-----------------------
	int iXcfX = (int)(m_aiStkSize[0] / fBin + 0.5f);
	int iXcfY = (int)(m_aiStkSize[1] / fBin + 0.5f);
	//---------------------------------------
	int aiXcfCmpSize[] = {iXcfX / 2 + 1, iXcfY / 2 * 2};
	m_afXcfBin[0] = (m_pSumBuffer->m_aiCmpSize[0] - 1.0f)
	   / (aiXcfCmpSize[0] - 1.0f);
	m_afXcfBin[1] = m_pSumBuffer->m_aiCmpSize[1]
	   * 1.0f / aiXcfCmpSize[1];
	//----------------------------------------------------
	m_pXcfBuffer = new CStackBuffer;
	m_pXcfBuffer->Create(aiXcfCmpSize, m_aiStkSize[2], 
	   m_piGpuIDs, m_iNumGpus);
}

void CBufferPool::mCreatePatBuffer(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_aiNumPatches[0] <= 1) return;
	if(pInput->m_aiNumPatches[1] <= 1) return;
	if(m_pXcfBuffer == 0L) return;
	//----------------------------
	int iXcfX = (m_pXcfBuffer->m_aiCmpSize[0] - 1) * 2;
	int iXcfY = m_pXcfBuffer->m_aiCmpSize[1];
	int iPatX = iXcfX / pInput->m_aiNumPatches[0];
	int iPatY = iXcfY / pInput->m_aiNumPatches[1];
	if(iPatX > (iXcfX / 2)) return;
	if(iPatY > (iXcfY / 2)) return;
	//-----------------------------
	int aiPatCmpSize[2] = {iPatX / 2 + 1, iPatY / 2 * 2};
	m_pPatBuffer = new CStackBuffer;
	m_pPatBuffer->Create(aiPatCmpSize, m_pXcfBuffer->m_iNumFrames,
	   m_piGpuIDs, m_iNumGpus);
}

void CBufferPool::mCreateFrmBuffer(void)
{
	m_pFrmBuffer = new CStackBuffer;
	int aiCmpSize[] = {m_aiStkSize[0] / 2 + 1, m_aiStkSize[1]};
	m_pFrmBuffer->Create(aiCmpSize, m_aiStkSize[2],
	m_piGpuIDs, m_iNumGpus);
}

