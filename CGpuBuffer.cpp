#include "CMainInc.h"
#include <Util/Util_Time.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <errno.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <nvToolsExt.h>

using namespace MotionCor2;

CGpuBuffer::CGpuBuffer(void)
{
	m_pvGpuFrames = 0L;
	m_ppvCpuFrames = 0L;
	m_iNumFrames = 0;    // <= m_iMaxAllFrms;
	m_iNumGpuFrames = 0; // <= m_iMaxGpuFrms;
	m_iGpuID = -1;       // use pinned cpu memory
	//-----------------
	m_iMaxGpuFrms = 0;
	m_iMaxCpuFrms = 0;
}

CGpuBuffer::~CGpuBuffer(void)
{
	this->Clean();
}

void CGpuBuffer::Clean(void)
{
	if(m_pvGpuFrames == 0L && m_ppvCpuFrames == 0L) return;
	cudaSetDevice(m_iGpuID);
	cudaDeviceSynchronize();
	//----------------------
	if(m_pvGpuFrames != 0L) 
	{	cudaFree(m_pvGpuFrames);
		m_pvGpuFrames = 0L;
	}
	if(m_ppvCpuFrames != 0L) 
	{	for(int i=0; i<m_iMaxCpuFrms; i++)
		{	cudaFreeHost(m_ppvCpuFrames[i]);
		}
		delete[] m_ppvCpuFrames;
		m_ppvCpuFrames = 0L;
	}
	//-----------------
	m_iNumGpuFrames = 0;
	m_iNumFrames = 0;
	m_iMaxGpuFrms = 0;
	m_iMaxCpuFrms = 0;
}

void CGpuBuffer::Create(size_t tFrmBytes, int iNumFrames, int iGpuID)
{
	this->Clean();
	//------------
	m_tFmBytes = tFrmBytes;
	m_iNumFrames = iNumFrames;
	m_iGpuID = iGpuID;
	if(m_iNumFrames <= 0) return;
	//---------------------------
	cudaSetDevice(m_iGpuID);
	mCalcGpuFrames();
	//---------------
	nvtxRangePushA("CGpuBuffer: allocate GPU memory");
	Util_Time utilTime;
	float afTimes[2] = {0.0f}, afGBs[2] = {0.0f};
	size_t tBytes = m_iNumGpuFrames * m_tFmBytes;
	afGBs[0] = (float)(tBytes / (1024.0 * 1024.0 * 1024.0));
	if(tBytes > 0) 
	{	utilTime.Measure();
		cudaMalloc(&m_pvGpuFrames, tBytes);
		afTimes[0] = utilTime.GetElapsedSeconds();
	}
	nvtxRangePop();
	if(m_iNumGpuFrames >= m_iNumFrames) 
	{	mPrintAllocTimes(afGBs, afTimes);	
		return;
	}
	//-----------------
	nvtxRangePushA("CGpuBuffer: allocate pinned memory");
	utilTime.Measure();
	int iCpuFrms = m_iNumFrames - m_iNumGpuFrames;
	mCreateCpuBuf(iCpuFrms);
	afTimes[1] = utilTime.GetElapsedSeconds();
	nvtxRangePop();
	//-----------------
	tBytes = m_iMaxCpuFrms * m_tFmBytes;
	afGBs[1] = (float)(tBytes / (1024.0f * 1024.0 * 1024.0));
	mPrintAllocTimes(afGBs, afTimes);
}

void CGpuBuffer::AdjustBuffer(int iNumFrames)
{
	if(iNumFrames == m_iNumFrames) return;
	//-----------------------------------------------------------
	// If the new stack can be fit entirely int the existing
	// GPU frame buffer, then simply update GPU frame numbers. 
	//-----------------------------------------------------------
	if(iNumFrames <= m_iMaxGpuFrms)
	{	m_iNumGpuFrames = iNumFrames;
		m_iNumFrames = iNumFrames;
		return;
	}
	//----------------------------------------------------------
	// 1. In this case, use all GPU frame buffer. The left over
	//    is placed into the pinned frame buffer.
	// 2. mCreateCpuBuf can create or expand the pinned buffer
	//    as needed.
	//----------------------------------------------------------
	m_iNumGpuFrames = m_iMaxGpuFrms;
	int iCpuFrms = iNumFrames - m_iNumGpuFrames;
	mCreateCpuBuf(iCpuFrms);
	m_iNumFrames = iNumFrames;
}

void* CGpuBuffer::GetFrame(int iFrame)
{
	if(m_pvGpuFrames == 0L && m_ppvCpuFrames == 0L) return 0L;
	if(iFrame < 0 || iFrame >= m_iNumFrames) return 0L;
	//-------------------------------------------------
	if(iFrame < m_iNumGpuFrames)
	{	char* gcFrames = reinterpret_cast<char*>(m_pvGpuFrames);
		void* gvFrame = gcFrames + iFrame * m_tFmBytes;
		return gvFrame;
	}
	else
	{	return m_ppvCpuFrames[iFrame - m_iNumGpuFrames];
	}
}

void CGpuBuffer::mCalcGpuFrames(void)
{
	cudaSetDevice(m_iGpuID);
	size_t tTotal = 0, tFree = 0;
	cudaMemGetInfo(&tFree, &tTotal);
	//------------------------------
	CInput* pInput = CInput::GetInstance();
	float fReserve = 1.0f - pInput->m_fGpuMemUsage;
	if(fReserve < 0.1f) fReserve = 0.1f;
	//----------------------------------
	size_t t3GB = 1024 * 1024 * (size_t)(1024 * 3);
	size_t tReserve = (size_t)(fReserve * tTotal);
	if(tReserve < t3GB) tReserve = t3GB;
	//----------------------------------
	if(tFree <= tReserve) m_iNumGpuFrames = 0;
	else m_iNumGpuFrames = (tFree - tReserve) / m_tFmBytes;
	if(m_iNumGpuFrames > m_iNumFrames) m_iNumGpuFrames = m_iNumFrames;
	//-----------------
	m_iMaxGpuFrms = m_iNumGpuFrames;
}

void CGpuBuffer::mCreateCpuBuf(int iNumFrms)
{
	if(iNumFrms <= 0) return;
	if(iNumFrms <= m_iMaxCpuFrms) return;
	//-----------------
	void** ppvFrames = new void*[iNumFrms];
	for(int i=0; i<m_iMaxCpuFrms; i++)
	{	ppvFrames[i] = m_ppvCpuFrames[i];
		m_ppvCpuFrames[i] = 0L;
	}
	//-----------------
	for(int i=m_iMaxCpuFrms; i<iNumFrms; i++)
        {       void* pvFrm = 0L;
                cudaMallocHost(&pvFrm, m_tFmBytes);
                ppvFrames[i] = pvFrm;
        }
	//-----------------
	if(m_ppvCpuFrames != 0L) delete[] m_ppvCpuFrames;
	m_ppvCpuFrames = ppvFrames;
	m_iMaxCpuFrms = iNumFrms;
}

void CGpuBuffer::mPrintAllocTimes(float* pfGBs, float* pfTimes)
{
	printf("GPU %d Allocation time: GPU (%6.2f GB) %6.2f s, "
               "CPU (%6.2f GB) %6.2f s\n", m_iGpuID, pfGBs[0], pfTimes[0],
               pfGBs[1], pfTimes[1]);
}	
