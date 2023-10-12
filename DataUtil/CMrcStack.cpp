#include "CDataUtilInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2::DataUtil;

CMrcStack::CMrcStack(void)
{
	memset(m_aiStkSize, 0, sizeof(m_aiStkSize));
	m_ppvFrames = 0L;
	m_iMode = -1;
	m_tFmBytes = 0;
	m_iBufSize = 0;
	//------------------------------------------
	m_iAcqIndex = 0;
	m_iStack = 0;
	m_fPixSize = 1.0f;
	memset(m_afExt, 0, sizeof(m_afExt));
	m_iNumFloats = sizeof(m_afExt) / sizeof(float);
}

CMrcStack::~CMrcStack(void)
{
	this->DeleteFrames();
}

void CMrcStack::Create(int iMode, int* piStkSize)
{
	//----------------------------------------------------------------
	// reallocate only when the frame byte is changed.
	//----------------------------------------------------------------	
	size_t tFmBytes = Mrc::C4BitImage::GetImgBytes(iMode, piStkSize);
	if(tFmBytes == m_tFmBytes && m_aiStkSize[2] >= piStkSize[2])
	{	m_iBufSize = m_aiStkSize[2];
		memcpy(m_aiStkSize, piStkSize, sizeof(m_aiStkSize));
		m_iMode = iMode;
		return;
	}
	//----------------------------------------------------------
	this->DeleteFrames();
	m_iMode = iMode;
	m_iBufSize = piStkSize[2];
	memcpy(m_aiStkSize, piStkSize, sizeof(int) * 3);
	m_tFmBytes = Mrc::C4BitImage::GetImgBytes(m_iMode, m_aiStkSize);
	//--------------------------------------------------------------
	m_ppvFrames = new void*[m_iBufSize];
	for(int i=0; i<m_iBufSize; i++)
	{	void* pvFrame = 0L;
		pvFrame = new char[m_tFmBytes];
		m_ppvFrames[i] = pvFrame;
	}
}

void CMrcStack::Create(int iMode, int* piFmSize, int iNumFms)
{
	int aiStkSize[] = {piFmSize[0], piFmSize[1], iNumFms};
	this->Create(iMode, aiStkSize);
}

void CMrcStack::DeleteFrame(int iFrame)
{
	if(m_ppvFrames == 0L) return;
	if(m_ppvFrames[iFrame] == 0L) return;
	//cudaFreeHost(m_ppvFrames[iFrame]);
	delete[] (char*)m_ppvFrames[iFrame];
	m_ppvFrames[iFrame] = 0L;
}

void CMrcStack::DeleteFrames(void)
{
	if(m_ppvFrames == 0L) return;
	for(int i=0; i<m_iBufSize; i++)
	{	//cudaFreeHost(m_ppvFrames[i]);
		delete[] (char*)m_ppvFrames[i];
	}
	if(m_ppvFrames != 0L) delete[] m_ppvFrames;
	m_ppvFrames = 0L;
	m_iBufSize = 0;
	m_tFmBytes = 0;
}

void* CMrcStack::GetFrame(int iFrame)
{
	if(m_ppvFrames == 0L) return 0L;
	else if(iFrame >= m_aiStkSize[2]) return 0L;
	//------------------------------------------
	size_t tBytes = m_tFmBytes * iFrame;
	char* pcFrame = reinterpret_cast<char*>(m_ppvFrames[iFrame]);
	return pcFrame;
}

int CMrcStack::GetPixels(void)
{
	int iPixels = m_aiStkSize[0] * m_aiStkSize[1];
	return iPixels;
}

size_t CMrcStack::GetVoxels(void)
{
	size_t tVoxels = m_aiStkSize[0] * m_aiStkSize[1];
	tVoxels *= m_aiStkSize[2];
	return tVoxels;
}

float CMrcStack::GetTiltA(void)
{
	return m_afExt[0];
}

float CMrcStack::GetPixelSize(void)
{
	return m_afExt[11];
}

void CMrcStack::GetTomoShift(float* pfShift)
{
	if(pfShift == 0L) return;
	if(m_iNumFloats < 7) return;
	pfShift[0] = m_afExt[5];
	pfShift[1] = m_afExt[6];
}
