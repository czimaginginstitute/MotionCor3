#include "CMainInc.h"
#include "Util/CUtilInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <errno.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2;

CStackBuffer::CStackBuffer(void)
{
	m_pGpuBuffers = 0L;
}

CStackBuffer::~CStackBuffer(void)
{
	this->Clean();
}

void CStackBuffer::Clean(void)
{
	if(m_pGpuBuffers == 0L) return;
	delete[] m_pGpuBuffers;
	m_pGpuBuffers = 0L;
}

//-----------------------------------------------------------------------------
// CStackBuffer is responsible to distribute stack frames to the memories of
// all available GPUs. If not enough, CPU memory is used to buffer the
// remaining frames. Each GPU buffer is abstracted in CGpuBuffer that hosts
// a subset of frames. CPU buffer is also abstracted in CGpuBuffer with its
// ID set to -1.
//-----------------------------------------------------------------------------
void CStackBuffer::Create
(	int* piCmpSize,
	int iNumFrames,
	int* piGpuIDs, 
	int iNumGpus
)
{	this->Clean();
	//------------
	m_aiCmpSize[0] = piCmpSize[0];
	m_aiCmpSize[1] = piCmpSize[1];
	m_iNumFrames = iNumFrames;
	m_piGpuIDs = piGpuIDs;
	m_iNumGpus = iNumGpus;
	m_tFmBytes = sizeof(cufftComplex) * 
	   m_aiCmpSize[0] * m_aiCmpSize[1];
	//-----------------
	int* piFrmsPerGpu = mCalcFramesPerGpu(m_iNumFrames);
	m_pGpuBuffers = new CGpuBuffer[m_iNumGpus];
	for(int i=0; i<m_iNumGpus; i++)
	{	int iGpuID = piGpuIDs[i];
		m_pGpuBuffers[i].Create(m_tFmBytes, 
		   piFrmsPerGpu[i], iGpuID);
	}
	if(piFrmsPerGpu != 0L) delete[] piFrmsPerGpu;
}

void CStackBuffer::Adjust(int iNumFrames)
{
	if(iNumFrames == m_iNumFrames) return;
	else m_iNumFrames = iNumFrames;
	//----------------
	int* piFrmsPerGpu = mCalcFramesPerGpu(iNumFrames);
	for(int i=0; i<m_iNumGpus; i++)
	{	m_pGpuBuffers[i].AdjustBuffer(piFrmsPerGpu[i]);
	}
	if(piFrmsPerGpu != 0L) delete[] piFrmsPerGpu;	
}

int CStackBuffer::GetStartFrame(int iNthGpu)
{
	int iStartFrame = 0;
	for(int i=0; i<iNthGpu; i++)
	{	iStartFrame += m_pGpuBuffers[i].m_iNumFrames;
	}
	return iStartFrame;
}

int CStackBuffer::GetNumFrames(int iNthGpu)
{
	return m_pGpuBuffers[iNthGpu].m_iNumFrames;
}

bool CStackBuffer::IsGpuFrame(int iNthGpu, int iFrame)
{
	if(iFrame < m_pGpuBuffers[iNthGpu].m_iNumGpuFrames) return true;
	else return false;
}

cufftComplex* CStackBuffer::GetFrame(int iNthGpu, int iFrame)
{
	void* pvFrame = m_pGpuBuffers[iNthGpu].GetFrame(iFrame);
	cufftComplex* pCmpFrame = reinterpret_cast<cufftComplex*>(pvFrame);
	return pCmpFrame;
}

cufftComplex* CStackBuffer::GetFrame(int iAbsFrame)
{
	int iStartFrm = 0;
	for(int i=0; i<m_iNumGpus; i++)
	{	int iStartFrm = this->GetStartFrame(i);
		int iNumFrms = this->GetNumFrames(i);
		int iRefFrm = iAbsFrame - iStartFrm;
		if(iRefFrm < 0) continue;
		else if(iRefFrm >= iNumFrms) continue;
		else return this->GetFrame(i, iRefFrm);
	}
	return 0L;
}
		
int CStackBuffer::GetFrameGpu(int iFrame)
{
	int iStart = 0;
	for(int i=0; i<m_iNumGpus; i++)
	{	int j = iFrame - iStart;
		if(j < m_pGpuBuffers[i].m_iNumFrames) return i;
 		else iStart += m_pGpuBuffers[i].m_iNumFrames;
	}
	return -1;
}

void CStackBuffer::SetDevice(int iNthGpu)
{
	if(iNthGpu >= m_iNumGpus) return;
        cudaSetDevice(m_piGpuIDs[iNthGpu]);
}

int* CStackBuffer::mCalcFramesPerGpu(int iNumFrms)
{
	int iFrmsPerGpu = m_iNumFrames / m_iNumGpus;
	int iRemain = m_iNumFrames - m_iNumGpus * iFrmsPerGpu;
	int* piFrmsPerGpu = new int[m_iNumGpus];
	//-----------------
	for(int i=0; i<m_iNumGpus; i++)
	{	piFrmsPerGpu[i] = iFrmsPerGpu;
        }
	if(iRemain == 0) return piFrmsPerGpu;
	//-----------------
	for(int i=0; i<iRemain; i++)
	{	piFrmsPerGpu[i] += 1;
	}
	return piFrmsPerGpu;
}
