#include "CUtilInc.h"
#include <memory.h>
#include <stdio.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cufft.h>

using namespace MotionCor2::Util;

CMultiGpuBase::CMultiGpuBase(void)
{
	m_pStreams = 0L;
	m_pForwardFFTs = 0L;
	m_pInverseFFTs = 0L;
	m_piGpuIDs = 0L;
	m_iNumGpus = 0;
}

CMultiGpuBase::~CMultiGpuBase(void)
{
	this->mCleanAll();
}

void CMultiGpuBase::mCleanAll(void)
{
	this->mDeleteForwardFFTs();
	this->mDeleteInverseFFTs();
	this->mDeleteStreams();
	if(m_piGpuIDs != 0L) delete[] m_piGpuIDs;
	m_piGpuIDs = 0L;
}

void CMultiGpuBase::mSetGpus(int* piGpuIDs, int iNumGpus)
{
	this->mCleanAll();
	//----------------
	m_piGpuIDs = new int[iNumGpus];
	memcpy(m_piGpuIDs, piGpuIDs, sizeof(int) * iNumGpus);
	m_iNumGpus = iNumGpus;
}

void CMultiGpuBase::mCreateStreams(int iStreamsPerGpu)
{
	if(m_iNumGpus <= 0) return;
	m_iStreamsPerGpu = iStreamsPerGpu;
	//--------------------------------
	m_pStreams = new cudaStream_t[m_iStreamsPerGpu * m_iNumGpus];
	for(int i=0; i<m_iNumGpus; i++)
	{	cudaSetDevice(m_piGpuIDs[i]);
		int iStart = m_iStreamsPerGpu * i;
		for(int j=0; j<m_iStreamsPerGpu; j++)
		{	cudaStream_t stream = 0;
			cudaStreamCreate(&stream);
			m_pStreams[iStart+j] = stream;
		}
	}
}

void CMultiGpuBase::mDeleteStreams(void)
{
	if(m_pStreams == 0L || m_piGpuIDs == 0L) return;
	//----------------------------------------------
	for(int i=0; i<m_iNumGpus; i++)
	{	cudaSetDevice(m_piGpuIDs[i]);
		int iStart = m_iStreamsPerGpu * i;
		for(int j=0; j<m_iStreamsPerGpu; j++)
		{	int iStream = iStart + j;
			cudaStreamSynchronize(m_pStreams[iStream]);
			cudaStreamDestroy(m_pStreams[iStream]);
		}
	}
	//-----------------------------------------------------
	delete[] m_pStreams;
	m_pStreams = 0L;
}

void CMultiGpuBase::mCreateForwardFFTs(int* piSize, bool bPad)
{
	if(m_iNumGpus <= 0) return;
	//-------------------------
	m_pForwardFFTs = new CCufft2D[m_iNumGpus];
	for(int i=0; i<m_iNumGpus; i++)
	{	cudaSetDevice(m_piGpuIDs[i]);
		m_pForwardFFTs[i].CreateForwardPlan(piSize, bPad);
	}
}

void CMultiGpuBase::mDeleteForwardFFTs(void)
{
	if(m_pForwardFFTs == 0L) return;
	for(int i=0; i<m_iNumGpus; i++)
	{	cudaSetDevice(m_piGpuIDs[i]);
		m_pForwardFFTs[i].DestroyPlan();
	}
	delete[] m_pForwardFFTs;
	m_pForwardFFTs = 0L;
}

void CMultiGpuBase::mCreateInverseFFTs(int* piSize, bool bCmp)
{
	if(m_iNumGpus <= 0) return;
	//-------------------------
	m_pInverseFFTs = new CCufft2D[m_iNumGpus];
	for(int i=0; i<m_iNumGpus; i++)
	{	cudaSetDevice(m_piGpuIDs[i]);
		m_pInverseFFTs[i].CreateInversePlan(piSize, bCmp);
	}
}

void CMultiGpuBase::mDeleteInverseFFTs(void)
{
	if(m_pInverseFFTs == 0L) return;
	for(int i=0; i<m_iNumGpus; i++)
	{	cudaSetDevice(m_piGpuIDs[i]);
		m_pInverseFFTs[i].DestroyPlan();
	}
	delete[] m_pInverseFFTs;
	m_pInverseFFTs = 0L;
}
