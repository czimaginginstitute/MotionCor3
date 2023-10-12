#include "CCorrectInc.h"
#include <memory.h>
#include <stdio.h>
#include <math.h>
#include <cuda.h>
#include <cuda_runtime.h>

using namespace MotionCor2;
using namespace MotionCor2::Correct;

//-------------------------------------------------------------------
// 1. The unpadded size x is stored in gridDim.x.
// 2. The padded size x is in iPadSizeX. If the image is unpadded,
//    then iPadSizeX = gridDim.x.
//-------------------------------------------------------------------
static __global__ void mGStretch
(	float* gfInFrm,
	float* gfMatrix,
	int iPadSizeX,
	int iSizeY,
	float* gfOutFrm
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if(y >= iSizeY) return;
	//---------------------
	float fCentX = 0.5f * gridDim.x;
	float fCentY = 0.5f * iSizeY;
	float fX = blockIdx.x - fCentX;
	float fY = y - fCentY;
	//--------------------
	float fOldX = fX * gfMatrix[0] + fY * gfMatrix[1] + fCentX;
	float fOldY = fX * gfMatrix[1] + fY * gfMatrix[2] + fCentY;
	if(fOldX < 1 || fOldY < 1 
	|| fOldX >= (gridDim.x - 1)
	|| fOldY >= (iSizeY - 1))
	{	fOldX = (8111 * (int)fabsf(fOldX)) % (gridDim.x - 1);
		fOldY = (8111 * (int)fabsf(fOldY)) % (iSizeY - 1);
	}
	//-----------------------------------------------------------
	int i = y * iPadSizeX + blockIdx.x;
	float* gfInPtr = gfInFrm + ((int)fOldY) * iPadSizeX + (int)fOldX;
	fX = fOldX - (int)fOldX; // weight to x+1
	fY = fOldY - (int)fOldY; // weight to y+1
        gfOutFrm[i] = (1 - fX) * (1 - fY) * gfInPtr[0]
	   + fX * (1 - fY) * gfInPtr[1]
	   + (1 - fX) * fY * gfInPtr[iPadSizeX]
	   + fX * fY * gfInPtr[iPadSizeX + 1];
}

GStretch::GStretch(void)
{
}

GStretch::~GStretch(void)
{
}

//-------------------------------------------------------------------
// Stretch image along the direction that has angle relative to the 
// x axis.
// 1. fStretch:     the amount of stretching is given by fStretch.
// 2. fStretchAxis: the angle of the stretching axis relative 
//    to the image x axis in degree.
//------------------------------------------------------------------- 
void GStretch::Setup(float fStretch, float fStretchAxis)
{
	double d2T = 2 * fStretchAxis * 3.1416 / 180;
	double dSin2T = sin(d2T);
	double dCos2T = cos(d2T);
	double dP = 0.5 * (fStretch + 1);
	double dM = 0.5 * (fStretch - 1);
	//-------------------------------
	float afMatrix[3] = {0.0f};
	afMatrix[0] = (float)(dP + dM * dCos2T);
	afMatrix[1] = (float)(1 * dM * dSin2T);
	afMatrix[2] = (float)(dP - dM * dCos2T);
	float fDet = afMatrix[0] * afMatrix[2] 
		- afMatrix[1] * afMatrix[1];
	//----------------------------------
	m_afMatrix[0] = afMatrix[2] / fDet;
	m_afMatrix[1] = -afMatrix[1] / fDet;
	m_afMatrix[2] = afMatrix[0] / fDet;
}

float* GStretch::GetMatrix(bool bGpu)
{
	float* pfMatrix = 0L;
	size_t tBytes = sizeof(m_afMatrix);
	//---------------------------------
	if(bGpu)
	{	pfMatrix = mCopyMatrixToGpu(m_afMatrix);
	}
	else
	{	pfMatrix = new float[3];
		memcpy(pfMatrix, m_afMatrix, tBytes);
	}
	return pfMatrix;
}

void GStretch::DoIt
(	float* gfInImg,
	bool bPadded,
	int* piImgSize,
	float* gfOutImg 
)
{	float* gfMatrix = mCopyMatrixToGpu(m_afMatrix);
	//---------------------------------------------
	int iSizeX = piImgSize[0];
	if(bPadded) iSizeX = (piImgSize[0] / 2 - 1) * 2;
	//----------------------------------------------
	dim3 aBlockDim(1, 512);
	dim3 aGridDim(iSizeX, 1);
	aGridDim.y = piImgSize[1] / aBlockDim.y + 1;
	mGStretch<<<aGridDim, aBlockDim>>>(gfInImg, gfMatrix, 
	   piImgSize[0], piImgSize[1], gfOutImg);
	//---------------------------------------------------
	cudaFree(gfMatrix); 
}

void GStretch::Unstretch(float* pfInShift, float* pfOutShift)
{
	float fX = pfInShift[0];
        float fY = pfInShift[1];
        pfOutShift[0] = fX * m_afMatrix[0] + fY * m_afMatrix[1];
        pfOutShift[1] = fX * m_afMatrix[1] + fY * m_afMatrix[2];
}

float* GStretch::mCopyMatrixToGpu(float* pfMatrix)
{	
	float* gfMatrix = 0L;
	int iBytes = sizeof(float) * 3;
	cudaMalloc(&gfMatrix, iBytes);
	cudaMemcpy(gfMatrix, m_afMatrix, iBytes, cudaMemcpyHostToDevice);
	return gfMatrix;
}

