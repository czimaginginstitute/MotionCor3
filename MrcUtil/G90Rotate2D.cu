#include "CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <math.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

//-------------------------------------------------------------------
// 1. Rotate image only a factor of 90 degrees.
// 2. This is why we use nearest neighbor value to ensure 
//    precise rotation.
// 3. Rotation center is at [(Nx - 1) * 0.5, (Ny - 1) * 0.5]
//-------------------------------------------------------------------
static __global__ void mGRotate
(	float* gfImg,
	int* giImgSize,
	int* giCosSin,
	float* gfRotImg,
	int iRotSizeX
)
{       int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iRotSizeX) return;
	int i = blockIdx.y * iRotSizeX + x;
        //---------------------------------
	float fNewX = x - (iRotSizeX - 1) * 0.5f;
	float fNewY = blockIdx.y - (gridDim.y - 1) * 0.5f;
	float fX = fNewX * giCosSin[0] + fNewY * giCosSin[1];
	float fY = -fNewX * giCosSin[1] + fNewY * giCosSin[0];
	int iX = (int)(fX + (giImgSize[0] - 1) * 0.5f + 0.5f);
	int iY = (int)(fY + (giImgSize[1] - 1) * 0.5f + 0.5f);
	int j = iY * giImgSize[0] + iX;
	gfRotImg[i] = gfImg[j];
}

G90Rotate2D::G90Rotate2D(void)
{
	m_gfRotImg = 0L;
	m_iImgBytes = 0;
}

G90Rotate2D::~G90Rotate2D(void)
{
	if(m_gfRotImg != 0L) cudaFree(m_gfRotImg);
}

float* G90Rotate2D::GetRotImg(bool bClean)
{
	float* gfRotImg = m_gfRotImg;
	if(bClean) m_gfRotImg = 0L;
	return gfRotImg;
}

void G90Rotate2D::GetRotImg(float* pfRotImg, bool bGpu)
{
	if(m_gfRotImg == 0L) return;
	if(pfRotImg == 0L) return;
	//------------------------
	cudaMemcpyKind aD2D = cudaMemcpyDeviceToDevice;
	cudaMemcpyKind aD2H = cudaMemcpyDeviceToHost;
	if(bGpu) cudaMemcpy(pfRotImg, m_gfRotImg, m_iImgBytes, aD2D);
	else cudaMemcpy(pfRotImg, m_gfRotImg, m_iImgBytes, aD2H);
}

void G90Rotate2D::GetRotSize(int* piSize, int iRotFactor, int* piRotSize)
{
     if(iRotFactor %2 == 0)
     {    piRotSize[0] = piSize[0];
          piRotSize[1] = piSize[1];
     }
     else
     {    piRotSize[0] = piSize[1];
          piRotSize[1] = piSize[0];
     }
}

void G90Rotate2D::Setup(int* piImgSize, int iRotFactor)
{
	m_aiImgSize[0] = piImgSize[0];
	m_aiImgSize[1] = piImgSize[1];
	this->GetRotSize(m_aiImgSize, iRotFactor, m_aiRotSize);
	//-----------------------------------------------------
	double dAngle = atan(1.0) * 4 / 180.0 * 90.0 * iRotFactor;
	float fCos = cos(dAngle);
	float fSin = sin(dAngle);
	m_aiCosSin[0] = (fCos > 0) ? (int)(fCos + 0.1) : (int)(fCos - 0.1);
	m_aiCosSin[1] = (fSin > 0) ? (int)(fSin + 0.1) : (int)(fSin - 0.1);
	//-----------------------------------------------------------------
	m_iImgBytes = sizeof(float) * m_aiRotSize[0] * m_aiRotSize[1];
	if(m_gfRotImg != 0L) cudaFree(m_gfRotImg);
	cudaMalloc(&m_gfRotImg, m_iImgBytes);	
}

//-------------------------------------------------------------------
// 1. iRotFactor is the factor of 90 degrees.
// 2. The rotated image will be written back into pfImg.
//-------------------------------------------------------------------
void G90Rotate2D::DoIt
(	float* pfImg,
	bool bGpu
)
{	float* gfImg = pfImg;
	if(!bGpu) gfImg = (float*)mCopyToDevice(pfImg, m_iImgBytes);
	if(m_gfRotImg == 0L) cudaMalloc(&m_gfRotImg, m_iImgBytes);
	//--------------------------------------------------------
	int* giImgSize = (int*)mCopyToDevice
	(  m_aiImgSize, sizeof(m_aiImgSize)
	);
	int* giCosSin = (int*)mCopyToDevice
	(  m_aiCosSin, sizeof(m_aiCosSin)
	);
	//-------------------------------
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(1, m_aiRotSize[1]);
	aGridDim.x = m_aiRotSize[0] / aBlockDim.x + 1;
	mGRotate<<<aGridDim, aBlockDim>>>
	(  gfImg, giImgSize, giCosSin,
	   m_gfRotImg, m_aiRotSize[0]
	);
	//---------------------------
	if(giImgSize != 0L) cudaFree(giImgSize);
	if(giCosSin != 0L) cudaFree(giCosSin);
	if(!bGpu && gfImg != 0L) cudaFree(gfImg);
}	

void* G90Rotate2D::mCopyToDevice(void* pvData, int iBytes)
{
	void* gvBuf = 0L;
	cudaMalloc(&gvBuf, iBytes);
	cudaMemcpy(gvBuf, pvData, iBytes, cudaMemcpyHostToDevice);
	return gvBuf;
}

