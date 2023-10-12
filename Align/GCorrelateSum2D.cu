#include "CAlignInc.h"
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <nvToolsExt.h>

using namespace MotionCor2::Align;
using namespace MotionCor2::Util;

#define OPT_NVIDIA

static __global__ void mGConv
(	cufftComplex* gCmpSum, 
	cufftComplex* gCmpXcf,
	int iCmpY,
	bool bSubtract
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
        if (y >= iCmpY) return;
	int i = y * gridDim.x + blockIdx.x;
	//---------------------------------
	cufftComplex cSum = gCmpSum[i];
	cufftComplex cXcf = gCmpXcf[i];
	if(bSubtract)
	{	cSum.x -= cXcf.x;
		cSum.y -= cXcf.y;
	}
	//-----------------------
	gCmpXcf[i].x = cXcf.x * cSum.x + cXcf.y * cSum.y;
	gCmpXcf[i].y = cXcf.x * cSum.y - cXcf.y * cSum.x;
}

static __global__ void mGLowpass
(	cufftComplex* gCmpXcf, 
	int iCmpY, 
	float fBFactor,
	bool bPhaseOnly
)
{	int y = blockIdx.y * blockDim.y + threadIdx.y;
        if(y >= iCmpY) return;
	int i = y * gridDim.x + blockIdx.x;
	//---------------------------------
	if(y > (iCmpY / 2)) y -= iCmpY;
	float fFilt = -0.25f * fBFactor /((gridDim.x - 1) * iCmpY);
	fFilt = expf(fFilt * (blockIdx.x * blockIdx.x + y * y));
	//------------------------------------------------------
	if(bPhaseOnly)
	{	float fAmp = sqrtf(gCmpXcf[i].x * gCmpXcf[i].x
			+ gCmpXcf[i].y * gCmpXcf[i].y);
		fFilt = fFilt / (fAmp + (float)1e-20);
	}
	//--------------------------------------------
        if (i > 0) 
	{	gCmpXcf[i].x *= fFilt;
		gCmpXcf[i].y *= fFilt;
        }
        else 
	{	gCmpXcf[0].x = 0.0f; // For K2 stacks  04/27/2018
		gCmpXcf[0].y = 0.0f; // For K3 stacks, 04/27/2018
        }
}

static __global__ void mGCenterOrigin(cufftComplex* gCmpXcf, int iCmpY)
{
	int y = blockIdx.y * blockDim.y + threadIdx.y;
	if (y >= iCmpY) return;
	int i = y * gridDim.x + blockIdx.x;
	//---------------------------------
	int iSign = ((blockIdx.x + y) % 2 == 0) ? 1 : -1;
	gCmpXcf[i].x *= iSign;
	gCmpXcf[i].y *= iSign;
}

GCorrelateSum2D::GCorrelateSum2D(void)
{
	m_fBFactor = 200.0f;
	m_bPhaseOnly = false;
	m_aiSeaSize[0] = 64;
	m_aiSeaSize[1] = 64;
	m_bSubtract = true;
}

GCorrelateSum2D::~GCorrelateSum2D(void)
{
}

void GCorrelateSum2D::SetFilter(float fBFactor, bool bPhaseOnly)
{
	m_fBFactor = fBFactor;
	m_bPhaseOnly = bPhaseOnly;
} 

void GCorrelateSum2D::SetSize(int* piCmpSize, int* piSeaSize)
{
	m_aiCmpSize[0] = piCmpSize[0];
	m_aiCmpSize[1] = piCmpSize[1];
	//----------------------------
	m_aiSeaSize[0] = piSeaSize[0];
	m_aiSeaSize[1] = piSeaSize[1];
}

void GCorrelateSum2D::SetSubtract(bool bSubtract)
{
	m_bSubtract = bSubtract;
}	

void GCorrelateSum2D::DoIt
(	cufftComplex* gCmpSum, 
	cufftComplex* gCmpXcf,
	float* pfPinnedXcf,
	Util::CCufft2D* pInverseFFT,
	cudaStream_t stream
)
{	nvtxRangePushA ("GCorrelateSum2D");
	dim3 aBlockDim(1, 64, 1);
        dim3 aGridDim(m_aiCmpSize[0], 1, 1);
	aGridDim.y = (m_aiCmpSize[1] + aBlockDim.y - 1) / aBlockDim.y;
	//------------------------------------------------------------
	mGConv<<<aGridDim, aBlockDim>>>(gCmpSum, gCmpXcf, 
	   m_aiCmpSize[1], m_bSubtract);
	mGLowpass<<<aGridDim, aBlockDim, 0, stream>>>(gCmpXcf, 
	   m_aiCmpSize[1], m_fBFactor, m_bPhaseOnly);
	mGCenterOrigin<<<aGridDim, aBlockDim, 0, stream>>>(gCmpXcf, 
	   m_aiCmpSize[1]);
	//---------------------------------------------------------
        pInverseFFT->Inverse(gCmpXcf, stream);
        nvtxRangePop();
	//-------------
	int iPadX = m_aiCmpSize[0] * 2;
	int iImgX = (m_aiCmpSize[0] - 1) * 2;
	int iStartX = (iImgX - m_aiSeaSize[0]) / 2;
	int iStartY = (m_aiCmpSize[1] - m_aiSeaSize[1]) / 2;
	//--------------------------------------------------
	float* gfPadXcf = reinterpret_cast<float*>(gCmpXcf);
	float* gfSrc = gfPadXcf + iStartY * iPadX + iStartX;
	//--------------------------------------------------
	Util::GPartialCopy::DoIt(gfSrc, iPadX, pfPinnedXcf, 
	   m_aiSeaSize[0], m_aiSeaSize, stream);
}

