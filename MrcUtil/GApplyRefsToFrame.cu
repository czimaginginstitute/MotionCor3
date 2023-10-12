#include "CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

static __device__ __constant__ int giMrcSize[2];

//-------------------------------------------------------------------
// 1. The gain-corrected frame could be padded for FFT. In this
//    case iPadX = (iSizeX / 2 + 1) * 2. Otherwise iPadX = iSizeX.
//-------------------------------------------------------------------
static __global__ void mGUnpack
(	unsigned char* pucPkdFrm,
	unsigned char* pucRawFrm,
	int iSizeX
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	//---------------------
	int i = blockIdx.y * ((iSizeX + 1) / 2) + x / 2;
	i = pucPkdFrm[i];
	if((x % 2) == 1) i = (i >> 4);
	//----------------------------
	pucRawFrm[blockIdx.y * iSizeX + x] = (i & 0xf);
}

static __global__ void mGDoPkd
(	unsigned char* gucPkdFrm,
	float* gfGain,
	float* gfDark,
	float* gfFrame,// frame may be cropped so its size can
	int iSizeX,    // smaller thant ref size in both x &
	int iPadX     // y dimensions.
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	//---------------------
	int xRef = (giMrcSize[0] - iSizeX) / 2 + x;
	int yRef = (giMrcSize[1] - gridDim.y) / 2 + blockIdx.y;
	//-----------------------------------------------------
	yRef = gucPkdFrm[yRef * ((giMrcSize[0] + 1) / 2) + xRef / 2];
	if((xRef % 2) == 1) yRef = (yRef >> 4);
	float fVal = (float)(yRef & 0xf);
	//-------------------------------
	int i = blockIdx.y * iPadX + x;
	if(gfDark != 0L) fVal -= gfDark[i];
	if(gfGain != 0L) fVal *= gfGain[i];
	//---------------------------------
	gfFrame[blockIdx.y * iPadX + x] = fVal;
}

template <typename T>
static __global__ void mGDoRaw
(	T* gtFrame,
	float* gfGain,
	float* gfDark,
	float* gfFrame,
	int iSizeX,
	int iPadX
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	float fVal = (float)gtFrame[blockIdx.y * giMrcSize[0] + x];
	//---------------------------------------------------------
	x = blockIdx.y * iPadX + x;
	if(gfDark != 0L) fVal -= gfDark[x];
	if(gfGain != 0L) fVal *= gfGain[x];
	gfFrame[x] = fVal;
}

GApplyRefsToFrame::GApplyRefsToFrame(void)
{
}

GApplyRefsToFrame::~GApplyRefsToFrame(void)
{
}

void GApplyRefsToFrame::SetRefs
(	float* gfGain,
	float* gfDark
)
{	m_gfGain = gfGain;
	m_gfDark = gfDark;
}

void GApplyRefsToFrame::SetSizes
(	int* piMrcSize, 
	int* piFrmSize, 
	bool bFrmPadded
)
{	m_aiMrcSize[0] = piMrcSize[0];
	m_aiMrcSize[1] = piMrcSize[1];
	cudaMemcpyToSymbol(giMrcSize, m_aiMrcSize, sizeof(int) * 2);
	Util::CheckCudaError("GApplyRefsToFrame: 1111");
	//----------------------------------------------------------
	m_aiFrmSize[0] = piFrmSize[0];
	m_aiFrmSize[1] = piFrmSize[1];
	m_iPadSizeX = m_aiFrmSize[0];
	//---------------------------
	if(bFrmPadded) 
	{	m_aiFrmSize[0] = (piFrmSize[0] / 2 - 1) * 2;
		m_iPadSizeX = (m_aiFrmSize[0] / 2 + 1) * 2;
	}
	//-------------------------------------------------
	int iStartX = (m_aiMrcSize[0] - m_aiFrmSize[0]) / 2;
	int iStartY = (m_aiMrcSize[1] - m_aiFrmSize[1]) / 2;
	m_iMrcOffset = iStartY * m_aiMrcSize[0] + iStartX;
}

void GApplyRefsToFrame::Unpack
(	unsigned char* gucPkdFrm,
	unsigned char* gucRawFrm,
	int* piFrmSize,
	cudaStream_t stream
)
{	dim3 aBlockDim(512, 1);
	int iGridX = (piFrmSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	dim3 aGridDim(iGridX, piFrmSize[1]);
	//----------------------------------
	mGUnpack<<<aGridDim, aBlockDim, 0, stream>>>
	( gucPkdFrm, gucRawFrm, piFrmSize[0]
	);
}

void GApplyRefsToFrame::DoIt
(	void* gvFrame,
	int iMrcMode,
	float* gfFrame,
	cudaStream_t stream
)
{	if(iMrcMode == Mrc::eMrcUChar || iMrcMode == Mrc::eMrcUCharEM)
	{	this->DoRaw((unsigned char*)gvFrame, gfFrame, stream);
	}
	else if(iMrcMode == Mrc::eMrc4Bits)
	{	this->DoPkd((unsigned char*)gvFrame, gfFrame, stream);
	}
	else if(iMrcMode == Mrc::eMrcShort)
	{	this->DoShort((short*)gvFrame, gfFrame, stream);
	}
	else if(iMrcMode == Mrc::eMrcUShort)
	{	this->DoUShort((unsigned short*)gvFrame, gfFrame, stream);
	}
	else if(iMrcMode == Mrc::eMrcFloat)
	{	this->DoFloat((float*)gvFrame, gfFrame, stream); 
	}
}

void GApplyRefsToFrame::DoRaw
(	unsigned char* gucRawFrm,
	float* gfFrame,
	cudaStream_t stream
)
{	dim3 aBlockDim(512, 1);
	int iGridX = (m_aiFrmSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	dim3 aGridDim(iGridX, m_aiFrmSize[1]);
	//------------------------------------
	unsigned char* gucSrcFrm = gucRawFrm + m_iMrcOffset;
	mGDoRaw<<<aGridDim, aBlockDim, 0, stream>>>
	( gucSrcFrm, m_gfGain, m_gfDark, gfFrame,
	  m_aiFrmSize[0], m_iPadSizeX );
}

void GApplyRefsToFrame::DoPkd
(	unsigned char* gucPkdFrm,
	float* gfFrame,
	cudaStream_t stream
)
{	dim3 aBlockDim(512, 1);
	int iGridX = (m_aiFrmSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	dim3 aGridDim(iGridX, m_aiFrmSize[1]);
	//------------------------------------
	mGDoPkd<<<aGridDim, aBlockDim, 0, stream>>>
	( gucPkdFrm, m_gfGain, m_gfDark, gfFrame, 
	  m_aiFrmSize[0], m_iPadSizeX );
}

void GApplyRefsToFrame::DoShort
(	short* gsFrm,
	float* gfFrame,
	cudaStream_t stream
)
{	dim3 aBlockDim(512, 1);
	int iGridX = (m_aiFrmSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	dim3 aGridDim(iGridX, m_aiFrmSize[1]);
	//------------------------------------
	short* gsSrcFrm = gsFrm + m_iMrcOffset;
	mGDoRaw<<<aGridDim, aBlockDim, 0, stream>>>
	( gsSrcFrm, m_gfGain, m_gfDark, gfFrame,
	  m_aiFrmSize[0], m_iPadSizeX );
}

void GApplyRefsToFrame::DoUShort
(  	unsigned short* gusFrm,
   	float* gfFrame,
	cudaStream_t stream
)
{	dim3 aBlockDim(512, 1);
	int iGridX = (m_aiFrmSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	dim3 aGridDim(iGridX, m_aiFrmSize[1]);
	//------------------------------------
	unsigned short* gusSrcFrm = gusFrm + m_iMrcOffset;
	mGDoRaw<<<aGridDim, aBlockDim, 0, stream>>>
	( gusSrcFrm, m_gfGain, m_gfDark, gfFrame,
	  m_aiFrmSize[0], m_iPadSizeX );
}

void GApplyRefsToFrame::DoFloat
(	float* gfInFrm,
	float* gfOutFrm,
	cudaStream_t stream
)
{	dim3 aBlockDim(512, 1);
	int iGridX = (m_aiFrmSize[0] + aBlockDim.x - 1) / aBlockDim.x;
	dim3 aGridDim(iGridX, m_aiFrmSize[1]);
	//------------------------------------
	float* gfSrcFrm = gfInFrm + m_iMrcOffset;
	mGDoRaw<<<aGridDim, aBlockDim, 0, stream>>>
	( gfSrcFrm, m_gfGain, m_gfDark, gfOutFrm,
	  m_aiFrmSize[0], m_iPadSizeX );
}

