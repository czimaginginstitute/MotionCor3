#include "CMrcUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <memory.h>
#include <stdio.h>

using namespace MotionCor2::MrcUtil;

//-------------------------------------------------------------------
// 1. The gain-corrected frame could be padded for FFT. In this
//    case iPadX = (iSizeX / 2 + 1) * 2. Otherwise iPadX = iSizeX.
//-------------------------------------------------------------------
static __global__ void mGUnpack
(	unsigned char* pucPkdFrm,
	unsigned char* pucRawFrm,
	int iSizeX,
	int iPadX
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	int iPad = blockIdx.y * iPadX + x;
	int iPkd = blockIdx.y * ((iSizeX + 1) / 2) + x / 2;
	//-------------------------------------------------
	int iShift = ((x % 2) == 0) ? 0 : 4;
	pucRawFrm[iPad] = (pucPkdFrm[iPkd] >> iShift) & 0xf;
}

static __global__ void mGDoRaw
(	unsigned char* gucFrame,
	float* gfGain,
	float* gfFrame,
	int iSizeX,
	int iPadX
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
	if(x >= iSizeX) return;
	int iPad = blockIdx.y * iPadX + x;
	int iSize = blockIdx.y * iSizeX + x;
	//----------------------------------
	gfFrame[iPad] = gucFrame[iSize]; 
	if(gfGain == 0L) return;
	//----------------------
	gfFrame[iPad] *= gfGain[iSize];
}

static __global__ void mGDoPkd
(	unsigned char* gucPkdFrm,
	float* gfGain,
	float* gfFrame,
	int iSizeX,
	int iPadX
)
{	int x = blockIdx.x * blockDim.x + threadIdx.x;
     if(x >= iSizeX) return;
     int iPad = blockIdx.y * iPadX + x;
     int iPkd = blockIdx.y * ((iSizeX + 1) / 2) + x / 2;
     //-------------------------------------------------
     int iShift = ((x % 2) == 0) ? 0 : 4;
     gfFrame[iPad] = (gucPkdFrm[iPkd] >> iShift) & 0xf;
	if(gfGain == 0L) return;
	//----------------------
	gfFrame[iPad] *= gfGain[blockIdx.y * iSizeX + x];
}

static __global__ void mGDoShort
(	short* gsFrame,
	float* gfGain,
	float* gfFrame,
	int iSizeX,
	int iPadX
)
{    int x = blockIdx.x * blockDim.x + threadIdx.x;
     if(x >= iSizeX) return;
     int iPad = blockIdx.y * iPadX + x;
     int iSize = blockIdx.y * iSizeX + x;
     //----------------------------------
     gfFrame[iPad] = gsFrame[iSize];
     if(gfGain == 0L) return;
	//----------------------
	gfFrame[iPad] *= gfGain[iSize];
}


static __global__ void mGDoUShort
(	unsigned short* gusFrame,
	float* gfGain,
	float* gfFrame,
	int iSizeX,
	int iPadX
)
{    int x = blockIdx.x * blockDim.x + threadIdx.x;
     if(x >= iSizeX) return;
     int iPad = blockIdx.y * iPadX + x;
     int iSize = blockIdx.y * iSizeX + x;
     //----------------------------------
     gfFrame[iPad] = gusFrame[iSize];
	if(gfGain == 0L) return;
	//----------------------
     gfFrame[iPad] *= gfGain[iSize];
}

static __global__ void mGDoFloat
(	float* gfInFrm,
	float* gfGain,
	float* gfOutFrm,
	int iSizeX,
	int iPadX)
{
     int x = blockIdx.x * blockDim.x + threadIdx.x;
     if(x >= iSizeX) return;
     int iPad = blockIdx.y * iPadX + x;
     int iSize = blockIdx.y * iSizeX + x;
     //----------------------------------
     gfOutFrm[iPad] = gfInFrm[iSize];
     if(gfGain == 0L) return;
     //----------------------
     gfOutFrm[iPad] *= gfGain[iSize];
}

GApplyGainToFrame::GApplyGainToFrame(void)
{
}

GApplyGainToFrame::~GApplyGainToFrame(void)
{
}

void GApplyGainToFrame::SetGain(float* gfGain, int* piSize)
{
	m_aiSize[0] = piSize[0];
	m_aiSize[1] = piSize[1];
	m_iPadX = (m_aiSize[0] / 2 + 1) * 2;
	m_gfGain = gfGain;
}

void GApplyGainToFrame::Unpack
(	unsigned char* gucPkdFrm,
	unsigned char* gucRawFrm,
	bool bPadded
)
{	int iPadX = bPadded ? m_iPadX : m_aiSize[0];
	//------------------------------------------
	dim3 aBlockDim(512, 1);
	int iGridX = m_aiSize[0] / aBlockDim.x + 1;
	dim3 aGridDim(iGridX, m_aiSize[1]);
	//---------------------------------
	mGUnpack<<<aGridDim, aBlockDim>>>
	(  gucPkdFrm, gucRawFrm, m_aiSize[0], iPadX
	);
}

void GApplyGainToFrame::DoIt
(	void* gvFrame,
	int iMrcMode,
	float* gfFrame,
	bool bPadded
)
{	if(iMrcMode == Mrc::eMrcUChar || iMrcMode == Mrc::eMrcUCharEM)
	{	this->DoRaw((unsigned char*)gvFrame, gfFrame, bPadded);
	}
	else if(iMrcMode == Mrc::eMrc4Bits)
	{	this->DoPkd((unsigned char*)gvFrame, gfFrame, bPadded);
	}
	else if(iMrcMode == Mrc::eMrcShort)
	{	this->DoShort((short*)gvFrame, gfFrame, bPadded);
	}
	else if(iMrcMode == Mrc::eMrcUShort)
	{	this->DoUShort((unsigned short*)gvFrame, gfFrame, bPadded);
	}
	else if(iMrcMode == Mrc::eMrcFloat)
	{	this->DoFloat((float*)gvFrame, gfFrame, bPadded);
	}
}

void GApplyGainToFrame::DoRaw
(	unsigned char* gucRawFrm,
	float* gfFrame,
	bool bPadded
)
{	int iPadX = bPadded ? m_iPadX : m_aiSize[0];
	//------------------------------------------
	dim3 aBlockDim(512, 1);
	int iGridX = m_aiSize[0] / aBlockDim.x + 1;
	dim3 aGridDim(iGridX, m_aiSize[1]);
	//---------------------------------
	mGDoRaw<<<aGridDim, aBlockDim>>>
	(  gucRawFrm, m_gfGain, gfFrame, m_aiSize[0], iPadX
	);
}

void GApplyGainToFrame::DoPkd
(	unsigned char* gucPkdFrm,
	float* gfFrame,
	bool bPadded
)
{	int iPadX = bPadded ? m_iPadX : m_aiSize[0];
	//------------------------------------------
	dim3 aBlockDim(512, 1);
	int iGridX = m_aiSize[0] / aBlockDim.x + 1;
	dim3 aGridDim(iGridX, m_aiSize[1]);
	//---------------------------------
	mGDoPkd<<<aGridDim, aBlockDim>>>
	(  gucPkdFrm, m_gfGain, gfFrame, m_aiSize[0], iPadX
	);
}

void GApplyGainToFrame::DoShort
(	short* gsFrm,
	float* gfFrame,
	bool bPadded
)
{	int iPadX = bPadded ? m_iPadX : m_aiSize[0];
	//------------------------------------------
	dim3 aBlockDim(512, 1);
     int iGridX = m_aiSize[0] / aBlockDim.x + 1;
     dim3 aGridDim(iGridX, m_aiSize[1]);
     //---------------------------------
     mGDoShort<<<aGridDim, aBlockDim>>>
     (  gsFrm, m_gfGain, gfFrame, m_aiSize[0], iPadX
     );
}

void GApplyGainToFrame::DoUShort
(  	unsigned short* gusFrm,
   	float* gfFrame,
	bool bPadded
)
{    int iPadX = bPadded ? m_iPadX : m_aiSize[0];
     //------------------------------------------
     dim3 aBlockDim(512, 1);
     int iGridX = m_aiSize[0] / aBlockDim.x + 1;
     dim3 aGridDim(iGridX, m_aiSize[1]);
     //---------------------------------
     mGDoUShort<<<aGridDim, aBlockDim>>>
     (  gusFrm, m_gfGain, gfFrame, m_aiSize[0], iPadX
     );
}

void GApplyGainToFrame::DoFloat
(       float* gfInFrm,
        float* gfOutFrm,
        bool bPadded
)
{       int iPadX = bPadded ? m_iPadX : m_aiSize[0];
        //------------------------------------------
        dim3 aBlockDim(512, 1);
        int iGridX = m_aiSize[0] / aBlockDim.x + 1;
        dim3 aGridDim(iGridX, m_aiSize[1]);
        //---------------------------------
        mGDoFloat<<<aGridDim, aBlockDim>>>
        (  gfInFrm, m_gfGain, gfOutFrm, m_aiSize[0], iPadX
        );
}
