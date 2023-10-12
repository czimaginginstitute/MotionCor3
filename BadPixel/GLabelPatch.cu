#include "CDetectMain.h"
#include <stdio.h>
#include <memory.h>
#include <math.h>

using namespace MotionCor2::BadPixel;

static __global__ void mGLabelPatch
(	float* gfImg,
	int iSizeX,
	int iSizeY,
	int* giPatchList,
	int iNumPatches,
	int iRadius)
{
	int iPatch = blockIdx.x * blockDim.x + threadIdx.x;
	if(iPatch >= iNumPatches) return;
	//-------------------------------
	iPatch = giPatchList[iPatch];
	int iCentX = iPatch % iSizeX;
	int iCentY = iPatch / iSizeX;
	//---------------------------
	float fMax = 0.0f;
	for(int y=-iRadius; y<=iRadius; y++)
	{	int yy = y + iCentY;
		if(yy < 0 || yy >= iSizeY) continue;
		for(int x=-iRadius; x<=iRadius; x++)
		{	int xx = x + iCentX;
			if(xx < 0 || xx >= iSizeX) continue;
			float fVal = gfImg[yy * iSizeX + xx];
			if(fVal > fMax) fVal = fMax;
		}
	}
	fMax = fMax * 1.25f + 10.0f;
	//--------------------------
        for(int i=0; i<10; i++)
        {       int x = (int)(iRadius * cosf(i * 0.314159f)) + iCentX;
                int y = (int)(iRadius * sinf(i * 0.314159f)) + iCentY;
                if(x >= 0 && y >= 0 && x < iSizeX && y < iSizeY)
                {       gfImg[y * iSizeX + x] = fMax;
                }
		x = iCentX * 2 - x;
		y = iCentX * 2 - y;
		if(x >= 0 && y >= 0 && x < iSizeX && y < iSizeY)
		{	gfImg[y * iSizeX + x] = fMax;
		}
	}
}

GLabelPatch::GLabelPatch(void)
{
	m_iRadius = 8;
}

GLabelPatch::~GLabelPatch(void)
{
}

void GLabelPatch::SetLabelSize(int iRadius)
{
	m_iRadius = iRadius;
}

void GLabelPatch::DoIt
(	float* gfImg, 
	int* piImgSize,
	int* piPatchList, 
	int iNumPatches)
{
	if(iNumPatches == 0) return;
	//--------------------------
	int* giPatchList = 0L;
	cudaMalloc(&giPatchList, sizeof(int) * iNumPatches);
	cudaMemcpy(giPatchList, piPatchList, sizeof(int) * iNumPatches,
	   cudaMemcpyDefault);
	//--------------------
	dim3 aBlockDim(512, 1);
	dim3 aGridDim(iNumPatches / aBlockDim.x + 1, 1);
	mGLabelPatch<<<aGridDim, aBlockDim>>>(gfImg, piImgSize[0],
	   piImgSize[1], giPatchList, iNumPatches, m_iRadius);
	//----------------------------------------------------
	cudaFree(giPatchList);
}

