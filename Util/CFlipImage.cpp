#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <string.h>
#include <stdio.h>

using namespace MotionCor2::Util;

CFlipImage::CFlipImage(void)
{
}

CFlipImage::~CFlipImage(void)
{
}

float* CFlipImage::DoIt(float* pfImg, int* piImgSize, bool bClean)
{
	if(pfImg == 0L) return 0L;
	int iPixels = piImgSize[0] * piImgSize[1];
	float* pfFlip = new float[iPixels];
	//---------------------------------
	int iEndY = piImgSize[1] - 1;
	int iBytes = sizeof(float) * piImgSize[0];
	for(int y=0; y<piImgSize[1]; y++)
	{	float* pfSrc = pfImg + y * piImgSize[0];
		float* pfDst = pfFlip + (iEndY - y) * piImgSize[0];
		memcpy(pfDst, pfSrc, iBytes);
	}
	if(bClean) delete[] pfImg;
	//------------------------
	return pfFlip;	
}

void CFlipImage::DoIt(float* pfImg, int* piImgSize)
{
	if(pfImg == 0L) return;
	float* pfFlip = this->DoIt(pfImg, piImgSize, false);
	size_t tBytes = sizeof(float) * piImgSize[0] * piImgSize[1];
	memcpy(pfImg, pfFlip, tBytes);
}

