#include "CUtilInc.h"
#include <cuda.h>
#include <cuda_runtime.h>
#include <string.h>
#include <stdio.h>

using namespace MotionCor2::Util;

CSaveTempMrc::CSaveTempMrc(void)
{
}

CSaveTempMrc::~CSaveTempMrc(void)
{
}

void CSaveTempMrc::SetFile(char* pcMain, const char* pcMinor)
{
	memset(m_acMrcFile, 0, sizeof(m_acMrcFile));
	if(pcMain == 0L) return;
	//-----------------
	strcpy(m_acMrcFile, pcMain);
	char* pcExt = strstr(m_acMrcFile, ".mrc");
	if(pcExt != 0L) pcExt[0] = '\0';
	//-----------------
	if(pcMinor != 0L && strlen(pcMinor) > 0) 
	{	strcat(m_acMrcFile, pcMinor);
	}
	//-----------------	
	if(strstr(m_acMrcFile, ".mrc")) return;
	else strcat(m_acMrcFile, ".mrc");
}

void CSaveTempMrc::GDoIt(cufftComplex* gCmp, int* piCmpSize)
{
	int aiPadSize[]= {0, piCmpSize[1]};
	aiPadSize[0] = piCmpSize[0] * 2;
	//------------------------------
	CCufft2D cufft2D;
	cufft2D.CreateInversePlan(piCmpSize, true);
	cufft2D.Inverse(gCmp);
	//--------------------
	this->GDoIt((float*)gCmp, aiPadSize);
	//-----------------------------------
	cufft2D.CreateForwardPlan(aiPadSize, true);
	cufft2D.Forward((float*)gCmp, aiPadSize);
}

void CSaveTempMrc::GDoIt(float* gfImg, int* piSize)
{
	int iPixels = piSize[0] * piSize[1];
	size_t tBytes = iPixels * sizeof(float);
	float* pfBuf = new float[iPixels];
	cudaMemcpy(pfBuf, gfImg, tBytes, cudaMemcpyDeviceToHost);
	this->DoIt(pfBuf, Mrc::eMrcFloat, piSize);
	delete[] pfBuf;
}

void CSaveTempMrc::GDoIt(unsigned char* gucImg, int* piSize)
{
	int iPixels = piSize[0] * piSize[1];
	size_t tBytes = iPixels * sizeof(char);
	unsigned char* pucBuf = new unsigned char[iPixels];
	cudaMemcpy(pucBuf, gucImg, tBytes, cudaMemcpyDeviceToHost);
	this->DoIt(pucBuf, Mrc::eMrcUChar, piSize);
	delete[] pucBuf;
}

void CSaveTempMrc::DoIt(void* pvImg, int iMode, int* piSize)
{
	Mrc::CSaveMrc aSaveMrc;
	if(!aSaveMrc.OpenFile(m_acMrcFile)) return;
	//-----------------------------------------
	aSaveMrc.SetMode(iMode);
	aSaveMrc.SetImgSize(piSize, 1, 1, 1.0f);
	aSaveMrc.SetExtHeader(0, 0, 0);
	aSaveMrc.DoIt(0, pvImg);
}	

