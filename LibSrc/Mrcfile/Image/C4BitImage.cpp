#include "../Include/CMrcFileInc.h"
#include <memory.h>
#include <stdio.h>

using namespace Mrc;

C4BitImage::C4BitImage(void)
{
}

C4BitImage::~C4BitImage(void)
{
}

int C4BitImage::GetPkdSize(int iSize)
{
	return (iSize + 1) / 2;
}

void C4BitImage::GetPkdSize(int* piRawSize, int* piPkdSize)
{
	piPkdSize[0] = C4BitImage::GetPkdSize(piRawSize[0]);
	piPkdSize[1] = piRawSize[1];
}

size_t C4BitImage::GetImgBytes(int iMode, int* piRawSize)
{
	size_t tImgBytes = 0;
	if(iMode == Mrc::eMrc4Bits)
	{	int aiPkdSize[2] = {0};
		GetPkdSize(piRawSize, aiPkdSize);
		tImgBytes = aiPkdSize[0];
		tImgBytes *= (aiPkdSize[1] * sizeof(char));
	}
	else
	{	size_t tPixels = piRawSize[0] * piRawSize[1];
		tImgBytes = CMrcModes::GetBits(iMode) / 8 * tPixels;
	}
	return tImgBytes;
}

size_t C4BitImage::GetLineBytes(int iMode, int iRawSize)
{
	size_t tLineBytes = 0;
	if(iMode == Mrc::eMrc4Bits)
	{	int iPkdSize = GetPkdSize(iRawSize);
		tLineBytes = iPkdSize * sizeof(char);
	}
	else
	{	size_t tRawSize = iRawSize;
		tLineBytes = CMrcModes::GetBits(iMode) / 8 * tRawSize;
	}
	return tLineBytes;
}

void* C4BitImage::GetPkdBuf(int* piRawSize)
{
	int aiPkdSize[2] = {0};
	C4BitImage::GetPkdSize(piRawSize, aiPkdSize);
	int iPkdPixels = aiPkdSize[0] * aiPkdSize[1];
	char* pcBuf = new char[iPkdPixels];
	return pcBuf;
}

void* C4BitImage::Pack(void* pvRawImg, int* piRawSize)
{
	void* pvPkdBuf = C4BitImage::GetPkdBuf(piRawSize);
	C4BitImage::Pack(pvRawImg, piRawSize, pvPkdBuf);
	return pvPkdBuf;
}

void C4BitImage::Pack(void* pvRawImg, int* piRawSize, void* pvPkdImg)
{	
	unsigned char* pucRaw = (unsigned char*)pvRawImg;
	unsigned char* pucPkd = (unsigned char*)pvPkdImg;
	int iPkdX = C4BitImage::GetPkdSize(piRawSize[0]);
	//-----------------------------------------------
	for(int y=0; y<piRawSize[1]; y++)
	{	unsigned char* pucSrc = pucRaw + y * piRawSize[0];
		unsigned char* pucDst = pucPkd + y * iPkdX;
		for(int x=0; x<piRawSize[0]; x+=2)
		{	pucDst[x/2] = (pucSrc[x] & 0xf);
		}
		for(int x=1; x<piRawSize[0]; x+=2)
		{	pucDst[x/2] += ((pucSrc[x] & 0xf) << 4);
		}
	}
}

void C4BitImage::Unpack(void* pvPkdImg, void* pvRawImg, int* piRawSize)
{
	unsigned char* pucPkd = (unsigned char*)pvPkdImg;
	unsigned char* pucRaw = (unsigned char*)pvRawImg;
	int iPkdX = C4BitImage::GetPkdSize(piRawSize[0]);
	//-----------------------------------------------
	for(int y=0; y<piRawSize[1]; y++)
	{	unsigned char* pucSrc = pucPkd + y * iPkdX;
		unsigned char* pucDst = pucRaw + y * piRawSize[0];
		for(int x=0; x<piRawSize[0]; x+=2)
		{	pucDst[x] = pucSrc[x/2] & 0xf;
		}
		for(int x=1; x<piRawSize[0]; x+=2)
		{	pucDst[x] = (pucSrc[x/2] >> 4) & 0xf;
		}
	}
}

