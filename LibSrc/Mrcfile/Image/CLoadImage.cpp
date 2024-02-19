#include "../Include/CMrcFileInc.h"
#include <Util/Util_SwapByte.h>
#include <memory.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>

using namespace Mrc;

CLoadImage::CLoadImage(void)
{
	m_iFile = -1;
	memset(m_aiImgSize, 0, sizeof(m_aiImgSize));
}

CLoadImage::~CLoadImage(void)
{
}

void CLoadImage::SetFile(int iFile)
{
	m_iFile = iFile;
	if(m_iFile == -1) return;
	//-----------------------
	CLoadMainHeader aLoadMain;
	aLoadMain.DoIt(m_iFile);
	aLoadMain.GetSize(m_aiImgSize, 3);
	m_iStackZ = aLoadMain.GetStackZ();
	m_iMode = aLoadMain.GetMode();
	m_iSymbt = aLoadMain.GetSymbt();
	//------------------------------
	m_iImgBytes = C4BitImage::GetImgBytes(m_iMode, m_aiImgSize);
	m_bSwapByte = aLoadMain.m_bSwapByte;
}

void* CLoadImage::DoIt(int iNthImage)
{
	if(m_iFile == -1) return 0L;
	void* pvImage = this->GetBuffer();
	this->DoIt(iNthImage, pvImage);
	return pvImage;
}
/*
void CLoadImage::DoIt(int iNthImage, void* pvImage)
{
	if(m_iFile == -1) return;
	char* pcImage = (char*)pvImage;
	mSeek(iNthImage, 0);
	//------------------
	int iLineBytes = C4BitImage::GetLineBytes(m_iMode, m_aiImgSize[0]);
	int iEndY = m_aiImgSize[1] - 1;
	//-----------------------------
	for(int y=0; y<m_aiImgSize[1]; y++)
	{	read(m_iFile, pcImage + y * iLineBytes, iLineBytes);
	}
	if(m_bSwapByte) mSwapBytes(pcImage, m_aiImgSize[0] * m_aiImgSize[1]);
}
*/
void CLoadImage::DoIt(int iNthImage, void* pvImage)
{
	if(m_iFile == -1) return;
	char* pcImage = (char*)pvImage;
	mSeek(iNthImage, 0);
	read(m_iFile, pvImage, m_iImgBytes);
	//----------------------------------
	if(m_bSwapByte) mSwapBytes(pcImage, m_aiImgSize[0] * m_aiImgSize[1]);
}

void CLoadImage::DoMany(int iStartImg, int iNumImgs, void** ppvImages)
{
	if(m_iFile == -1) return;
	mSeek(iStartImg, 0);
	//------------------
	int iPixels = m_aiImgSize[0] * m_aiImgSize[1];
	int iEndImg = iStartImg + iNumImgs;
	for(int i=iStartImg; i<iEndImg; i++)
	{	void* pvImage = this->GetBuffer();
		read(m_iFile, pvImage, m_iImgBytes);
		if(m_bSwapByte) mSwapBytes((char*)pvImage, iPixels);
		ppvImages[i] = pvImage;
	}
}

void CLoadImage::DoPart(int iImage, int* piOffset,
	int* piPartSize, void* pvImage)
{
	if(m_iFile == -1) return;
	if(pvImage == 0L) return;
	char* pcBuf = (char*)pvImage;
	//---------------------------
	int iPixBytes = CMrcModes::GetBits(m_iMode) / 8;
	int iPartXBytes = piPartSize[0] * iPixBytes;
	int iFullXBytes = m_aiImgSize[0] * iPixBytes;
	int iImgOffset = (piOffset[1] * m_aiImgSize[0] 
		+ piOffset[0]) * iPixBytes;
	int iBufOffset = (piPartSize[1] - 1) * iPartXBytes;
	//-------------------------------------------------
	for(int y=0; y<piOffset[1]; y++)
	{	mSeek(iImage, iImgOffset);
		iImgOffset += iFullXBytes;
		char* pcDst = pcBuf + iBufOffset;
		read(m_iFile, pcDst, iPartXBytes);
		iBufOffset -= iPartXBytes;
	}
	//--------------------------------
	if(m_bSwapByte) 
	{	mSwapBytes(pcBuf, piPartSize[0] * piPartSize[1]);
	}
}

void* CLoadImage::GetBuffer(void)
{
	if(m_iImgBytes <= 0) return 0L;
	void* pvBuf = new char[m_iImgBytes];
	return pvBuf;
}

void CLoadImage::mSeek(int iNthImage, int iBytes)
{
	off64_t tImgBytes = m_iImgBytes;
	off64_t tOffset = 1024 + m_iSymbt + tImgBytes * iNthImage + iBytes;
	lseek64(m_iFile, tOffset, SEEK_SET);
}

void CLoadImage::mSwapBytes(void* pvImage, int iPixels)
{
	if(m_iMode == 1)
	{	short* psImage = (short*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	psImage[i] = Util_SwapByte::DoIt(psImage[i]);
		}
	}
	else if(m_iMode == 2 || m_iMode == 4)
	{	int* piImage = (int*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	piImage[i] = Util_SwapByte::DoIt(piImage[i]);
		}
	}
	else if(m_iMode == 6)
	{	unsigned short* psImage = (unsigned short*)pvImage;
		for(int i=0; i<iPixels; i++)
		{	psImage[i] = Util_SwapByte::DoIt(psImage[i]);
		}
	}
}
