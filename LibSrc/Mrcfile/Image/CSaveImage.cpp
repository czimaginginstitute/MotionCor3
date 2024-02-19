#include "../Include/CMrcFileInc.h"
#include <memory.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>

using namespace Mrc;

CSaveImage::CSaveImage(void)
{
	m_iFile = -1;
	memset(m_aiImgSize, 0, sizeof(m_aiImgSize));
}

CSaveImage::~CSaveImage(void)
{
}

void CSaveImage::SetFile(int iFile)
{
	m_iFile = iFile;
}

void CSaveImage::SetMode(int iMode)
{
	m_iMode = iMode;
}

void CSaveImage::SetImgSize(int* piSize)
{
	m_aiImgSize[0] = piSize[0];
	m_aiImgSize[1] = piSize[1];
}

void CSaveImage::SetSymbt(int iSymbt)
{
	m_iSymbt = iSymbt;
}

void CSaveImage::DoIt(int iNthImage, void* pvImage)
{
	if(m_iFile == -1) return;
	size_t tPixels = m_aiImgSize[0] * m_aiImgSize[1];
	size_t tImgBytes = tPixels * CMrcModes::GetBits(m_iMode) / 8;
	//-----------------------------------------------------------
	off64_t tOffset = 1024 + m_iSymbt + iNthImage * tImgBytes;
	lseek64(m_iFile, tOffset, SEEK_SET);
        write(m_iFile, pvImage, tImgBytes);
}

void CSaveImage::DoIt(int iStartImg, int iNumImgs, void* pvImages)
{
	if(m_iFile == -1) return;
	size_t tPixels = m_aiImgSize[0] * m_aiImgSize[1];
        size_t tImgBytes = tPixels * CMrcModes::GetBits(m_iMode) / 8;
	size_t tTotalBytes = tImgBytes * iNumImgs;
	//----------------------------------------
	off64_t tOffset = 1024 + m_iSymbt + iStartImg * tImgBytes;
        lseek64(m_iFile, tOffset, SEEK_SET);
	char* pcImg = (char*)pvImages;
	write(m_iFile, pcImg, tImgBytes);	
	for(int i=1; i<iNumImgs; i++)
	{	pcImg = pcImg + tImgBytes;
		write(m_iFile, pcImg, tImgBytes);
	}
}

