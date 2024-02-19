#include "../Include/CMrcFileInc.h"
#include <memory.h>

using namespace Mrc;

CVerticalFlip::CVerticalFlip(void)
{
}

CVerticalFlip::~CVerticalFlip(void)
{
}

void* CVerticalFlip::DoIt(void* pvImage, int* piSize, int iMode)
{
	int iImgBytes = C4BitImage::GetImgBytes(iMode, piSize);
	void* pvBuf = new char[iImgBytes];
	this->DoIt(pvImage, piSize, iMode, pvBuf);
	return pvBuf;
}

void CVerticalFlip::DoIt(void* pvImage, int* piSize, 
		int iMode, void* pvBuf)
{
	m_aiSize[0] = piSize[0];
	m_aiSize[1] = piSize[1];
	m_iMode = iMode;
	m_iPixels = piSize[0] * piSize[1];
	//--------------------------------
	if(m_iMode == Mrc::eMrcUChar) mDoUChar(pvImage, pvBuf);
	if(m_iMode == Mrc::eMrcUCharEM) return mDoUChar(pvImage, pvBuf);
	if(m_iMode == Mrc::eMrcShort) return mDoShort(pvImage, pvBuf);
	if(m_iMode == Mrc::eMrcUShort) return mDoUShort(pvImage, pvBuf);
	if(m_iMode == Mrc::eMrcFloat) return mDoFloat(pvImage, pvBuf);
	if(m_iMode == Mrc::eMrcInt) return mDoInt(pvImage, pvBuf);
	if(m_iMode == Mrc::eMrc4Bits) return mDo4Bits(pvImage, pvBuf);
}

void CVerticalFlip::mDoUChar(void* pvImage, void* pvBuf)
{
	unsigned char* pcImage = (unsigned char*)pvImage;
	unsigned char* pcBuf = (unsigned char*)pvBuf;
	int iBytes = m_aiSize[0] * sizeof(char);
	//--------------------------------------
	unsigned char* pcSrc = pcImage;
	unsigned char* pcDst = pcBuf + (m_aiSize[1] - 1) * m_aiSize[0];
	memcpy(pcDst, pcSrc, iBytes);
	//---------------------------
	for(int y=1; y<m_aiSize[1]; y++)
	{	pcSrc = pcSrc + m_aiSize[0];
		pcDst = pcDst - m_aiSize[0];
		memcpy(pcDst, pcSrc, iBytes);
	}
}

void CVerticalFlip::mDoShort(void* pvImage, void* pvBuf)
{
	short* psImage = (short*)pvImage;
	short* psBuf = (short*)pvBuf;
	int iBytes = m_aiSize[0] * sizeof(short);
	//---------------------------------------
	short* psSrc = psImage;
	short* psDst = psBuf + (m_aiSize[1] - 1) * m_aiSize[0];
	memcpy(psDst, psSrc, iBytes);
	//---------------------------
	for(int y=1; y<m_aiSize[1]; y++)
	{	psSrc = psSrc + m_aiSize[0];
		psDst = psDst - m_aiSize[0];
		memcpy(psDst, psSrc, iBytes);
	}
}

void CVerticalFlip::mDoUShort(void* pvImage, void* pvBuf)
{
	unsigned short* psImage = (unsigned short*)pvImage;
	unsigned short* psBuf = (unsigned short*)pvBuf;
	int iBytes = m_aiSize[0] * sizeof(short);
	//---------------------------------------
	unsigned short* psSrc = psImage;
	unsigned short* psDst = psBuf + (m_aiSize[1] - 1) * m_aiSize[0];
	memcpy(psDst, psSrc, iBytes);
	//---------------------------
	for(int y=1; y<m_aiSize[1]; y++)
	{	psSrc = psSrc + m_aiSize[0];
		psDst = psDst - m_aiSize[0];
		memcpy(psDst, psSrc, iBytes);
	}
}

void CVerticalFlip::mDoFloat(void* pvImage, void* pvBuf)
{
	float* pfImage = (float*)pvImage;
	float* pfBuf = (float*)pvBuf;
	int iBytes = m_aiSize[0] * sizeof(float);
	//---------------------------------------
	float* pfSrc = pfImage;
	float* pfDst = pfBuf + (m_aiSize[1] - 1) * m_aiSize[0];
	memcpy(pfDst, pfSrc, iBytes);
	//---------------------------
	for(int y=1; y<m_aiSize[1]; y++)
	{	pfSrc = pfSrc + m_aiSize[0];
		pfDst = pfDst - m_aiSize[0];
		memcpy(pfDst, pfSrc, iBytes);
	}
}

void CVerticalFlip::mDoInt(void* pvImage, void* pvBuf)
{
	int* piImage = (int*)pvImage;
	int* piBuf = (int*)pvBuf;
	int iBytes = m_aiSize[0] * sizeof(int);
	//-------------------------------------
	int* piSrc = piImage;
	int* piDst = piBuf + (m_aiSize[1] - 1) * m_aiSize[0];
	memcpy(piDst, piSrc, iBytes);
	//---------------------------
	for(int y=1; y<m_aiSize[1]; y++)
	{	piSrc = piSrc + m_aiSize[0];
		piDst = piDst - m_aiSize[0];
		memcpy(piDst, piSrc, iBytes);
	}
}

void CVerticalFlip::mDo4Bits(void* pvImage, void* pvBuf)
{
	int iMode = Mrc::eMrc4Bits;
	int iLineBytes = C4BitImage::GetLineBytes(iMode, m_aiSize[0]);
	//------------------------------------------------------------
	char* pcSrc = (char*)pvImage;
	char* pcDst = (char*)pvBuf + (m_aiSize[1] - 1) * iLineBytes;
	memcpy(pcDst, pcSrc, iLineBytes);
	//-------------------------------
	for(int y=1; y<m_aiSize[1]; y++)
	{	pcSrc = pcSrc + iLineBytes;
		pcDst = pcDst - iLineBytes;
		memcpy(pcDst, pcSrc, iLineBytes);
	} 
}
