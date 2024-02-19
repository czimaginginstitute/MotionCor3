#include "../Include/CMrcFileInc.h"

using namespace Mrc;

CMrcScale::CMrcScale(void)
{
	m_iPixels = 0;
	m_fScale = 1.0f;
}

CMrcScale::~CMrcScale(void)
{
}

void CMrcScale::Setup(int iNewMode, float fScale)
{
	m_iNewMode = iNewMode;
	m_fScale = fScale;
}

void* CMrcScale::DoIt(float* pfData, int iPixels)
{
	int iPixBytes = CMrcModes::GetBits(m_iNewMode) / 8;
	void* pvData = new char[iPixels * iPixBytes];
	this->DoIt(pfData, iPixels, pvData);
	return pvData;
}

void CMrcScale::DoIt(float* pfData, int iPixels, void* pvData)
{
	m_iPixels = iPixels;
	if(m_iNewMode == Mrc::eMrcUChar) mToUChar(pfData, pvData);
	else if(m_iNewMode == Mrc::eMrcUCharEM) mToUChar(pfData, pvData);
	else if(m_iNewMode == Mrc::eMrcShort) mToShort(pfData, pvData);
	else if(m_iNewMode == Mrc::eMrcUShort) mToUShort(pfData, pvData);
	else if(m_iNewMode == Mrc::eMrcInt) mToInt(pfData, pvData);
	else if(m_iNewMode == Mrc::eMrcFloat) mToFloat(pfData, pvData);
}

void CMrcScale::mToUChar(float* pfData, void* pvData)
{
	unsigned char* pucBuf = (unsigned char*)pvData;
	for(int i=0; i<m_iPixels; i++)
	{	int iVal = (int)(pfData[i] * m_fScale);
		if(iVal < 0) pucBuf[i] = 0;
		else if(iVal > 255) pucBuf[i] = 255;
		else pucBuf[i] = (unsigned char)iVal;
	}
}

void CMrcScale::mToShort(float* pfData, void* pvData)
{
	short* psBuf = (short*)pvData;
	for(int i=0; i<m_iPixels; i++)
	{	int iVal = (int)(pfData[i] * m_fScale);
		if(iVal > 32767) psBuf[i] = 32767;
		else if(iVal < -32767) psBuf[i] = -32767;
		else psBuf[i] = (short)iVal;
	}
}

void CMrcScale::mToUShort(float* pfData, void* pvData)
{
	unsigned short* psBuf = (unsigned short*)pvData;
	for(int i=0; i<m_iPixels; i++)
	{	int iVal = (int)(pfData[i] * m_fScale);
		if(iVal > 65535) psBuf[i] = 65535;
		else if(iVal < 0) psBuf[i] = 0;
		else psBuf[i] = (unsigned short)iVal;
	}
}

void CMrcScale::mToInt(float* pfData, void* pvData)
{
	int* piBuf = (int*)pvData;
	for(int i=0; i<m_iPixels; i++)
	{	piBuf[i] = (int)(pfData[i] * m_fScale);
	}
}

void CMrcScale::mToFloat(float* pfData, void* pvData)
{
	float* pfBuf = (float*)pvData;
	for(int i=0; i<m_iPixels; i++)
	{	pfBuf[i] = pfData[i] * m_fScale;
	}
}
