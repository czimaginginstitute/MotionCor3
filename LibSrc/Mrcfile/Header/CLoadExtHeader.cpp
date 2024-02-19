#include "../Include/CMrcFileInc.h"
#include <Util/Util_SwapByte.h>
#include <memory.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>

using namespace Mrc;

CLoadExtHeader::CLoadExtHeader(void)
{
	m_iFile = -1;
	m_pcHeader = 0L;
	m_iHeaderSize = 0;
	m_iNumInts = 0;
	m_iNumFloats = 0;
	m_iSymbt = 0;
	m_iNthHeader = -1;
	m_bSwapByte = false;
}

CLoadExtHeader::~CLoadExtHeader(void)
{
	if(m_pcHeader != 0L) delete[] m_pcHeader;
}

void CLoadExtHeader::SetFile(int iFile)
{
	if(m_pcHeader != 0L) 
	{	delete[] m_pcHeader;
		m_pcHeader = 0L;
	}
	//----------------------
	m_iFile = iFile;
	CLoadMainHeader aLoadMain;
	aLoadMain.DoIt(iFile);
	//--------------------
	m_bSwapByte = aLoadMain.m_bSwapByte;
	aLoadMain.GetSize(m_aiImgSize, 3);
	m_iNumInts = aLoadMain.GetNumInts();
	m_iNumFloats = aLoadMain.GetNumFloats();
	m_iSymbt = aLoadMain.GetSymbt();
	//------------------------------
	m_iHeaderSize = m_iNumInts * sizeof(int)
		+ m_iNumFloats * sizeof(float);
	if(m_iHeaderSize > 0) m_pcHeader = new char[m_iHeaderSize];
	//---------------------------------------------------------
	int iGainSize1 = m_iSymbt - m_aiImgSize[2] * m_iHeaderSize;
	int iGainSize2 = m_aiImgSize[0] * m_aiImgSize[1] * sizeof(float);
	if(iGainSize1 == iGainSize2) m_bHasGain = true;
	else m_bHasGain = false;
}

void CLoadExtHeader::DoIt(int iNthHeader)
{
	m_iNthHeader = iNthHeader;
	if(m_iFile == -1) return;
	if(m_pcHeader == 0L) return;
	if(m_iNthHeader < 0) return;
	if(m_iNthHeader >= m_aiImgSize[2]) return;
	//----------------------------------------
	size_t iOffset = 1024 + iNthHeader * m_iHeaderSize;
	lseek64(m_iFile, iOffset, SEEK_SET);
	read(m_iFile, m_pcHeader, m_iHeaderSize);
	if(!m_bSwapByte) return;
	//----------------------
	int iItems = m_iNumInts + m_iNumFloats;
	char acBuf[4] = {0};
	for(int i=0; i<iItems; i++)
	{	int j = i * 4;
		acBuf[0] = m_pcHeader[j+3];
		acBuf[1] = m_pcHeader[j+2];
		acBuf[2] = m_pcHeader[j+1];
		acBuf[3] = m_pcHeader[j];
		memcpy(m_pcHeader + j , acBuf, 4);
	}
}

void CLoadExtHeader::LoadGain(float* pfGain)
{
	if(m_iFile == -1) return;
	if(pfGain == 0L) return;
	if(!m_bHasGain) return;
	//---------------------
	int iOffset = 1024 + m_aiImgSize[2] * m_iHeaderSize;
	int iPixels = m_aiImgSize[0] * m_aiImgSize[1];
	int iBytes = iPixels * sizeof(float);
	lseek64(m_iFile, iOffset, SEEK_SET);
	read(m_iFile, pfGain, iBytes);
	if(!m_bSwapByte) return;
	//----------------------
	char* pcGain = (char*)pfGain;
	char acBuf[4] = {0};
	for(int i=0; i<iPixels; i++)
	{	int j = i * 4;
		acBuf[0] = pcGain[j+3];
		acBuf[1] = pcGain[j+2];
		acBuf[2] = pcGain[j+1];
		acBuf[3] = pcGain[j];
		memcpy(pcGain + j , acBuf, 4);
	}
}

void CLoadExtHeader::GetTilt(float* pfTilt, int iSize)
{
	if(iSize == 0 || m_pcHeader == 0L) return;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	memcpy(pfTilt, pfFields, iSize * sizeof(float));
}

void CLoadExtHeader::GetStage(float* pfStage, int iSize)
{
	if(iSize == 0 || m_pcHeader == 0L) return;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	memcpy(pfStage, pfFields+2, iSize * sizeof(float));	
}

void CLoadExtHeader::GetShift(float* pfShift, int iSize)
{
	if(iSize == 0 || m_pcHeader == 0L) return;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	memcpy(pfShift, pfFields+5, iSize * sizeof(float));	
}

float CLoadExtHeader::GetDefocus(void)
{
	if(m_pcHeader == 0L || m_iNumFloats < 8) return 0.0f;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	return pfFields[7];
}

float CLoadExtHeader::GetExposure(void)
{
	if(m_pcHeader == 0L || m_iNumFloats < 9) return 0.0f;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	return pfFields[8];
}

float CLoadExtHeader::GetMean(void)
{
	if(m_pcHeader == 0L || m_iNumFloats < 10) return 0.0f;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	return pfFields[9];
}

float CLoadExtHeader::GetTiltAxis(void)
{
	if(m_pcHeader == 0L || m_iNumFloats < 11) return 0.0f;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	return pfFields[10];
}

float CLoadExtHeader::GetPixelSize(void)
{
	if(m_pcHeader == 0L || m_iNumFloats < 12) return 0.0f;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	return pfFields[11];
}

float CLoadExtHeader::GetMag(void)
{
	if(m_pcHeader == 0L || m_iNumFloats < 13) return 0.0f;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	return pfFields[12];
}

float CLoadExtHeader::GetNthFloat(int iNthFloat)
{
	if(m_pcHeader == 0L || m_iNumFloats < (iNthFloat+1)) return 0.0f;
	float* pfFields = (float*)(m_pcHeader + m_iNumInts * sizeof(int));
	return pfFields[iNthFloat];
}

