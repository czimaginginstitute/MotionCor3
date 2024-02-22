#include "../Include/CMrcFileInc.h"
#include <memory.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>

using namespace Mrc;

CSaveExtHeader::CSaveExtHeader(void)
{
	m_pcHeaders = 0L;
	m_iHeaderBytes = 0;
	m_iNumInts = 0;
	m_iNumFloats = 0;
	m_iFile = -1;
}

CSaveExtHeader::~CSaveExtHeader(void)
{
	if(m_pcHeaders != 0L) delete[] m_pcHeaders;
}

void CSaveExtHeader::SetFile(int iFile)
{
	m_iFile = iFile;
}

void CSaveExtHeader::Setup(int iNumInts, int iNumFloats, int iNumHeaders)
{
	if(m_pcHeaders != 0L) 
	{	delete[] m_pcHeaders;
		m_pcHeaders = 0L;
	}
	//-----------------------
	m_iNumInts = iNumInts;
	m_iNumFloats = iNumFloats;
	m_iNumHeaders = iNumHeaders;
	//--------------------------
	m_iHeaderBytes = m_iNumInts * sizeof(int) 
	   + m_iNumFloats * sizeof(float);
	//--------------------------------
	int iBytes = m_iHeaderBytes * m_iNumHeaders;
	if(iBytes == 0) return;
	m_pcHeaders = new char[iBytes];
	memset(m_pcHeaders, 0, iBytes);
}

void CSaveExtHeader::Reset(void)
{
	if(m_pcHeaders == 0L) return;
	int iBytes = m_iNumHeaders * m_iHeaderBytes;
	memset(m_pcHeaders, 0, iBytes);
}

void CSaveExtHeader::SetTilt(int iHeader, float* pfTilt, int iElems)
{
	if(m_pcHeaders == 0L || iHeader >= m_iNumHeaders) return;
	char* pcDst = m_pcHeaders + iHeader * m_iHeaderBytes
	   + m_iNumInts * sizeof(int);
	if(iElems > 2) iElems = 2;
	memcpy(pcDst, pfTilt, sizeof(float) * iElems);
}

void CSaveExtHeader::SetStage(int iHeader, float* pfStage, int iElems)
{
	if(m_pcHeaders == 0L || iHeader >= m_iNumHeaders) return;
	char* pcDst = m_pcHeaders + iHeader * m_iHeaderBytes 
	   + m_iNumInts * sizeof(int) + 2 * sizeof(float);
	if(iElems > 3) iElems = 3;
	memcpy(pcDst, pfStage, sizeof(float) * iElems);
}

void CSaveExtHeader::SetShift(int iHeader, float* pfShift, int iElems)
{
	if(m_pcHeaders == 0L || iHeader >= m_iNumHeaders) return;
	char* pcDst = m_pcHeaders + iHeader * m_iHeaderBytes 
	   + m_iNumInts * sizeof(int) + 5 * sizeof(float);
	if(iElems > 2) iElems = 2;
	memcpy(pcDst, pfShift, sizeof(float) * iElems);
}

void CSaveExtHeader::SetDefocus(int iHeader, float fDefocus)
{
	mSetFloatField(iHeader, 7, fDefocus);
}

void CSaveExtHeader::SetExp(int iHeader, float fExp)
{
	mSetFloatField(iHeader, 8, fExp);
}

void CSaveExtHeader::SetMean(int iHeader, float fMean)
{
	mSetFloatField(iHeader, 9, fMean);
}

void CSaveExtHeader::SetTiltAxis(int iHeader, float fTiltAxis)
{
	mSetFloatField(iHeader, 10, fTiltAxis);
}

void CSaveExtHeader::SetPixelSize(int iHeader, float fPixelSize)
{
	mSetFloatField(iHeader, 11, fPixelSize);
}

void CSaveExtHeader::SetMag(int iHeader, float fMag)
{
	mSetFloatField(iHeader, 12, fMag);
}

void CSaveExtHeader::SetNthFloat(int iHeader, int iNthFloat, float fVal)
{
	mSetFloatField(iHeader, iNthFloat, fVal);
}

void CSaveExtHeader::mSetFloatField(int iHeader, int iField, float fVal)
{
        if(m_pcHeaders == 0L || iHeader >= m_iNumHeaders) return;
        float* pfFields = reinterpret_cast<float*>(m_pcHeaders
           + iHeader * m_iHeaderBytes + m_iNumInts * sizeof(int));
        pfFields[iField] = fVal;
}


void CSaveExtHeader::SetHeader(int iHeader, char* pcHeader, int iSize)
{
	if(m_pcHeaders == 0L || iHeader >= m_iNumHeaders) return;
	int iMinSize = (iSize < m_iHeaderBytes) ? iSize : m_iHeaderBytes;
	char* pcDst = m_pcHeaders + iHeader * m_iHeaderBytes;
	memcpy(pcDst, pcHeader, iMinSize);
}	

void CSaveExtHeader::DoIt(void)
{
	if(m_iFile == -1) return;
	if(m_pcHeaders == 0L) return;
        lseek64(m_iFile, 1024, SEEK_SET);
	int iBytes = m_iNumHeaders * m_iHeaderBytes;
        write(m_iFile, m_pcHeaders, iBytes);
}

void CSaveExtHeader::SaveGain(int iOffset, float* pfGain, int iBytes)
{
	if(m_iFile == -1) return;
	if(pfGain == 0L || iBytes == 0) return;
	lseek64(m_iFile, iOffset, SEEK_SET);
	write(m_iFile, pfGain, iBytes);
}

