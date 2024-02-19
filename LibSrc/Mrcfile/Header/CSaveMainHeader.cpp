#include "../Include/CMrcFileInc.h"
#include <memory.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdio.h>
#include <inttypes.h>

using namespace Mrc;

CSaveMainHeader::CSaveMainHeader(void)
{
	m_iFile = -1;
	mSetDefault();
	//------------
	m_dMeanSum = 0;
	m_iNumMeans = 0;
}

CSaveMainHeader::~CSaveMainHeader(void)
{
}

void CSaveMainHeader::SetFile(int iFile)
{
	m_iFile = iFile;
}

void CSaveMainHeader::Reset(void)
{
	mSetDefault();
	m_aHeader.nx = 0;
	m_aHeader.ny = 0;
	m_aHeader.nz = 0;
	m_dMeanSum = 0;
	m_iNumMeans = 0;
}

void CSaveMainHeader::SetImgSize
(	int* piImgSize,
	int iNumImgs,
	int iNumImgStacks,
	float fPixelSize
)
{	m_aHeader.nx = piImgSize[0];
	m_aHeader.ny = piImgSize[1];
	m_aHeader.nz = iNumImgs * iNumImgStacks;
	//--------------------------------------
	m_aHeader.mx = m_aHeader.nx;
	m_aHeader.my = m_aHeader.ny;
	m_aHeader.mz = iNumImgs;
	//----------------------
	m_aHeader.xlen = m_aHeader.mx * fPixelSize;
	m_aHeader.ylen = m_aHeader.my * fPixelSize;
	m_aHeader.zlen = m_aHeader.mz * fPixelSize;
}

void CSaveMainHeader::SetMode(int iMode)
{
	m_aHeader.mode = iMode;
}

void CSaveMainHeader::SetMinMaxMean
(	float fMin, 
	float fMax, 
	float fMean
)
{
	if(m_iNumMeans <= 0)
	{	m_aHeader.amin = fMin;
		m_aHeader.amax = fMax;
	}
	else
	{	if(m_aHeader.amin > fMin) m_aHeader.amin = fMin;
		if(m_aHeader.amax < fMax) m_aHeader.amax = fMax;
	}
	//------------------------------------------------------
	m_dMeanSum += fMean;
	m_iNumMeans += 1;
	m_aHeader.amean = (float)(m_dMeanSum / m_iNumMeans);
}

void CSaveMainHeader::SetNumInts(int iNumInts)
{
	m_aHeader.numintegers = iNumInts;
}

void CSaveMainHeader::SetNumFloats(int iNumFloats)
{
	m_aHeader.numfloats = iNumFloats;
}

void CSaveMainHeader::SetNthLabel(char* pcLabel, int iIndex)
{
	if(pcLabel == 0L) return;
	if(iIndex < 0 || iIndex > 9) return;
	char* pDestLabel = m_aHeader.data[iIndex];
	size_t iSize = strlen(pcLabel);
	if(iSize >= 80) iSize = 79;
	memcpy(pDestLabel, pcLabel, iSize);
	m_aHeader.nlabl += 1;
}

void CSaveMainHeader::SetNthLabel(wchar_t* pwcLabel, int iNthLabel)
{
	if(pwcLabel == 0L) return;
	int iSize = wcslen(pwcLabel);
	if(iSize == 0) return;
	//--------------------
	char* pcLabel = new char[iSize + 1];
	for(int i=0; i<iSize; i++)
	{	pcLabel[i] = (char)pwcLabel[i];
	}
	pcLabel[iSize] = 0;
	//-----------------
	this->SetNthLabel(pcLabel, iNthLabel);
	if(pcLabel != 0L) delete[] pcLabel;
}

void CSaveMainHeader::SetSymbt(int iSymbt)
{
	m_aHeader.nsymbt = iSymbt;
}

void CSaveMainHeader::DoIt(void)
{
	if(m_iFile == -1) return;
	lseek64(m_iFile, 0, SEEK_SET);
	write(m_iFile, &m_aHeader, 1024);
}

void CSaveMainHeader::DoIt(CMainHeader* pHeader)
{
	if(m_iFile == -1 || pHeader == 0L) return;
	lseek64(m_iFile, 0, SEEK_SET);
	write(m_iFile, pHeader, 1024);
}

int CSaveMainHeader::GetMode(void)
{
	return m_aHeader.mode;
}

void CSaveMainHeader::GetImgSize(int* piImgSize, int iElems)
{
	if(iElems == 1) 
	{	piImgSize[0] = m_aHeader.nx;
	}
	else if(iElems == 2)
	{	piImgSize[0] = m_aHeader.nx;
		piImgSize[1] = m_aHeader.ny;
	}
	else if(iElems == 3)
	{	piImgSize[0] = m_aHeader.nx;
		piImgSize[1] = m_aHeader.ny;
		piImgSize[2] = m_aHeader.nz;
	}
}

int CSaveMainHeader::GetNumInts(void)
{	
	return m_aHeader.numintegers;
}

int CSaveMainHeader::GetNumFloats(void)
{
	return m_aHeader.numfloats;
}

int CSaveMainHeader::GetSymbt(void)
{
	return m_aHeader.nsymbt;
}

int CSaveMainHeader::GetGainBytes(void)
{
	int iExtHdrSize = m_aHeader.numfloats * sizeof(float)
		+ m_aHeader.numintegers * sizeof(int);
	int iGainBytes = m_aHeader.nsymbt - m_aHeader.nz * iExtHdrSize;
	return iGainBytes;
}

void CSaveMainHeader::mSetDefault(void)
{	
	m_aHeader.nxstart = 0;
	m_aHeader.nystart = 0;
	m_aHeader.nzstart = 0;
	//--------------------
	m_aHeader.mx = 1;
	m_aHeader.my = 1;
	m_aHeader.mz = 1;
	//---------------
	m_aHeader.alpha = 90;
	m_aHeader.beta = 90;
	m_aHeader.gamma = 90;
	//-------------------
	m_aHeader.mapc = 1;
	m_aHeader.mapr = 2;
	m_aHeader.maps = 3;
	//-----------------
	m_aHeader.ispg = 0;
	strcpy(m_aHeader.blank, "AGAR");  // EXTTYPE
	int iVersion = 2014 * 10;
	memcpy(m_aHeader.blank + 4, &iVersion, sizeof(int));
	//--------------------------------------------------
	m_aHeader.dvid = -16224;
	m_aHeader.sub = 1;
	m_aHeader.type = 5;
	//-----------------
	m_aHeader.xtilt = 90;
	m_aHeader.ytilt = 90;
	m_aHeader.ztilt = 90;
	//-------------------
	m_aHeader.numintegers = 0;
	m_aHeader.numfloats = 0;
	m_aHeader.nsymbt = 0;
	//-------------------
	char* pcMap = (char*)(&m_aHeader.z0);
	strcpy(pcMap, "MAP");
	//-------------------
	mCheckByteOrder();
	
}

//-------------------------------------------------------------------
// 1. Jude Short, MRC Lab, 10/25/2016.
// 2. JSB 192 (2015) 146-150
// 3. You need to put the string 'MAP ' into word 53,
// 2. and the machine stamp into word 54. For the machine stamp you 
//    need to write 0x44 0x44 0x00 0x00 for little-endian machines 
//    and 0x11 0x11 0x00 0x00 for big endian machines.
//-------------------------------------------------------------------
void CSaveMainHeader::mCheckByteOrder(void)
{
	union 
	{  uint32_t i[1];
           uint8_t c[4];
    	}  endian;
	endian.i[0] = 0x0a0b0c0d;
	//-----------------------
    	if( endian.c[0] == 0x0a &&
            endian.c[1] == 0x0b &&
            endian.c[2] == 0x0c &&
            endian.c[3] == 0x0d)     // big Endian 
	{	endian.c[0] = 0x11;
		endian.c[1] = 0x11;
		endian.c[2] = 0x00;
		endian.c[3] = 0x00;
	}
	else
	{	endian.c[0] = 0x44;  // little Endian
		endian.c[1] = 0x44;
		endian.c[2] = 0x00;
		endian.c[3] = 0x00;
	}
	memcpy(&m_aHeader.x0, endian.c, 4);
	//---------------------------------
	char* pcWord53 = (char*)(&m_aHeader.z0);
	strcpy(pcWord53, "MAP");
}
