#include "../Include/CMrcFileInc.h"
#include <memory.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <stdio.h>

using namespace Mrc;

CLoadMainHeader::CLoadMainHeader(void)
{
}

CLoadMainHeader::~CLoadMainHeader(void)
{
}

void CLoadMainHeader::DoIt(int iFile)
{
	m_bSwapByte = false;
	memset(&m_aHeader, 0, sizeof(CMainHeader));
	if(iFile == -1) return;
	//---------------------
	lseek64(iFile, 0, SEEK_SET);
	read(iFile, &m_aHeader, 1024);
	//----------------------------
	int iMaxSize = 32 * 1024;
	if(m_aHeader.mode < 0) m_bSwapByte = true;
	else if(m_aHeader.nx < 0) m_bSwapByte = true;
	else if(m_aHeader.ny < 0) m_bSwapByte = true;
	else if(m_aHeader.nz < 0) m_bSwapByte = true;
	else if(m_aHeader.nz < 0) m_bSwapByte = true;
	else if(m_aHeader.nx > iMaxSize) m_bSwapByte = true;
	else if(m_aHeader.ny > iMaxSize) m_bSwapByte = true;
	else if(m_aHeader.nz > iMaxSize) m_bSwapByte = true;
	//--------------------------------------------------
	if(m_bSwapByte) m_aHeader.SwapByte();
}

int CLoadMainHeader::GetSizeX(void)
{
	return m_aHeader.nx;
}

int CLoadMainHeader::GetSizeY(void)
{
	return m_aHeader.ny;
}

int CLoadMainHeader::GetSizeZ(void)
{
	return m_aHeader.nz;
}

void CLoadMainHeader::GetSize(int* piSize, int iElems)
{
	if(iElems <= 0) return;
	piSize[0] = m_aHeader.nx;
	if(iElems > 1) piSize[1] = m_aHeader.ny;
	if(iElems > 2) piSize[2] = m_aHeader.nz;
}

int CLoadMainHeader::GetStackZ(void)
{
	return m_aHeader.mz;
}

int CLoadMainHeader::GetMode(void)
{
	return m_aHeader.mode;
}

int CLoadMainHeader::GetSymbt(void)
{
	return m_aHeader.nsymbt;
}

int CLoadMainHeader::GetNumInts(void)
{
	return m_aHeader.numintegers;
}

int CLoadMainHeader::GetNumFloats(void)
{
	return m_aHeader.numfloats;
}

float CLoadMainHeader::GetLengthX(void)
{
	return m_aHeader.xlen;
}

float CLoadMainHeader::GetLengthY(void)
{
	return m_aHeader.ylen;
}

float CLoadMainHeader::GetMean(void)
{
	return m_aHeader.amean;
}

float CLoadMainHeader::GetMax(void)
{
	return m_aHeader.amax;
}

float CLoadMainHeader::GetMin(void)
{
	return m_aHeader.amin;
}

char* CLoadMainHeader::GetNthLabel(int iNthLabel)
{
	if(iNthLabel < 0 || iNthLabel > 9) return 0L;
	return m_aHeader.data[iNthLabel];
}

float CLoadMainHeader::GetPixelSize(void)
{
	if(m_aHeader.nx <= 0) return 0.0f;
	float fPixelSize = m_aHeader.xlen / m_aHeader.nx;
	return fPixelSize;
}

int CLoadMainHeader::GetGainBytes(void)
{
	int iExtHeaderSize = m_aHeader.numfloats * sizeof(float)
		+ m_aHeader.numintegers * sizeof(int);
	int iGainBytes = m_aHeader.nsymbt - m_aHeader.nz * iExtHeaderSize;
	return iGainBytes;
}
