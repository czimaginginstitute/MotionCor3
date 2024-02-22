#include "../Include/CMrcFileInc.h"
#include <Util/Util_FileName.h>
#include <memory.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

using namespace Mrc;

CReviseExtHeader::CReviseExtHeader(void)
{
	m_iFile = -1;
	m_iHeader = -1;
}

CReviseExtHeader::~CReviseExtHeader()
{
	this->CloseFile();
}

bool CReviseExtHeader::OpenFile(char* pcFileName)
{
	m_iHeader = -1;
	this->CloseFile();
	//----------------
	mode_t aMode = S_IRUSR | S_IWUSR | S_IWGRP | S_IRGRP | S_IROTH;
	m_iFile = open(pcFileName, O_RDWR, aMode);
	if(m_iFile == -1) return false;
	//-----------------------------
	m_aLoadExt.SetFile(m_iFile);
	m_aSaveExt.SetFile(m_iFile);
	m_aSaveExt.Setup(m_aLoadExt.m_iNumInts, m_aLoadExt.m_iNumFloats, 
	   m_aLoadExt.m_aiImgSize[2]);
	return true;
}

bool CReviseExtHeader::OpenFile(wchar_t* pwcFileName)
{
	this->CloseFile();
	if(pwcFileName == 0L) return false;
	int iSize = wcslen(pwcFileName);
	if(iSize == 0) return false;
	//--------------------------
	char* pcFile = new char[iSize + 1];
	for(int i=0; i<iSize; i++) pcFile[0] = (char)pwcFileName[i];
	pcFile[iSize] = 0;
	//----------------
	bool bOpen = this->OpenFile(pcFile);
	if(pcFile != 0L) delete[] pcFile;
	return bOpen;
}

void CReviseExtHeader::CloseFile(void)
{
	if(m_iFile == -1) return;
	close(m_iFile);
	m_iFile = -1;
}

void CReviseExtHeader::Load(int iNthHeader)
{
	if(m_iHeader == iNthHeader) return;
	m_iHeader = iNthHeader;
	m_aLoadExt.DoIt(iNthHeader);
	m_aSaveExt.SetHeader(m_iHeader, m_aLoadExt.m_pcHeader, 
	   m_aLoadExt.m_iHeaderSize);
}

void CReviseExtHeader::SetStage(float* pfStage, int iElems)
{	
	m_aSaveExt.SetStage(m_iHeader, pfStage, iElems);
}

void CReviseExtHeader::SetShift(float* pfShift, int iElems)
{
	m_aSaveExt.SetShift(m_iHeader, pfShift, iElems);
}

void CReviseExtHeader::SetFloat(float fValue, int iField)
{
	m_aSaveExt.SetNthFloat(m_iHeader, iField, fValue);
}

void CReviseExtHeader::Save(void)
{
	if(m_iHeader == -1) return;
	if(m_iFile == -1) return;
	m_aSaveExt.DoIt();
}
