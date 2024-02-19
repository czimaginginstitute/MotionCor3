#include "../Include/CMrcFileInc.h"
#include <Util/Util_FileName.h>
#include <Util/Util_String.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace Mrc;

CLoadMrc::CLoadMrc()
{
	m_pLoadMain = new CLoadMainHeader;
	m_pLoadExt = new CLoadExtHeader;
	m_pLoadImg = new CLoadImage;
	m_iFile = -1;
	m_fPixelSize = 0.0f;
}

CLoadMrc::~CLoadMrc()
{
	this->CloseFile();
	if(m_pLoadMain != 0L) delete m_pLoadMain;
	if(m_pLoadExt != 0L) delete m_pLoadExt;
	if(m_pLoadImg != 0L) delete m_pLoadImg;
}

bool CLoadMrc::OpenFile(char* pcFileName)
{
	this->CloseFile();
	if(pcFileName == NULL || strlen(pcFileName) == 0) return false;
	m_iFile = open(pcFileName, O_RDONLY);
	if(m_iFile == -1) return false;
	//-----------------------------
	m_pLoadMain->DoIt(m_iFile);
	m_pLoadExt->SetFile(m_iFile);
	m_pLoadImg->SetFile(m_iFile);
	//---------------------------
	m_fPixelSize = m_pLoadMain->GetPixelSize();
	if(m_fPixelSize <= 0.01f)
	{	m_pLoadExt->DoIt(0);
		m_fPixelSize = m_pLoadExt->GetPixelSize();
	}
	//------------------------------------------------
	return true;
}

bool CLoadMrc::OpenFile(wchar_t* pwcFileName)
{
	this->CloseFile();
	if(pwcFileName == 0L) return false;
	int iSize = wcslen(pwcFileName);
	if(iSize == 0L) return false;
	//---------------------------
	char* pcFile = new char[iSize + 1];
	for(int i=0; i<iSize; i++) pcFile[i] = (char)pwcFileName[i];
	pcFile[iSize] = 0;
	//----------------
	bool bOpen = this->OpenFile(pcFile);
	if(pcFile != 0L) delete[] pcFile;
	return bOpen;
}

bool CLoadMrc::OpenFile(char* pcFileName, int iSerialNum)
{
	this->CloseFile();
	if(pcFileName == NULL) return false;
	if(strlen(pcFileName) == 0) return false;
	//---------------------------------------
	Util_FileName aUtilFileName(pcFileName);
	char* pcSerialFileName = aUtilFileName.InsertSerial(iSerialNum);
	bool bOpen = this->OpenFile(pcSerialFileName);
	delete[] pcSerialFileName;
	return bOpen;
}

bool CLoadMrc::OpenFile(wchar_t* pwcFileName, int iSerialNum)
{
	this->CloseFile();
	if(pwcFileName == 0L) return false;
	int iSize = wcslen(pwcFileName);
	if(iSize == 0L) return false;
	//---------------------------
	char* pcFile = new char[iSize + 1];
	for(int i=0; i<iSize; i++) pcFile[i] = (char)pwcFileName[i];
	pcFile[iSize] = 0;
	//----------------
	bool bOpen = this->OpenFile(pcFile, iSerialNum);
	if(pcFile != 0L) delete[] pcFile;
	return bOpen;
}

void CLoadMrc::CloseFile(void)
{
	if(m_iFile == -1) return;
	close(m_iFile);
	m_iFile = -1;
}

float CLoadMrc::GetPixelSize(void)
{
	return m_fPixelSize;
}

