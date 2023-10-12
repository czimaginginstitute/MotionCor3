#include "CDataUtilInc.h"
#include "../CMainInc.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/types.h>
#include <sys/inotify.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>

using namespace MotionCor2;
using namespace MotionCor2::DataUtil;

CStackFolder* CStackFolder::m_pInstance = 0L;

CStackFolder* CStackFolder::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CStackFolder;
	return m_pInstance;
}

void CStackFolder::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

//------------------------------------------------------------------------------
// m_aMutex: It is initialized in Util_Thread. Do not init here
//------------------------------------------------------------------------------
CStackFolder::CStackFolder(void)
{
	m_dRecentFileTime = 0.0;
}

//------------------------------------------------------------------------------
// m_aMutex: It is destroyed in Util_Thread. Do not destroy here.
//------------------------------------------------------------------------------
CStackFolder::~CStackFolder(void)
{
	this->mClean();
}

void CStackFolder::PushPackage(CDataPackage* pDataPackage)
{
	pthread_mutex_lock(&m_aMutex);
	m_aFileQueue.push(pDataPackage);
	pthread_mutex_unlock(&m_aMutex);
}

CDataPackage* CStackFolder::GetPackage(bool bPop)
{
	CDataPackage* pDataPackage = 0L;
	pthread_mutex_lock(&m_aMutex);
	if(!m_aFileQueue.empty())
	{	pDataPackage = m_aFileQueue.front();
		if(bPop) m_aFileQueue.pop();
	}
	pthread_mutex_unlock(&m_aMutex);
	return pDataPackage;
}

void CStackFolder::DeleteFront(void)
{
	CDataPackage* pPackage = this->GetPackage(true);
	if(pPackage != 0L) delete pPackage;
}

int CStackFolder::GetQueueSize(void)
{
	pthread_mutex_lock(&m_aMutex);
	int iSize = m_aFileQueue.size();
	pthread_mutex_unlock(&m_aMutex);
	return iSize;
}

bool CStackFolder::ReadFiles(void)
{
	this->mClean();
	bool bSuccess = true;
	m_dRecentFileTime = 0.0;
	//----------------------
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iSerial == 0) 
	{	bSuccess = mReadSingle();
		return bSuccess;
	}
	//----------------------
	bSuccess = mGetDirName();
	if(!bSuccess) return false;
	strcpy(m_acSuffix, pInput->m_acInSuffix);
	printf("Directory: %s\n", m_acDirName);
	printf("Prefix:    %s\n", m_acPrefix);
	printf("Suffix:    %s\n", m_acSuffix);
	//-------------------------------------------------
	// Read all the movies in the specified folder for
	// batch processing.
	//-------------------------------------------------
	if(pInput->m_iSerial == 1) 
	{	int iNumRead = mReadFolder(true);
		bSuccess = (iNumRead > 0) ? true : false;
	}
	//-------------------------------------------------------
	// Monitor the specified folder for new movie files that
	// have been saved recently. If yes, read the new movie
	// file name for future loading.
	//------------------------------------------------------- 
	else if(pInput->m_iSerial > 1) 
	{	bSuccess = mAsyncReadFolder();
	}
	return bSuccess;
}

bool CStackFolder::mReadSingle(void)
{
	int iSize = 256;
	char* pcFileName = new char[iSize];
	memset(pcFileName, 0, sizeof(char) * iSize);
	//------------------------------------------
	char* pcInputFile = mGetInPrefix();
	strcpy(pcFileName, pcInputFile);
	//------------------------------
	CDataPackage* pDataPackage = new CDataPackage;
	pDataPackage->m_pcInFileName = pcFileName;
	//---------------------------------------
	m_aFileQueue.push(pDataPackage);
	return true;
}

//--------------------------------------------------------------------
// 1. User passes in a file name that is used as the template to 
//    search for a series stack files containing serial numbers.
// 2. The template file contains the full path that is used to
//    determine the folder containing the series stack files
//--------------------------------------------------------------------
int CStackFolder::mReadFolder(bool bFirstTime)
{
	DIR* pDir = opendir(m_acDirName);
	if(pDir == 0L)
	{	fprintf(stderr, "Error: cannot open folder\n   %s"
		   "in CStackFolder::mReadFolder.\n\n", m_acDirName);
		return -1;
	}
	//----------------
	int iNumRead = 0;
	int iPrefix = strlen(m_acPrefix);
	int iSuffix = strlen(m_acSuffix);
	struct dirent* pDirent;
	char *pcPrefix = 0L, *pcSuffix = 0L;
	//----------------------------------
	struct stat statBuf;
	char acFullFile[256] = {'\0'};
	strcpy(acFullFile, m_acDirName);
	char* pcMainFile = acFullFile + strlen(m_acDirName);
	//--------------------------------------------------
	while(true)
	{	pDirent = readdir(pDir);
		if(pDirent == 0L) break;
		if(pDirent->d_name[0] == '.') continue;
		//-------------------------------------
		if(iPrefix > 0)
		{	pcPrefix = strstr(pDirent->d_name, m_acPrefix);
			if(pcPrefix == 0L) continue;
		}
		//----------------------------------
		if(iSuffix > 0)
		{	pcSuffix = strcasestr(pDirent->d_name 
			   + iPrefix, m_acSuffix);
			if(pcSuffix == 0L) continue;
		}
		//----------------------------------
		// check if this is the latest file
		//----------------------------------
		strcpy(pcMainFile, pDirent->d_name);
		if(stat(acFullFile, &statBuf) < 0) continue;
		double dTime = statBuf.st_mtim.tv_sec +
		   1e-9 * statBuf.st_mtim.tv_nsec;
		double dDeltaT = dTime - m_dRecentFileTime;
		//-----------------------------------------------------
		// But if this is the first time to read the directory,
		// bypass the latest file check, read all instead.
		//-----------------------------------------------------
		if(dDeltaT <= 0 && !bFirstTime) continue;
		if(dDeltaT > 0) m_dRecentFileTime = dTime;
		//----------------------------------------
		CDataPackage* pPackage = new CDataPackage;
		pPackage->m_pcInFileName = new char[256];
		strcpy(pPackage->m_pcInFileName, acFullFile);
		pPackage->m_pcSerial = mGetSerial(pDirent->d_name);
		this->PushPackage(pPackage);
		printf("added: %s\n", pPackage->m_pcInFileName);
		iNumRead += 1;
	}
	closedir(pDir);
	return iNumRead;
}

char* CStackFolder::mGetSerial(char* pcInputFile)
{
	char acBuf[256] = {'\0'};
	int iPrefixLen = strlen(m_acPrefix);
	strcpy(acBuf, pcInputFile+iPrefixLen);
	//-------------------------------------------
	int iSuffix = strlen(m_acSuffix);
	if(iSuffix > 0)
	{	char* pcSuffix = strcasestr(acBuf, m_acSuffix);
		if(pcSuffix != 0L) pcSuffix[0] = '\0';
	}
	else
	{	char* pcExt = strcasestr(acBuf, ".mrc");
		if(pcExt == 0L) pcExt = strcasestr(acBuf, ".tif");
		if(pcExt == 0L) pcExt = strcasestr(acBuf, ".eer");
		if(pcExt != 0L) pcExt[0] = '\0';
	}
	//--------------------------------------
	char* pcSerial = new char[256];
	memset(pcSerial, 0, sizeof(char) * 256);
	strcpy(pcSerial, acBuf);
	return pcSerial;
}
	
char* CStackFolder::mGetInPrefix(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->IsInMrc()) return pInput->m_acInMrcFile;
	else if(pInput->IsInTiff()) return pInput->m_acInTifFile;
	else if(pInput->IsInEer()) return pInput->m_acInEerFile;
	else return 0L;
}

bool CStackFolder::mGetDirName(void)
{
	char* pcPrefix = mGetInPrefix();
	char* pcSlash = strrchr(pcPrefix, '/');
	if(pcSlash == 0L)
	{	strcpy(m_acPrefix, pcPrefix);
		char* pcRet = getcwd(m_acDirName, sizeof(m_acDirName));
		strcpy(m_acDirName, "./");
		return true;
	}
	//------------------
	int iNumChars = pcSlash - pcPrefix + 1;
	memcpy(m_acDirName, pcPrefix, iNumChars);
	//---------------------------------------
	int iBytes = strlen(pcPrefix) - iNumChars;
	if(iBytes > 0) memcpy(m_acPrefix, pcSlash + 1, iBytes);
	return true;
}

void CStackFolder::mClean(void)
{
	CDataPackage* pDataPackage = 0L;
	while(!m_aFileQueue.empty())
	{	pDataPackage = m_aFileQueue.front();
		m_aFileQueue.pop();
		if(pDataPackage != 0L) delete pDataPackage;
	}
}

bool CStackFolder::mAsyncReadFolder(void)
{
	this->Start();
	return true;
}

void CStackFolder::ThreadMain(void)
{
	const char* pcMethod = "in CStackFolder::ThreadMain";
	m_ifd = inotify_init1(IN_NONBLOCK);
	m_iwd = inotify_add_watch(m_ifd, m_acDirName, IN_CLOSE_WRITE);
	if(m_ifd == -1)
	{	fprintf(stderr, "Error: unable to init inotify %s.\n\n",
		   pcMethod); return;
	}
	if(m_iwd == -1) 
	{	fprintf(stderr, "Error: add_watch failed %s.\n\n",
		   pcMethod);
		close(m_ifd); return;
	}
	//---------------------------
	CInput* pInput = CInput::GetInstance();
	int iCount = 0;
	int iEventSize = sizeof(inotify_event);
	char acEventBuf[4096] = {'\0'};
	inotify_event* pEvents = (inotify_event*)acEventBuf;
	//--------------------------------------------------
	bool bFirstTime = true;
	while(true)
	{	int iNumEvents = read(m_ifd, acEventBuf, 4096);
		if(iNumEvents <= 0)
		{	this->mWait(2.0f);
			iCount += 2; 
			if(iCount > pInput->m_iSerial) break;
			printf("no new files, wait for %d s\n",
			   pInput->m_iSerial - iCount);
			continue;
		}
		bool bCloseWrite = true;
		for(int i=0; i<iNumEvents; i++)
		{	inotify_event* pEvent = pEvents + i;
			bCloseWrite = pEvent->mask & IN_CLOSE_WRITE;
			if(bCloseWrite) break;
		}
		if(!bCloseWrite) continue; 
		//------------------------	
		int iNumFiles = mReadFolder(bFirstTime);
		if(iNumFiles > 0) iCount = 0;
		bFirstTime = false;
	}
	inotify_rm_watch(m_ifd, m_iwd);
	close(m_ifd);
}
