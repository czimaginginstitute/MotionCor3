#include "CMainInc.h"
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/file.h>
#include <unistd.h>
#include <sys/types.h>
#include <signal.h>
#include <string.h>

using namespace MotionCor2;

CCheckFreeGpus::CCheckFreeGpus(void)
{
	strcpy(m_acGpuFile, "/tmp/MotionCor2_FreeGpus.txt");
	m_piGpuIds = 0L;
	m_piPids = 0L;
	m_iNumGpus = 0;
}

CCheckFreeGpus::~CCheckFreeGpus(void)
{
	mClean();
}

void CCheckFreeGpus::SetAllGpus(int* piGpuIds, int iNumGpus)
{
	mClean();
	m_piGpuIds = new int[iNumGpus];
	m_piPids = new int[iNumGpus];
	m_iNumGpus = iNumGpus;
	//--------------------
	memcpy(m_piGpuIds, piGpuIds, sizeof(int) * m_iNumGpus);
	memset(m_piPids, 0, sizeof(int) * m_iNumGpus);
	m_iPid = getpid();
}

int CCheckFreeGpus::GetFreeGpus(int* piFreeGpus, int iNumFreeGpus)
{
	int iFd = mOpenFile();
	if(iFd == -1) return -1;
	//----------------------
	int iRet = mLockFile(iFd);
	if(iRet == -1) 
	{	close(iFd);
		return -1;
	}
	//---------------- 
	FILE* pFile = fdopen(iFd, "w+t");
	mReadGpus(pFile);
	//---------------
	int iFreeGpus = 0;
	for(int i=0; i<m_iNumGpus; i++)
	{	if(mCheckActivePid(m_piPids[i])) continue;
		piFreeGpus[iFreeGpus] = m_piGpuIds[i];
		m_piPids[i] = m_iPid;
		iFreeGpus++;
		if(iFreeGpus == iNumFreeGpus) break;
	}
	if(iFreeGpus > 0) mWriteGpus(pFile);
	//----------------------------------
	mUnlockFile(iFd);
	fclose(pFile);
	return iFreeGpus;
}

void CCheckFreeGpus::FreeGpus(void)
{
	int iFd = mOpenFile();
	if(iFd == -1) return;
	//-------------------
	int iRet = mLockFile(iFd);
	if(iRet == -1)
	{	close(iFd);
		return;
	}
	//-------------
	for(int i=0; i<m_iNumGpus; i++)
	{	int iPid = m_piPids[i];
		if(iPid == 0) continue;
		if(iPid == m_iPid) m_piPids[i] = 0;
		else if(!mCheckActivePid(iPid)) m_piPids[i] = 0;
	}
	FILE* pFile = fdopen(iFd, "w+t");
	mWriteGpus(pFile);
	//----------------
	mUnlockFile(iFd);
	fclose(pFile);
}
 
void CCheckFreeGpus::mClean(void)
{
	if(m_piGpuIds != 0L) delete[] m_piGpuIds;
	if(m_piPids != 0L) delete[] m_piPids;
	m_piGpuIds = 0L;
	m_piPids = 0L;
}

int CCheckFreeGpus::mOpenFile(void)
{
	int iFd = open(m_acGpuFile, O_RDWR | O_CREAT, 0666);
	chmod(m_acGpuFile, 0666);
	if(iFd > -1) return iFd;
	//----------------------
	printf
	( "Warning: Unable to open free GPU file: %s.\n"
	  "         %s\n\n", m_acGpuFile, strerror(iFd)
	);
	return -1;
}

int CCheckFreeGpus::mLockFile(int iFd)
{
	int iRet = flock(iFd, LOCK_EX);
	if(iRet == 0) return iRet;
	printf
	( "Warning: Unable to lock GPU file in CCheckFreeGpus::mLockFile.\n\n"
	);
	return -1;
}

int CCheckFreeGpus::mUnlockFile(int iFd)
{
	int iRet = flock(iFd, LOCK_UN);
	if(iRet == 0) return iRet;
	printf
	( "Warning: Unable to unlock GPU file in CCheckFreeGpus::mLockFile.\n\n"
	);
	return -1;
}

void CCheckFreeGpus::mReadGpus(FILE* pFile)
{
	char acLine[256] = {'\0'};
	while(!feof(pFile))
	{	char* pcRet = fgets(acLine, 256, pFile);
		if(pcRet == 0L) continue;
		int iGpuId = 0, iPid = 0;
		int iEntries = sscanf(acLine, "%d %d", &iGpuId, &iPid);
		if(iEntries != 2) continue;
		//-------------------------
		if(iPid == 0) continue;
		else if(!mCheckActivePid(iPid)) continue;
		//---------------------------------------
		for(int i=0; i<m_iNumGpus; i++)
		{	if(iGpuId != m_piGpuIds[i]) continue;
			m_piPids[i] = iPid;
		}
	}
}

void CCheckFreeGpus::mWriteGpus(FILE* pFile)
{
	fseek(pFile, 0, SEEK_SET);
	char acEntry[64] = {'\0'};
	for(int i=0; i<m_iNumGpus; i++)
	{	sprintf(acEntry, "%d  %d", m_piGpuIds[i], m_piPids[i]);
		fprintf(pFile, "%-16s\n", acEntry);
	}
}

bool CCheckFreeGpus::mCheckActivePid(int iPid)
{
	if(iPid <= 0) return false;
	int iRet = kill(iPid, 0);
	if(iRet == 0) return true;
	//------------------------
	int iGroupID = getpgid(iPid);
	if(iGroupID == -1) return false;	
	else return true;
}
