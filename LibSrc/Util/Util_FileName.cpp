#include "Util_FileName.h"
#include "Util_Number.h"
#include "Util_String.h"
#include <string.h>
#include <memory.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/statvfs.h>
#include <sys/types.h>

Util_FileName::Util_FileName(char* pcFileName)
{
	mSetFileName(pcFileName);
}

Util_FileName::~Util_FileName(void)
{
	delete[] m_pcFileName;
}

char* Util_FileName::GetMainName(void)
{
	char* pcMainName = Util_String::GetCopy(m_pcMainName);
	return pcMainName;
}

char* Util_FileName::GetExtName(void)
{
	char* pcExtName = Util_String::GetCopy(m_pcExtName);
	return pcExtName;
}

char* Util_FileName::InsertSerial(int iSerial)
{
	char cVal[16];
	if(iSerial < 10) sprintf(cVal, "000%d", iSerial);
	else if(iSerial < 100) sprintf(cVal, "00%d", iSerial);
	else if(iSerial < 1000) sprintf(cVal, "0%d", iSerial);
	else sprintf(cVal, "%d", iSerial);

	char* pcHome = getenv("HOME");

	int iSize = 128;
	char* pcNewName = new char[iSize];
	if(m_pcMainName == NULL) 
	{	strcpy(pcNewName, pcHome);
		strcat(pcNewName, "/test");
	}
	else strcpy(pcNewName, m_pcMainName);
	
	strcat(pcNewName, cVal);
	strcat(pcNewName, ".");
	if(m_pcExtName != NULL) strcat(pcNewName, m_pcExtName);
	return pcNewName;
}

char* Util_FileName::ReplaceExt(char* pcExt)
{
	int iSize = 128;
	char* pcNewName = new char[iSize];
	if(m_pcMainName == NULL) strcpy(pcNewName, "~/test");
	else strcpy(pcNewName, m_pcMainName);
	if(pcExt == NULL || strlen(pcExt) == 0) return pcNewName;
	strcat(pcNewName, ".");
	strcat(pcNewName, pcExt);
	return pcNewName;
}

char* Util_FileName::AppendText(char* pcText)
{
	int iSize = 128;
	char* pcNewName = new char[iSize];
	strcpy(pcNewName, m_pcMainName);
	if(pcText != NULL && strlen(pcText) > 0) strcat(pcNewName, pcText);
	if(m_pcExtName == NULL) return pcNewName;
	strcat(pcNewName, ".");
	strcat(pcNewName, m_pcExtName);
	return pcNewName;
}

int Util_FileName::GetFreeDiskSpace(void)
{	struct statvfs aStatvfs;
	int iStatus = statvfs("/", &aStatvfs);
	if(iStatus == -1) return 0;
	
	double dTotal = aStatvfs.f_bfree * (aStatvfs.f_bsize / 1024);
	int iTotal = (int)(dTotal / (1024 * 1024));
	return iTotal;
}

void Util_FileName::mSetFileName(char* pcFileName)
{
	int iSize = 128;
	m_pcMainName = NULL;
	m_pcExtName = NULL;
	m_pcFileName = new char[iSize];
	memset(m_pcFileName, 0, sizeof(char) * iSize);

	if(pcFileName == NULL || strlen(pcFileName) == 0) return;
	strcpy(m_pcFileName, pcFileName);
	char* pcNext = NULL;
	m_pcMainName = strtok(m_pcFileName, ".");
	m_pcExtName = strtok(NULL, ".");
}
