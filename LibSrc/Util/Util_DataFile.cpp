#include "Util_DataFile.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <memory.h>
#include <sys/stat.h>
#include <fcntl.h>

Util_DataFile::Util_DataFile(void)
{
	m_iFile = -1;
	m_iNumRows = 0;
	m_iNumCols = 0;
	m_pfValues = 0L;
	m_ppcRows = 0L;
}

Util_DataFile::~Util_DataFile(void)
{
	mFreeBuffer();
}

bool Util_DataFile::ReadIt(char* pcFileName)
{
	mFreeBuffer();
	if(m_pfValues != 0L) delete[] m_pfValues;
	if(pcFileName == NULL || strlen(pcFileName) == 0) 
	{	printf("Error: Invalid file name\n");
		return false;
	}
	//-------------------
	m_iFile = open(pcFileName, O_RDONLY);
	if(m_iFile == -1) 
	{	fprintf
		(  stderr, "Util_DataFile: %s\n, %s\n",
		   "cannot open file.", pcFileName
		);
		return false;
	}
	//-------------------
	mReadFile();
	close(m_iFile);
	m_iFile = -1;
	return true;
}

int Util_DataFile::GetNumCols(void)
{
	return m_iNumCols;
}

int Util_DataFile::GetNumRows(void)
{
	return m_iNumRows;
}

float Util_DataFile::GetValue(int iRow, int iCol)
{
	if(iRow < 0 || iRow >= m_iNumRows) return 0.0f;
	if(iCol < 0 || iCol >= m_iNumCols) return 0.0f;
	return m_pfValues[iRow * m_iNumCols + iCol];
}

float* Util_DataFile::GetNthCol(int iNthCol)
{
	if(iNthCol < 0 || iNthCol >= m_iNumCols) return 0L;
	float* pfCol = new float[m_iNumRows];
	for(int i=0; i<m_iNumRows; i++)
	{	pfCol[i] = this->GetValue(i, iNthCol);
	}
	return pfCol;
}

float* Util_DataFile::GetNthRow(int iNthRow)
{
	if(iNthRow < 0 || iNthRow >= m_iNumRows) return 0L;
	float* pfRow = new float[m_iNumCols];
	for(int i=0; i<m_iNumCols; i++)
	{	pfRow[i] = this->GetValue(iNthRow, i);
	}
	return pfRow;
}

ssize_t Util_DataFile::mGetFileSize(void)
{
    if(m_iFile == -1) return 0;
    struct stat aStat;
    fstat(m_iFile, &aStat);
    return aStat.st_size;
}


void Util_DataFile::mReadFile(void)
{
	ssize_t iCount = mGetFileSize();
	if(iCount == 0) return;
	char* pcBuf = new char[iCount + 1];
	memset(pcBuf, 0, iCount + 1);
	
	ssize_t iNumRead = read(m_iFile, pcBuf, iCount);
	if(iNumRead != iCount)
	{	delete[] pcBuf;
		return;
	}
	mParseFile(pcBuf, iCount);
	delete[] pcBuf;
}

void Util_DataFile::mParseFile(char* pcBuf, ssize_t iSize)
{
	mDetNumRows(pcBuf);
	mDetNumCols();
	int iNumElems = m_iNumRows * m_iNumCols;
	if(iNumElems == 0)
	{	mFreeBuffer();
		return;
	}
	m_pfValues = new float[iNumElems];
	memset(m_pfValues, 0, sizeof(float) * iNumElems);
	int iNumRows = m_iNumRows;
	m_iNumRows = 0;
	for(int i=0; i<iNumRows; i++) mParseNthRow(i);
	delete[] m_ppcRows;
	m_ppcRows = 0L;
}

void Util_DataFile::mDetNumRows(char* pcBuf)
{
	int iNumRows = 1;
	int iSize = strlen(pcBuf);
	for(int i=0; i<iSize; i++)
	{	if(pcBuf[i] == '\n' || pcBuf[i] == '\r') iNumRows++;
		else continue;
	}
	m_ppcRows = new char*[iNumRows];
	memset(m_ppcRows, 0, sizeof(char*) * iNumRows);

	char* pcToken = strtok(pcBuf, "\n\r");
	for(int i=0; i<iNumRows; i++) 
	{	if(pcToken != 0L) 
		{	m_ppcRows[m_iNumRows] = pcToken;
			m_iNumRows++;
		}
		pcToken = strtok(NULL, "\n\r");
	}
}

void Util_DataFile::mDetNumCols(void)
{
	for(int i=0; i<m_iNumRows; i++)
	{	char* pcRow = mGetNthRow(i);
		m_iNumCols = mDetNumElems(pcRow);
		delete[] pcRow;
		if(m_iNumCols > 0) break;
	}
}

int Util_DataFile::mDetNumElems(char* pcRow)
{
	if(pcRow == 0L || strlen(pcRow) == 0) return 0;
	char cDelimit[] = ", \t";
	char* pcToken = strtok(pcRow, cDelimit);
	int iNumElements = 0;
	while(pcToken != 0L)
	{	iNumElements++;
		pcToken = strtok(NULL, cDelimit);
	}
	return iNumElements;
}

char* Util_DataFile::mGetNthRow(int iNthRow)
{
	if(m_ppcRows[iNthRow] == 0L) return 0L;
	int iSize = strlen(m_ppcRows[iNthRow]) + 1;
	char* pcLine = new char[iSize];
	strcpy(pcLine, m_ppcRows[iNthRow]);
	return pcLine;
}

void Util_DataFile::mParseNthRow(int iNthRow)
{
	char* pcRow = mGetNthRow(iNthRow);
	int iNumElems = mDetNumElems(pcRow);
	delete[] pcRow;
	if(iNumElems != m_iNumCols) return;

	char cDelimit[] = ", \t";
	pcRow = mGetNthRow(iNthRow);
	char* pcToken = strtok(pcRow, cDelimit);
	int iIndex = 0;
	while(pcToken != NULL)
	{	m_pfValues[m_iNumRows * m_iNumCols + iIndex] = (float)atof(pcToken);
		iIndex++;
		pcToken = strtok(NULL, cDelimit);
	}
	m_iNumRows++;
}

void Util_DataFile::mFreeBuffer(void)
{
	if(m_ppcRows != 0L) delete[] m_ppcRows;
	if(m_pfValues != 0L) delete[] m_pfValues;
	m_ppcRows = 0L;
	m_pfValues = 0L;
	m_iNumRows = 0;
	m_iNumCols = 0;
}
