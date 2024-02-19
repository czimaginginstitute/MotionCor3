#pragma once

#include <sys/types.h>

class Util_DataFile
{
public:

	Util_DataFile(void);

	~Util_DataFile(void);

	bool ReadIt(char* pcFileName);
	float GetValue(int iRow, int iCol);

	int GetNumRows(void);

	int GetNumCols(void);

	float* GetNthRow(int iNthRow);	/* out */

	float* GetNthCol(int iNthCol);	/* out */

private:

    ssize_t mGetFileSize(void);    
	void mCloseFile(void);

	void mReadFile(void);

	void mParseFile(char* pcBuf, ssize_t iSize);

	void mDetNumRows(char* pcBuf);

	void mDetNumCols(void);

	int mDetNumElems(char* pcRow);

	char* mGetNthRow(int iNthRow);

	void mParseNthRow(int iNthRow);

	void mFreeBuffer(void);

	void mAllocBuffer(void);

	int m_iNumRows;

	int m_iNumCols;

	float* m_pfValues;

	char** m_ppcRows;

    int m_iFile;
};
