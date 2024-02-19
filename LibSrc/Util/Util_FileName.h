#pragma once

class Util_FileName
{
public:

	Util_FileName(char* pcFileName);


	~Util_FileName(void);

	//---------------------------------------------------------------
	// Extract the string before file extension. For example, if the
	// input file name is "/temp/test.mrc", this method returns
	// "/temp/test". Do not free memory of returned string.
	//---------------------------------------------------------------
	char* GetMainName(void);			/* [out] */

	//---------------------------------------------------------------
	// Return the file extension. Do not free memory of returned
	// string.
	//---------------------------------------------------------------
	char* GetExtName(void);				/* [out] */

	//---------------------------------------------------------------
	// Insert a serial number before file extension. For example, if
	// the file name is "/temp/test.mrc" and the serial number is
	// 10, the return is "/temp/test0010.mrc". Caller free memory.
	//---------------------------------------------------------------
	char* InsertSerial(int iSerial);		/* out */

	char* ReplaceExt(char* pcExt);			/* out */

	//---------------------------------------------------------------
	// Append the text before the file extension. For example, if the
	// file name is "/temp/test.mrc" and the text is "tgt", the
	// return is "/temp/testtgt.mrc". Caller frees memory.
	//---------------------------------------------------------------
	char* AppendText(char* pcText);			/* [out] */

	//---------------------------------------------------------------
	// Returns the free space in GB of the drive where the file will
	// be written.
	//---------------------------------------------------------------
	int GetFreeDiskSpace(void);

private:

	void mSetFileName(char* pcFileName);

	char* m_pcFileName;

	char* m_pcMainName;

	char* m_pcExtName;
};
