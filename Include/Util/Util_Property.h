#pragma once

//=============================================================================
//
// This class is specific for Util_Property.
//
//=============================================================================
class Util_Entry
{
public:
	
	Util_Entry(void);

	~Util_Entry(void);

	void SetKey(char* pcKey);

	void AddVal(char* pcVal);

	bool IsSame(char* pcKey);
	// -----------------------
	// must be freed by caller
	// -----------------------
	char* GetValue(int iNthVal);
	// -----------------
	// get numeric value
	// -----------------
	double GetDValue(int iNthVal);
	// -----------------
	// get boolean value
	// -----------------
	bool GetBValue(int iNthVal);

	void Clean(void);

	char m_acKey[64];

	char** m_ppcVals;
	
	int m_iNumValues;
};

class Util_Property
{
public:

	Util_Property(void);

	Util_Property(char* pcFileName);
	
	~Util_Property(void);

	void SetFileName(char* pcFileName);
	// ------------------------------------
	// query number of values in this entry
	// ------------------------------------
	int GetNumValues(char* pcKey);
	// --------------------------------
	// get the first value in the entry
	// --------------------------------
	double GetValue(char* pcKey);
	// ----------------------------------------
	// get the first boolean value in the entry
	// ----------------------------------------
	bool GetBooleanValue(char* pcKey);
	// ------------------------------
	// get the nth value in the entry
	// caller free the memory.
	// ------------------------------
	char* GetStringValue(char* pcKey, int iNthVal); 
	// -------------------------------
	// get all the values in the entry 
	// nust call GetNumValues first.
	// -------------------------------
	void GetIValues(char* pcKey, int* piVals);

	void GetFValues(char* pcKey, float* pfVals);

	void GetDValues(char* pcKey, double* pdVals); 
	
	int GetNumEntries(void) { return m_iNumEntries; }

private:

	void mParseLine(char* pcLine);

	void mRemoveSpace(char* pcLine, char* pcBuf);

	void mAddValue(Util_Entry* pEntry, char* pcVal);

	int mGetIndex(char* pcKey);

	int mExpandBuffer(int iBufSize);

	void mClean(void);

	int m_iNumEntries;

	Util_Entry** m_ppEntries;
};
