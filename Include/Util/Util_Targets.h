#pragma once

//=============================================================================
//
// A target is a pair of floating numbers that can represent, for example,
// a point in an image.
//
//=============================================================================
class Util_Targets
{
public:

	//---------------------------------------------------------------
	// By default, each target is made of two floats.
	//---------------------------------------------------------------
	Util_Targets(void);

	//---------------------------------------------------------------
	// iTargetLength specifies number of floats in each target.
	//---------------------------------------------------------------
	Util_Targets(int iTargetLength);

	~Util_Targets(void);

	//---------------------------------------------------------------
	// Add the new target to the end of the target list.
	//---------------------------------------------------------------
	void AddTarget(float* pfTarget);

	//---------------------------------------------------------------
	// Remove the Nth target and the target buffer will be resorted.
	//---------------------------------------------------------------
	void RemoveNthTarget(int iNthTarget);

	//---------------------------------------------------------------
	// Remove all the targets.
	//---------------------------------------------------------------
	void RemoveAll(void);

	//---------------------------------------------------------------
	// Return the Nth target, do not free the memory.
	//---------------------------------------------------------------
	float* GetNthTarget(int iNthTarget);

	//---------------------------------------------------------------
	// Return the index of the target closest to the input one or -1
	// if the buffer is empty.
	//---------------------------------------------------------------
	int FindClosest(float* pfTarget);

	//---------------------------------------------------------------
	// Copy the targets contained in the input object.
	//---------------------------------------------------------------
	void CopyTargets(Util_Targets* pTargets);

	//---------------------------------------------------------------
	// Number of currently stored targets
	//---------------------------------------------------------------
	int m_iNumTargets;

	//---------------------------------------------------------------
	// Number of floats in each target
	//---------------------------------------------------------------
	int m_iTargetLength;

private:

	void mExpandBuffer(void);

	float** mCreateBuffer(int Size);

	float mCalcDist(float* pfSrc, float* pfTgt);

	void mRemoveNull(int iIndex);

	void mFreeBuffer(void);

	int m_iBufSize;

	float** m_ppfTargets;
};
