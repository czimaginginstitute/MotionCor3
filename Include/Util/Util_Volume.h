#pragma once
#include <sys/types.h>

class Util_Volume
{
public:
	
	Util_Volume(void);

	~Util_Volume(void);

	void SetSize(int iSizeX, int iSizeY, int iSizeZ);

	void SetSliceXY(float* pfData, int iZ);

	//----------------------------------------
	// When bCopy is false, caller should not
	// free the returned memory.
	//----------------------------------------
	float* GetSliceXY(int iZ, bool bCopy);

	void GetSliceXY(float* pfData, int iZ);

	void SetSliceXZ(float* pfData, int iY);

	void GetSliceXZ(float* pfData, int iY);

	//--------------------------------------------------
	// Caller should free the returned memory.
	//--------------------------------------------------
	Util_Volume* GetSubVolumeX(int iStartX, int iSizeX);

	void SetSubVolumeX(Util_Volume* pVol, int iStartX);

	//-------------------------------------------------
	// Caller should free the returned memory.
	//-------------------------------------------------
	Util_Volume* GetSubVolumeY(int iStartY, int iSizeY);

	void SetSubVolumeY(Util_Volume* pVol, int iStartY);

	void GetSubVolume(int* piStart, Util_Volume* pSubvol);

	void SetSubVolume(int* piStart, Util_Volume* pSubvol);

	//-------------------------------------------------
	// Caller should free the returned memory.
	//-------------------------------------------------
	float* GetLineX(int iY, int iZ);

	void SetLineX(float* pfData, int iY, int iZ);

	//-------------------------------------------------
	// Caller should free the returned memory.
	//-------------------------------------------------
	float* GetLineY(int iX, int iZ);

	void SetLineY(float* pfData, int iX, int iZ);

	//-------------------------------------------------
	// Caller should free the returned memory.
	//-------------------------------------------------
	float* GetLineZ(int iX, int iY);

	void SetLineZ(float* pfData, int iX, int iY);

	size_t GetVoxels(void);

	void DeleteVolume(void);

	void SetZero(void);

	void Copy(Util_Volume* pVolume);

	//-------------------------
	// Caller free the memory.
	//-------------------------
	Util_Volume* GetCopy(void);

	int m_iSize[3];

	float* m_pfData;

private:

	void mFree(void);

	void mAllocate(void);
};
