#pragma once

class Util_Patch2D
{
public:
	Util_Patch2D(void);
	~Util_Patch2D(void);
	void Setup
	(  int*piImgSize, 
	   int* piPatchSize,
	   float fOverlap
	);
	void DoIt
	(  void* pvImage, 
	   int iMrcMode
	);
	void* GetPatch
	(  int iPatch,
	   bool bClean
	);
	void GetCenter
	(  int iPatch,
	   int* piCent
	);
	void GetCenter
	(  int iPatch,
	   float* pfCent
	);
	float* GetCenterX(void);
	float* GetCenterY(void);
	int m_aiPatSize[2];
	int m_iNumPatches;
	int m_iMrcMode;
private:
	void mClean(void);
	void mCalcNumPatches(void);
	void mExtractPatch(int iPatch);
	void* m_pvImg;
	int m_aiImgSize[2];
	float m_fOverlap;
	void** m_ppvPatches;
	int* m_piStartXs;
	int* m_piStartYs;
	int m_aiPatches[2];
};
