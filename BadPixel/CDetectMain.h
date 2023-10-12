#pragma once
#include "../DataUtil/CDataUtilInc.h"

namespace MotionCor2
{
namespace BadPixel
{

class CTemplate
{
public:
	CTemplate(void);
	~CTemplate(void);
	void Create(int* piSize, float* pfMod);
        int m_aiSize[2];
private:
        void mNormalize(float* pfTemplate);
};	//CTemplate


class GLocalCC
{
public:
	GLocalCC(void);
	~GLocalCC(void);
	void SetRef(float* gfRef, int* piRefSize);
	void DoIt
	( float* gfPadImg, 
	  int* piPadSize,
	  int iOffset, 
	  int iPartSize,
	  float* gfPadCC,
	  cudaStream_t stream
	);
private:
	float* m_gfRef;
};	//GLocalCC

//-------------------------------------------------------------------
// 1. Given bad pixel patch template (m_pfMod), it is used to perform
//    local correlation on the input image.
// 2. The output is the correlation map m_pfCC. It has the same
//    size of the input image.
//-------------------------------------------------------------------
class CLocalCCMap 
{
public:
	CLocalCCMap(void);
	~CLocalCCMap(void);
	static void DoIt(int* piModSize);
private:
	void mDoIt(int iNthGpu);
	void mWait(void);
	//---------------
	float* m_gfPadSum;
	float* m_gfPadCC;
	float* m_pfMod;
	int m_aiPadSize[2];
	cudaStream_t m_aStream;
	int m_iNthGpu;
};

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
class GDetectPatch
{
public:
	GDetectPatch(void);
	~GDetectPatch(void);
	void DoIt
	( float* gfPadSum, 
	  float* gfPadCC,
	  float* gfPadBuf,
	  unsigned char* pucBadMap,
	  int* piPadSize,
	  int* piModSize,
	  float fStdThreshold
	);
private:
	void mUpdateBadMap
	( float* gfCCMap,
	  unsigned char* pucBadMap,
	  int* piPadSize,
	  int* piModSize
	);
	float m_fCCThreshold;
};


class GDetectHot
{
public:
        GDetectHot(void);
        ~GDetectHot(void);
        void DoIt
	( float* gfPadSum,
	  float* gfPadBuf, 
	  int* piPadSize, 
	  float fStdThreshold,
	  unsigned char* pucBadMap
	);
        int m_iNumHots;
};

class GCombineMap
{
public:
	GCombineMap(void);
	~GCombineMap(void);
	
	void GDoIt
	(  unsigned char* gucMap1,
	   unsigned char* gucMap2,
	   unsigned char* gucResMap,
	   int* piMapSize
	);
	unsigned char* GCopyMap
	(  unsigned char* pucMap,
	   int* piMapSize
	);
};

class GLabelPatch
{
public:
        GLabelPatch(void);
        ~GLabelPatch(void);
        void SetLabelSize(int iRadius);
        void DoIt
	(  float* gfImg, 
	   int* piImgSize,
	   int* piPatchList, 
	   int iNumPatches
	);
        int m_aiImgSize[2];
private:
        int m_iRadius;
};	//CLabelDefect


class CSumStack
{
public:
        CSumStack(void);
        ~CSumStack(void);
	float* DoIt
	(  DataUtil::CMrcStack* pMrcStack,
	   int* piGpuIds, int iNumGpus
	);
private:
        void mAddUChar(void* pvFrame);
        void mAddFloat(void* pvFrame);
        int m_iMrcMode;
};


class CDetectMain
{
public:
	static CDetectMain* GetInstance(void);
	static void DeleteInstance(void);
	~CDetectMain(void);
	void DoIt(void);
	unsigned char* GetDefectMap(bool bClean);
private:
	CDetectMain(void);
	void mDetectPatch(void);
	void mDetectHot(void);
	void mLabelDefects(float* gfImg, int* piDefects, int iNumDefects);
	void mLoadDefectFile(void);
	void mLoadDefectMap(void);
	bool mLoadDefectMapMRC(void);
	bool mLoadDefectMapTIFF(void);
	unsigned char* m_pucBadMap;
	float m_fThreshold;
	int m_aiPadSize[2];
	int m_aiDefectSize[2];
	static CDetectMain* m_pInstance;
};

}}
