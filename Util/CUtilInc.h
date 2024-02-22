#pragma once
#include <Mrcfile/CMrcFileInc.h>
#include <cufft.h>
#include <pthread.h>

namespace MotionCor2
{
namespace Util
{
//-------------------------------------------------------------------
// 1. The following functions are define in CSimpleFuncs.cpp
//-------------------------------------------------------------------
size_t GetUCharBytes(int* piSize);
size_t GetFloatBytes(int* piSize);
size_t GetCmpBytes(int* piSize);

unsigned char* GGetUCharBuf(int* piSize, bool bZero);
unsigned char* CGetUCharBuf(int* piSize, bool bZero);

float* GGetFloatBuf(int* piSize, bool bZero);
float* CGetFloatBuf(int* piSize, bool bZero);

cufftComplex* GGetCmpBuf(int* piSize, bool bZero);
cufftComplex* CGetCmpBuf(int* piSize, bool bZero);

void* GetPinnedBuf(int* piSize, int iPixelBytes, bool bZero);
void* GetGpuBuf(int* piSize, int iPixelBytes, bool bZero);

unsigned char* GCopyFrame(unsigned char* pucSrc, int* piSize, 
   cudaStream_t stream=0);
unsigned char* CCopyFrame(unsigned char* pucSrc, int* piSize, 
   cudaStream_t stream=0);
void CopyFrame(unsigned char* pucSrc, unsigned char* pucDst, 
   int* piSize, cudaStream_t stream=0);

float* GCopyFrame(float* pfSrc, int* piSize, cudaStream_t stream=0);
float* CCopyFrame(float* pfSrc, int* piSize, cudaStream_t stream=0);
void CopyFrame(float* pfSrc, float* pfDst, int* piSize, cudaStream_t stream=0);

cufftComplex* GCopyFrame(cufftComplex* pCmpSrc, int* piSize, 
   cudaStream_t stream=0);
cufftComplex* CCopyFrame(cufftComplex* pCmpSrc, int* piSize, 
   cudaStream_t stream=0);
void CopyFrame(cufftComplex* pCmpSrc, cufftComplex* pCmpDst, 
   int* piSize, cudaStream_t stream=0);

size_t GetGpuMemory(int iGpuId);

int CalcNumGpuFrames(int* piFrmSize, int iGpuId, double dOccupance);

void PrintGpuMemoryUsage(const char* pcInfo);
float GetGpuMemoryUsage(void);

void CheckCudaError(const char* pcLocation);
void CheckRUsage(const char* pcLocation);

class GPad2D
{
public:
	void Pad
	( float* gfImg, 
	  int* piImgSize, 
	  float* gfPadImg,
	  cudaStream_t stream
	);
	void Unpad
	( float* gfPadImg, 
	  int* piPadSize, 
	  float* gfImg,
	  cudaStream_t stream = 0
	);
};

//-------------------------------------------------------------------
class CParseArgs
{
public:
        CParseArgs(void);
        ~CParseArgs(void);
        void Set(int argc, char* argv[]);
        bool FindVals(const char* pcTag, int aiRange[2]);
        void GetVals(int aiRange[2], float* pfVals);
        void GetVals(int aiRange[2], int* piVal);
        void GetVal(int iArg, char* pcVal);
        void GetVals(int aiRange[2], char** ppcVals);
private:
        char** m_argv;
        int m_argc;
};

class CCufft2D
{
public:
	CCufft2D(void);
	~CCufft2D(void);
	void CreateForwardPlan(int* piSize, bool bPad);
	void CreateInversePlan(int* piSize, bool bCmp);
	void DestroyPlan(void);
	bool Forward
	( float* gfPadImg, cufftComplex* gCmpImg,
	  bool bNorm, cudaStream_t stream = 0
	);
	bool Forward
	( float* gfPadImg, bool bNorm, 
	  cudaStream_t stream = 0
	);
	cufftComplex* ForwardH2G(float* pfImg, bool bNorm);
	bool Inverse
	( cufftComplex* gCom, float* gfPadImg, 
	  cudaStream_t stream = 0
	);
	bool Inverse(cufftComplex* gCom, cudaStream_t stream=0);
	float* InverseG2H(cufftComplex* gCmp);
	void SubtractMean(cufftComplex* gComplex);
private:
	bool mCheckError(cufftResult* pResult, const char* pcFormat);
	const char* mGetErrorEnum(cufftResult error);
	cufftHandle m_aPlan;
	cufftType m_aType;
	int m_iFFTx;
	int m_iFFTy;
};

//-------------------------------------------------------------------
class GFFT1D
{
public:
	GFFT1D(void);
	~GFFT1D(void);
	void DestroyPlan(void);
	void CreatePlan
	( int iFFTSize,
	  int iNumLines,
	  bool bForward
	);
	void Forward(float* gfPadLines,bool bNorm);
	void Inverse(cufftComplex* gCmpLines);
private:
	int m_iFFTSize;
	int m_iNumLines;
	cufftType m_cufftType;
	cufftHandle m_cufftPlan;
};

class GFFTUtil2D
{
public:
	GFFTUtil2D(void);
	~GFFTUtil2D(void);
	//-----------------
	void Multiply
	( cufftComplex* gComp, int* piCmpSize, 
	  float fFactor, cudaStream_t stream = 0
	);
	void GetAmp
	( cufftComplex* gComp, int* piCmpSize,
	   float* pfAmpRes, bool bGpuRes,
           cudaStream_t stream = 0
	);
	void Shift
	( cufftComplex* gComp, int* piCmpSize,
	  float* pfShift,
          cudaStream_t stream = 0
	);
	void Lowpass
	( cufftComplex* gInCmp, cufftComplex* gOutCmp,
	  int* piCmpSize, float fBFactor,
	  cudaStream_t stream = 0
	);
};

class GRoundEdge
{
public:
	GRoundEdge(void);
	~GRoundEdge(void);
	void SetMask(float* pfCent, float* pfSize);
	void SetScale(float fScale);
	void DoIt(float* gfImg, int* piImgSize);
private:
	float m_afMaskCent[2];
	float m_afMaskSize[2];
	float m_fScale;
};	//GRoundEdge

class GNormalize2D
{
public:
	GNormalize2D(void);
	~GNormalize2D(void);
	void DoIt
	( float* gfImg, int* piSize, bool bPadded,
	  float fMean, float fStd,
	  cudaStream_t stream = 0
	);
};

class GThreshold2D
{
public:
	GThreshold2D(void);
	~GThreshold2D(void);
	void DoIt
	( float* gfImg, int* piImgSize, bool bPadded,
	  float fMin, float fMax
	);
};

class GFourierResize2D
{
public:
	GFourierResize2D(void);
	~GFourierResize2D(void);
	static void GetBinnedCmpSize
	(  int* piCmpSize,// cmp size before binning
	   float fBin,
	   int* piNewSize // cmp size after binning
	);
	static void GetBinnedImgSize
	(  int* piImgSize, // img size before binning
	   float fBin,
	   int* piNewSize
	);
	static float CalcPixSize
	(  int* piImgSize, // img size before binning
	   float fBin,
	   float fPixSize  // before binning
	);
	static void GetBinning
	(  int* piCmpSize,  // cmp size before binning
	   int* piNewSize,  // cmp size after binning
	   float* pfBinning
	);
	void DoIt
	( cufftComplex* gCmpIn, 
	  int* piSizeIn,
	  cufftComplex* gCmpOut, 
	  int* piSizeOut,
	  bool bSum,
	  cudaStream_t stream = 0
	);
};

class GAddFrames
{
public:
	GAddFrames(void);
	~GAddFrames(void);
	void DoIt
	(  float* gfFrame1,
	   float fFactor1,
	   float* gfFrame2,
	   float fFactor2,
	   float* gfSum,
	   int* piFrmSize,
           cudaStream_t stream=0
	);
	void DoIt
	(  cufftComplex* gCmp1,
	   float fFactor1,
	   cufftComplex* gCmp2,
	   float fFactor2,
	   cufftComplex* gCmpSum,
	   int* piCmpSize,
           cudaStream_t stream=0
	);
	void DoIt
	(  unsigned char* gucFrm1,
	   unsigned char* gucFrm2,
	   unsigned char* gucSum,
	   int* piFrmSize,
	   cudaStream_t stream=0
	);
};

class CFourierCrop2D
{
public:
	CFourierCrop2D(void);
	~CFourierCrop2D(void);
	void Clear(void);
	void Setup(int* piCmpSize, float fBin);
	void Setup(int* piCmpSizeIn, int* piCmpSizeOut);
	void DoIt
	( cufftComplex* gCmpFrm, 
	  cufftComplex* gCmpBuf, 
	  float* gfImg,
	  cudaStream_t stream
	);
	static void GetCmpSize
	( int* piCmpSizeIn, 
	  float fBin, 
	  int* piCmpSizeOut
	);
	static void GetImgSize
	( int* piCmpSizeIn,
	  float fBin,
	  int* piImgSizeOut
	);
	int m_aiOutCmpSize[2];
private:
	CCufft2D m_aCufft2D;
	int m_aiInCmpSize[2];
};

class GPositivity2D
{
public:
	GPositivity2D(void);
	~GPositivity2D(void);
	void DoIt(float* gfImg, int* piImgSize, cudaStream_t stream = 0);
	void AddVal(float* gfImg, int* piImgSize, float fVal,
	   cudaStream_t stream = 0);
};

class GGriddingCorrect
{
public:
	GGriddingCorrect(void);
	~GGriddingCorrect(void);
	void DoCmp
	( cufftComplex* gCmp, 
	  int* piCmpSize,
          cudaStream_t stream=0
	);
};   

class CPad2D
{
public:
	CPad2D(void);
	~CPad2D(void);

	static void GetPadSize(int* piImgSize, int* piPadSize);
	static void GetImgSize(int* piPadSize, int* piImgSize);

	static float* GGetPadBuf(int* piImgSize, bool bZero);
	static float* CGetPadBuf(int* piImgSize, bool bZero);

	static float* GGetImgBuf(int* piPadSize, bool bZero);
	static float* CGetImgBuf(int* piPadSize, bool bZero);

	float* GPad(float* pfImg, int* piSize);
	float* CPad(float* pfImg, int* piSize);	
	void Pad(float* pfImg, int* piImgSize, float* pfPad);

	float* GUnpad(float* pfPad, int* piSize);
	float* CUnpad(float* pfPad, int* piSize);
	void Unpad(float* pfPad, int* piPadSize, float* pfImg);

};	//CPad2D

class CFlipImage
{
public:
	CFlipImage(void);
	~CFlipImage(void);
	void DoIt(float* pfImg, int* piImg);
	float* DoIt(float* pfImg, int* piImg, bool bClean);
};

class CSaveTempMrc
{
public:
	CSaveTempMrc(void);
	~CSaveTempMrc(void);
	void SetFile(char* pcMain, const char* pcMinor);
	void GDoIt(cufftComplex* gCmp, int* piCmpSize);
        void GDoIt(float* gfImg, int* piSize);
        void GDoIt(unsigned char* gucImg, int* piSize);
        void DoIt(void* pvImg, int iMode, int* piSize);
private:
        char m_acMrcFile[256];
};	//CSaveTempMrc

//-------------------------------------------------------------------
// 1. Divide a stack of frames into multiple groups of frames. Each
//    group will be summed in relevant classes to form a reduced
//    stack containing these sums before motion measurement. 
//-------------------------------------------------------------------
class CGroupFrames
{
public:
        CGroupFrames(void);
        ~CGroupFrames(void);
        void DoGroupSize(int iNumFrames, int iGroupSize);
        void DoNumGroups(int iNumFrames, int iNumGroups);
        int GetGroupStart(int iGroup);
        int GetGroupSize(int iGroup);
        int GetNumGroups(void);
        int GetNumFrames(void);
        int GetGroup(int iFrame);
private:
        void mClean(void);
        void mGroup(void);
        int m_iNumFrames;
        int m_iGroupSize;
        int m_iNumGroups;
        int* m_piGroupStarts;
        int* m_piGroupSizes;
};

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
class CNextItem
{
public:
	CNextItem(void);
	~CNextItem(void);
	void Create(int iNumItems);
	void Reset(void);
	int GetNext(void);
	int GetNumItems(void);
private:
	int m_iNumItems;
	int m_iNextItem;
	pthread_mutex_t m_aMutex;
};

class GCalcMoment2D
{
public:
	GCalcMoment2D(void);
	~GCalcMoment2D(void);
	void Clean(void);
	void SetSize(int* piImgSize, bool bPadded);
	float DoIt
	( float* gfImg,
	  int iExponent,
	  bool bSync,
	  cudaStream_t stream = 0
	);
	float GetResult(void);
	void Test(float* gfImg, float fExp);
private:
	void mDoIt(float* gfImg, float* gfSum, cudaStream_t stream);
	int m_aiImgSize[2];
	int m_iPadX;
	float* m_gfBuf;
	dim3 m_aGridDim;
	dim3 m_aBlockDim;
};

class GFindMinMax2D
{
public:
	GFindMinMax2D(void);
	~GFindMinMax2D(void);
	void Clean(void);
	void SetSize(int* piImgSize, bool bPadded);
	float DoMin(float* gfImg, bool bSync, cudaStream_t stream = 0);
	float DoMax(float* gfImg, bool bSync, cudaStream_t stream = 0);
	float GetResult(void);
	void Test(float* gfImg);
private:
	int m_aiImgSize[2];
	int m_iPadX;
	float* m_gfBuf;
	dim3 m_aGridDim;
	dim3 m_aBlockDim;
};

class CSplineFit1D
{
public:
	CSplineFit1D(void);
	~CSplineFit1D(void);
	void Clean(void);
	void SetNumKnots(int iNumKnots);
	bool DoIt(float* pfX, float* pfY, bool* pbBad, int iSize,
	   float* pfKnots, float* pfFit, float fReg);
private:
	float mCalcFit(float fX);
	void mCalcTerms(float fX);
	void mCalcCurTerms(float fX);
	int m_iNumKnots;
	int m_iDim;
	float* m_pfTerms;
	float* m_pfCoeff;
	float* m_pfMatrix;
	float* m_pfKnots;
};

class CRemoveSpikes1D
{
public:
	CRemoveSpikes1D(void);
	~CRemoveSpikes1D(void);
	void Clean(void);
	void SetDataSize(int iSize);
	void DoIt(float* pfShiftX, float* pfShiftY, bool bSingle);
private:
	bool mFindBad(float* pfRawShift, float* pfFitShift, float fTol);
	void mRemoveSingle(float* pfRawShift);
	float* m_pfFitX;
	float* m_pfFitY;
	float* m_pfBuf;
	float* m_pfTime;
	float* m_pfKnots;
	bool* m_pbBad;
	int m_iSize;
	int m_iNumKnots;
	CSplineFit1D m_aSplineFit;
};

class CFileName
{
public:
	CFileName(void);
	~CFileName(void);
	void Analyze(char* pcFileName);
	bool IsMrc(void) { return m_bMrc; }
	bool IsTiFF(void) { return m_bTiff; }
	int GetSerial(void) { return m_iSerial; }
	char m_acMainName[256];
private:
	void mReset(void);
	void mGetMainName(void);
	void mGetSerial(void);
	char m_acFileName[256];
	int m_iSerial;
	bool m_bMrc;
	bool m_bTiff;
};

class GPartialCopy
{
public:
  	static void DoIt
	( float* pSrc, 
	  int iSrcSizeX,
	  float* pDst, 
	  int iCpySizeX, 
	  int* piDstSize,
	  cudaStream_t stream = 0
	);
	static void DoIt
	( cufftComplex* pSrc, 
	  int iSrcSizeX, 
	  cufftComplex* pDst,
	  int iCpySizeX,
	  int* piDstSize,
	  cudaStream_t stream = 0
	);
};

class GPhaseShift2D
{
public:
	GPhaseShift2D(void);
	~GPhaseShift2D(void);
	void DoIt
	( cufftComplex* gInCmp,
	  int* piCmpSize,
	  float* pfShift,
	  bool bSum,
	  cufftComplex* gOutCmp,
	  cudaStream_t stream = 0
	);
	void DoIt
	( cufftComplex* gCmpFrm,
	  int* piCmpSize,
	  float* pfShift,
	  cudaStream_t stream = 0
	);
};

class GCorrLinearInterp
{
public:
	GCorrLinearInterp(void);
	~GCorrLinearInterp(void);
	void DoIt
	( cufftComplex* gCmpFrm, int* piCmpSize,
	  cudaStream_t stream = 0
	);
};

class CMultiGpuBase
{
public:
	CMultiGpuBase(void);
	virtual ~CMultiGpuBase(void);
protected:
	void mCleanAll(void);
	void mSetGpus(int* piGpuIDs, int iNumGpus);
	void mCreateStreams(int iStreamsPerGpu);
	void mDeleteStreams(void);
	//------------------------
	void mCreateForwardFFTs(int* piSize, bool bPad = true);
	void mDeleteForwardFFTs(void);
	//----------------------------
	void mCreateInverseFFTs(int* piSize, bool bCmp = true);
	void mDeleteInverseFFTs(void);
	//----------------------------
	cudaStream_t* m_pStreams;
	CCufft2D* m_pForwardFFTs;
	CCufft2D* m_pInverseFFTs;
	int* m_piGpuIDs;
	int m_iNumGpus;
	int m_iStreamsPerGpu;
};

}}
