#pragma once
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../DataUtil/CDataUtilInc.h"
#include <CuUtil/DeviceArray2D.h>
#include <Util/Util_Thread.h>
#include <cuda.h>
#include <cufft.h>

namespace MotionCor2
{
namespace Align
{
namespace DU = DataUtil;
class CStackShift;
class CPatchShifts;
class CZbinStack;

class CAlignParam
{
public:
	static CAlignParam* GetInstance(void);
	static void DeleteInstance(void);
	~CAlignParam(void);
	bool PhaseOnly(void);
	bool SimpleSum(void);
	bool CorrectMag(void);
	void GetMagStretch(float* pfStretch);
	bool PatchAlign(void);
	bool SplitSum(void);
	int GetNumPatches(void);
	int GetFrameRef(int iNumFrames);
	//------------------------------
	int m_iTomoAcqIndex;
	int m_iTomoFrmsPerStack;
private:
	CAlignParam(void);
	static CAlignParam* m_pInstance;
};

class CGenAlignedStack
{
public:
	static CGenAlignedStack* GetInstance(void);
	static void DeleteInstance(void);
	~CGenAlignedStack(void);
	void DoIt(CStackShift* pStackShift);
private:
	CGenAlignedStack(void);
	static CGenAlignedStack* m_pInstance;
};

class CCreateAlnSum
{
public:
	CCreateAlnSum(void);
	~CCreateAlnSum(void);
	float* DoIt
	( cufftComplex* gCmpSum,
	  int* piCmpSize,
	  float fBin
	);
	int m_aiImgSize[2];
private:
	void mFourierBin(float fBin);
	cufftComplex* m_gCmpSum;
	int m_aiCmpSize[2];
};

class CAlignedSum 
{
public:
	CAlignedSum(void);
	~CAlignedSum(void);
	static void DoIt(EBuffer eBuffer, CStackShift* pStackShift,
	   int* piSumRange);
private:
	void mDoIt(int iNthGpu);
	void mWait(void);
	void mDoFrame(int iFrame);
	//------------------------
	CStackBuffer* m_pFrmBuffer;
	CStackBuffer* m_pTmpBuffer;
	CStackBuffer* m_pSumBuffer;
	CStackShift* m_pStackShift;
	int m_iNthGpu;
	cudaStream_t m_aStreams[2];
	cufftComplex* m_gCmpSums[2];
	int m_iAbsFrm;
	int m_iStream;
};
	
class CTransformStack : public Util_Thread 
{
public:
	CTransformStack(void);
	~CTransformStack(void);
	static void DoIt
	( EBuffer eBuf,
	  bool bForward,
	  bool bNorm = true
	);
	void ThreadMain(void);
	void Run(int iNthGpu);
private:
	void mTransformGpuFrames(void);
	void mTransformCpuFrames(void);
	void mTransformFrame(cufftComplex* gCmpFrm);
	//------------------------------------------
	CStackBuffer* m_pFrmBuffer;
	CStackBuffer* m_pTmpBuffer;
	int m_iNthGpu;
	cudaStream_t m_aStreams[2];
	Util::CCufft2D* m_pCufft2D;
};

class CGenXcfStack 
{
public:
	CGenXcfStack(void);
	~CGenXcfStack(void);
	static void DoIt(CStackShift* pStackShift);
private:
	void mDoIt(int iNthGpu);
	void mWaitStreams(void);
	void mDoXcfFrame(int iXcfFrm);
	//----------------------------
	CStackBuffer* m_pXcfBuffer;
	CStackBuffer* m_pFrmBuffer;
	CStackBuffer* m_pTmpBuffer;
	CStackBuffer* m_pSumBuffer;
	int m_iNthGpu;
	int m_iStream;
	cudaStream_t m_aStreams[2];
};

class CInterpolateShift
{
public:
	CInterpolateShift(void);
	~CInterpolateShift(void);
	CStackShift* DoIt
	( CStackShift* pGroupShift,
	  DU::CFmGroupParam* pFmGroupParam,
	  DU::CFmIntegrateParam* pFmIntParam,
	  bool bNearest
	);
	void DoIt
	( CStackShift* pGroupShift, 
	  CStackShift* pOutShift,
	  DU::CFmGroupParam* pFmGroupParam,
	  DU::CFmIntegrateParam* pFmIntParam, 
	  bool bNearest
	);
private:
	void mInterpolate
	( float* pfGpCents,
	  float* pfFmCents,
	  float* pfGpShifts,
	  float* pfFmShifts
	);
	int mFindGroup(float* pfGpCents, float fCent);
	int m_iNumFrames;
	int m_iNumGroups;
};

//-------------------------------------------------------------------
// CStackShift: store the measured shifts of each frame.
//   1. For patch based alignment, the location and size of the
//      patch is also buffered.
//-------------------------------------------------------------------
class CStackShift
{
public:
	CStackShift(void);
	~CStackShift(void);
	void Setup(int iNumFrames);
	void SetCenter	 // region where measurement is performed
	( int* piStart, // 2 elements
	  int* piSize	 // 2 elements, (0, 0) is full size
	);
	void SetCenter(float fCentX, float fCentY);
	void Clear(void);
	void Reset(void);
	void SetShift(int iFrame, float* pfShift);
	void SetShift(CStackShift* pSrcShift);
	void AddShift(int iFrame, float* pfShift);
	void AddShift(CStackShift* pIncShift);
	//------------------------------------
	void MakeRelative(int iRefFrame);
	void Multiply(float fFactX, float fFactY);
	void TruncateDecimal(void);
	//-------------------------
	void GetShift(int iFrame, float* pfShift, float fFact=1.0f);
	void GetRelativeShift(int iFrame, float* pfShift, int iRefFrame);
	float* GetShifts(void);  // do not free
	CStackShift* GetCopy(void);
	void GetCenter(float* pfLoc);
	int GetCentralFrame(void);
	void RemoveSpikes(bool bSingle);
	void DisplayShift(const char* pcHeader, int iRow=-1);
	void Smooth(float fWeight);
	//---------------------------------------------------
	int m_iNumFrames;
	float m_afCenter[3];
	bool m_bConverged;
private:
	float* m_pfShiftXs;
	float* m_pfShiftYs;
};	//CStackShift

//---------------------------------------------------------
class CPatchShifts
{
public:
	CPatchShifts(void);
	~CPatchShifts(void);
	void Setup
	( int iNumPatches, // number of patches
	  int* piFullSize  // full stack size 3 elements
	);
	void Setup
	( int iPatchesX,   // number of patches in x axis
	  int iPatchesY,   // number of patches in y axis 
	  int* piFullSize  // full stack size 3 elements
	);
	void SetFullShift
	( CStackShift* pFullShift
	);
	void SetRawShift  // buffer pStackShift
	( CStackShift* pStackShift,
	  int iPatch
	);
	void GetLocalShift
	( int iFrame,
	  int iPatch,
	  float* pfShift
	);
	void GetPatchCenter
	( int iPatch,
	  float* piCenter 
	);
	void CalcShiftSigma
	( int iFrame,
	  float* pfSigmaXY
	);
	void LogFullShifts
	( char* pcLogFile
	);
	void LogPatchShifts // show side by side raw and fit shifts
	( char* pcLogFile	    // of each patch
	);
	void LogFrameShifts // show side by side raw and fit shifts
	( char* pcLogFile      // of each frame
	);
	void CopyCentersToGpu
	( float* gfPatCenters
	);
	void CopyShiftsToGpu(float* gfPatShifts);
	void CopyFlagsToGpu(bool* gbBadShifts);
	void MakeRelative(void);
	void DetectBads(void);
	//----------------------
	CStackShift* m_pFullShift; // shifts of full frames
	int m_iNumPatches;
	int m_aiFullSize[3];
private:
	void mClean(void);
	void mDetectBadOnFrame(int iFrame);
	void mCalcMeanStd(int iFrame, float* pfMeanStd);
	float mCalcLocalRms(int iFrame, int iPatch);
	float* m_pfPatCenters;
	float* m_pfPatShifts;
	bool* m_pbBadShifts;
};	//CPatchShifts

//---------------------------------------------------------
class GCorrelateSum2D
{
public:
	GCorrelateSum2D(void);
	~GCorrelateSum2D(void);
	void SetFilter(float fBFactor, bool bPhaseOnly);
	void SetSize(int* piCmpSize, int* piSeaSize);
	void SetSubtract(bool bSubtract);
	void DoIt
	( cufftComplex* gCmpSum, 
	  cufftComplex* gCmpXcf,
	  float* pfPinnedXcf,
	  Util::CCufft2D* pInverseFFT,
	  cudaStream_t stream = 0
	);
private:
	float m_fBFactor;
	bool m_bPhaseOnly;
	int m_aiCmpSize[2];
	int m_aiSeaSize[2];
	bool m_bSubtract;
};	

class CPeak2D
{
public:
	CPeak2D(void);
	~CPeak2D(void);
	void DoIt(float* pfImg, int* piImgSize);
	void Mask(float* pfImg, int* piImgSize, int* piPos, int iR);
	float m_fPeak;
	float m_afShift[2];
private:
	void mSearchIntPeak(void);
	void mSearchFloatPeak(void);
	bool mIsCentralPeak(void);
	int m_aiPeak[2];
	float m_afPeak[2];
	int m_aiImgSize[2];
	float* m_pfImg;
};	//CPeak2D

class GCC2D
{
public:
	GCC2D(void);
	~GCC2D(void);
	void SetSize(int* piCmpSize);
	void SetBFactor(float fBFactor);
	float DoIt(cufftComplex* gCmp1, cufftComplex* gCmp2,
	   cudaStream_t stream);
private:
	void mTest(cufftComplex* gCmp1, cufftComplex* gCmp2);
	dim3 m_aGridDim;
	dim3 m_aBlockDim;
	int m_aiCmpSize[2];
	float m_fBFactor;
	float* m_gfCC;
	float* m_pfCC;
};

class CEarlyMotion
{
public:
	CEarlyMotion(void);
	~CEarlyMotion(void);
	void Setup(EBuffer eBuffer, float fBFactor);
	void DoIt(DU::CDataPackage* pPackage, CStackShift* pStackShift);	
	
private:
	void mDoIt(void);
	float mIterate(CStackShift* pStackShift, int iAxis);
	void mGetNodeShifts(CStackShift* pStackShift, 
	   int iAxis, float* pfShift);
	void mCalcCoeff(float fGain, float* pfShift, float* pfCoeff);
	void mCalcShift(float* pfCoeffXs, float* pfCoeffYs,
	   CStackShift* pfStackShift);
	void mCorrelate(int iStep, CStackShift* pStackShift);
	void mFindPeaks(float* pfPeaks);
	void mFindPeak(int iPeak, float* pfPeak);
	float m_fBFactor;
	cufftComplex* m_gCmpRef;
	EBuffer m_eBuffer;
	int m_aiCmpSize[2];
	GCorrelateSum2D m_aGCorrelateSum;
	CStackShift* m_pStackShift;
	Util::CCufft2D* m_pInverseFFT;
	int m_aiCent[3];
	int m_aiSeaSize[2];
	int m_aiSumRange[2];
	int m_iNumSteps;
	float m_fStepSize;
};

//---------------------------------------------------------
class CAlignStack
{
public:
	CAlignStack(void);
	~CAlignStack(void);
	void Set1(int iNthGpu);
	void Set2(EBuffer eBuffer, DU::CFmGroupParam* pFmGroupParam);
	void Set3(float fBFactor, bool bPhaseOnly);
	void DoIt
	( CStackShift* pStackShift,
	  CStackShift* pGroupShift
	);
	void WaitStreams(void);
	float m_fErr;
private:
	void mDoFrames(void);
	void mDoGroups(void);
	bool mCheckGpuGroup(int iGroup);
	void mDoGroup(int iGroup, int iStream);
	void mPhaseShift(int iStream, bool bSum);
	void mCorrelate(int iFrame, int iStream);
	void mFindPeaks(void);
	void mUpdateError(float* pfShift);
	void mDestroyStreams(void);
	//-------------------------
	CStackBuffer* m_pFrmBuffer;
	CStackBuffer* m_pTmpBuffer;
	CStackShift* m_pStackShift;
	CStackShift* m_pGroupShift;
	int m_aiSeaSize[2];
	Util::CCufft2D* m_pInverseFFT;
	DU::CFmGroupParam* m_pFmGroupParam;
	GCorrelateSum2D m_aGCorrelateSum;
	cufftComplex* m_gCmpSum;
	cudaStream_t m_aStreams[2];
	int m_aiGpuFmRange[2];
	bool* m_pbGpuGroups;
	int m_iNthGpu;
	int m_iAbsFrm;
};

class CIterativeAlign
{
public:
	CIterativeAlign(void);
	~CIterativeAlign(void);
	void Setup
	( EBuffer eBuffer,
	  float fBFactor,
	  bool bPhaseOnly
	);
	void DoIt
	( DU::CDataPackage* pPackage,
	  CStackShift* pStackShift
	);
	char* GetErrorLog(void);
private:
	CStackShift* mAlignStack
	( CStackShift* pInitShift, 
	  int iNumGroups
	); 
	EBuffer m_eBuffer;
	CAlignStack* m_pAlignStacks;
	int m_iMaxIterations;
	int m_iIterations;
	float m_fTol;
	float m_fBFactor;
	bool m_bPhaseOnly;
	float m_afXcfBin[2];
	float* m_pfErrors;
	DU::CDataPackage* m_pPackage;
};

//---------------------------------------------------------
class CAlignBase : public Util::CMultiGpuBase
{
public:
	CAlignBase(void);
	virtual ~CAlignBase(void);
	void Clean(void);
	virtual void DoIt(DU::CDataPackage* pPackage);
	virtual void LogShift(char* pcLogFile);
	int m_aiImgSize[2]; // after Fourier cropping
	int m_aiPadSize[2]; // after Fourier cropping
	int m_aiCmpSize[2]; // after Fourier cropping
protected:
	void mCreateAlnSums(void);
	void mCreateAlnStack(void);
	CStackShift* m_pFullShift;
	DU::CDataPackage* m_pPackage;
};	//CAlignBase

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------
class CSimpleSum : public CAlignBase
		 
{
public:
	CSimpleSum(void);
	virtual ~CSimpleSum(void);
	void DoIt(DU::CDataPackage* pPackage);
private:
	void mCalcSum(void);
	void mGenStack(void);
	void mCropFrames(int iNthGpu);
	void mCropFrame
	( cufftComplex* gCmpFrm,
	  cufftComplex* gCmpBuf,
	  cudaStream_t stream,
	  int iNthGpu
	);
	void mUnpad
	( cufftComplex* gCmpPad,
	  float* gfUnpad,
	  cudaStream_t stream = 0
	);
};

class CFullAlign : public CAlignBase
{
public:
	CFullAlign(void);
	virtual ~CFullAlign(void);
	void Align(DU::CDataPackage* pPackage);
	virtual void DoIt(DU::CDataPackage* pPackage);
	void LogShift(char* pcLogFile);
protected:
	void mFourierTransform(bool bForward);
	void mDoAlign(void);
	virtual void mCorrect(void);
};	//CFullAlign;

//--------------------------------------------------------------------
//
//--------------------------------------------------------------------
class CPatchAlign : public CFullAlign
{
public:
	CPatchAlign(void);
	virtual ~CPatchAlign(void);
	void DoIt(DU::CDataPackage* pPackage);
	void LogShift(char* pcLogFile);
private:
	void mCorrectFullShift(void);
	void mCalcPatchShifts(void);
	void mChoose(void);
	void mCalcMeanStd
	( float* pfData, int iSize,
	  double* pdMean, double* pdStd
	);
	void mFindGraphenePeaks
	( cufftComplex* gCmp,
	  int* piCmpSize
	);
	CPatchShifts* m_pPatchShifts;
};

class CPatchCenters
{
public:
	static CPatchCenters* GetInstance(void);
	static void DeleteInstance(void);
	~CPatchCenters(void);
	void Calculate(void);
	void GetCenter(int iPatch, int* piCenter);
	void GetStart(int iPatch, int* piStart);
	int m_iNumPatches;
	int m_aiXcfSize[2];
	int m_aiPatSize[2];
private:
	CPatchCenters(void);
	int mCalcStart(float fCent, int iPatSize, int iXcfSize);
	int* m_piPatStarts;
	static CPatchCenters* m_pInstance;
};

class CExtractPatch 
{
public:
	CExtractPatch(void);
	~CExtractPatch(void);
	//static void GetNominalCenter(int iPatch, float* pfCenter);
	static void DoIt(int iPatch);
	void Run(int iNthGpu);
	void Wait(void);
private:
	void mProcessGpuFrames(void);
	void mProcessCpuFrames(void);
	void mProcessFrame(int iFrame, cudaStream_t stream);
	int m_iNthGpu;
	cudaStream_t m_aStream[2];
};
     

//-------------------------------------------------------------------
// CMeasurePatches: Measure the motion of each patch until all
// patches have been measured.
// 1. Each thread checks first if there is any patch waiting to
//    be measured.
// 2. If found, the thread will extract the patch and perform
//    alignment. 
//-------------------------------------------------------------------
class CMeasurePatches 
{
public:
	CMeasurePatches(void);
	~CMeasurePatches(void);
	void DoIt
	( DU::CDataPackage* pPackage,
	  CPatchShifts* pPatchShifts
	);
private:
	void mCalcPatchShift(int iPatch);
	CPatchShifts* m_pPatchShifts;
	float m_fTol;
	bool m_bPhaseOnly;
	DU::CDataPackage* m_pPackage;
};

class CSaveAlign
{
public:
	static CSaveAlign* GetInstance(void);
	static void DeleteInstance(void);
	~CSaveAlign(void);
	bool Open(char* pcFileName);
	bool Open(char* pcDirName, char* pcStackFile);
	void SaveSetting(int* piStkSize, int* piPatches, int* piThrow);
	void DoGlobal(CStackShift* pStackShift);
	void DoLocal(CPatchShifts* pPatchShifts);
	char* GetStartTag(char* pcTagName);
	char* GetEndTag(char* pcTagName);
	char m_acSetting[64];
	char m_acStackSize[64];
	char m_acPatches[64];
	char m_acThrow[64];
	char m_acGlobalShift[64];
	char m_acLocalShift[64];
	char m_acConverge[64];
	char m_acStackID[64];
	char m_acPatchID[64];
private:
	CSaveAlign(void);
	void mSaveGlobal(CStackShift* pFullShift);
	void mSaveLocal(CPatchShifts* pPatchShifts);
	FILE* m_pFile;
	static CSaveAlign* m_pInstance;
};

class CLoadAlign
{
public:
	static CLoadAlign* GetInstance(void);
	static void DeleteInstance(void);
	~CLoadAlign(void);
	void Clean(void);
	bool Open(char* pcFileName);	
	bool Open(char* pcDirName, char* pcStackFile);
	bool IsLoaded(void);
	CStackShift* GetFullShift(bool bClean);
	CPatchShifts* GetPatchShifts(bool bClean);
	int m_aiStkSize[3];
	int m_aiThrow[2];
	int m_aiPatches[3];
private:
	CLoadAlign(void);
	void mReadFile(void);
	void mReadSetting(void);
	void mReadGlobalShift(void);
	void mReadPatchShift(void);
	FILE* m_pFile;
	bool m_bLoaded;
	CStackShift* m_pFullShift;
	CPatchShifts* m_pPatchShifts;
	static CLoadAlign* m_pInstance;
};

//--------------------------------------------------------------------
// This class should be only used to bin the aligned stack in z axis
// when -OutStack option is followed by 1 zbinning. This stack must
// be 4-byte float per pixel.
//--------------------------------------------------------------------
class CZbinStack
{
public:
	CZbinStack(void);
	~CZbinStack(void);
	void DoIt(DU::CDataPackage* pPackage);
private:
        void mGroup(int iGroup, int iGroupStart, int iGroupSize);
        DU::CMrcStack* m_pMrcStack;
};

class GNormByStd2D
{
public:
	GNormByStd2D(void);
	~GNormByStd2D(void);
	void DoIt(float* gfImg, int* piImgSize, bool bPadded,
	   int* piWinSize, cudaStream_t stream = 0);
};

class CDetectFeatures
{
public:
	static CDetectFeatures* GetInstance(void);
	static void DeleteInstance();
	~CDetectFeatures(void);
	void DoIt(CStackShift* pXcfStackShift, int* piNumPatches);
	void GetCenter(int iPatch, int* piImgSize, float* pfCent);
	void FindNearest(float* pfLoc, int* piImgSize, int* piPatSize, 
	   float* pfNewLoc);
private:
	CDetectFeatures(void);
	void mClean(void);
	void mFindCenters(void);
	void mFindCenter(float* pfCenter);
	bool mCheckFeature(float fCentX, float fCentY);
	void mSetUsed(float fCentX, float fCentY);
	int mCheckRange(int iStart, int iSize, int* piRange);
	int m_aiBinnedSize[2];
	int m_aiNumPatches[2];
	float m_afPatSize[2];
	int m_aiSeaRange[4];
	bool* m_pbFeatures;
	bool* m_pbUsed;
	float* m_pfCenters;
	static CDetectFeatures* m_pInstance;
};

//--------------------------------------------------------------------
// CAlignMain: The entry point to start alignment and correction.
//--------------------------------------------------------------------
class CAlignMain
{
public:
	CAlignMain(void);
	~CAlignMain(void);
	void DoIt(DU::CDataPackage* pPackage);
	DU::CAlnSums* GetAlnSums(bool bClean);
	DU::CMrcStack* GetAlnStack(bool bClean);
private:
	char* mCreateLogFile(void);
	void mClean(void);
	DU::CDataPackage* m_pPackage;
	DU::CMrcStack* m_pAlnStack;
	DU::CAlnSums* m_pAlnSums;
};

}}
