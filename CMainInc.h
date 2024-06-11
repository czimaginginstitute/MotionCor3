#pragma once
#include "Util/CUtilInc.h"
#include "DataUtil/CDataUtilInc.h"
#include "MrcUtil/CMrcUtilInc.h"
#include "TiffUtil/CTiffFileInc.h"
#include <Util/Util_Thread.h>
#include <stdio.h>
#include <cufft.h>

namespace MotionCor2 
{
namespace DU = MotionCor2::DataUtil;

class CInput
{
public:
	static CInput* GetInstance(void);
	static void DeleteInstance(void);
	~CInput(void);
	void ShowTags(void);
	void Parse(int argc, char* argv[]);
	void GetBinnedSize(int* piImgSize, int* piBinnedSize);
	bool IsInMrc(void);
	bool IsInTiff(void);
	bool IsInEer(void);
	float GetFinalPixelSize(); // after binning & mag correction
	float GetFinalPixelSize(float fPixelSize);
	char m_acInMrcFile[256];
	char m_acInTifFile[256];
	char m_acInEerFile[256];
	char m_acInSuffix[256];
	char m_acInSkips[256];
	char m_acGainMrc[256];
	char m_acDarkMrc[256];
	char m_acOutMrcFile[256];
	char m_acArchiveFolder[256];
	char m_acDefectFile[256];
	char m_acInAlnFolder[256];
	char m_acOutAlnFolder[256];
	char m_acDefectMap[256];
	char m_acFullSumMrc[256];
	char m_acFmIntFile[256];
	int m_aiNumPatches[3];
	int m_iIterations;
	float m_fTolerance;
	float m_afBFactor[2];
	int m_iPhaseOnly;
	char m_acTmpFile[256];
	char m_acLogDir[256];
	int* m_piGpuIds;
	int* m_piGpuMems;
	int m_iNumGpus;
	int m_iUseGpus;
	int m_iStackZ;
	float m_fFourierBin;
	int m_iAlign;
	float m_fInitDose;
	float m_fFmDose;
	float m_fPixelSize;
	int m_iKv;
	float m_fCs;
	float m_fAmpCont;
	float m_fExtPhase;
	int m_aiThrow[2];
	float m_afSumRange[2];
	int m_aiGroup[2];
	int m_iFmRef;
	int m_iSerial;
	int m_iRotGain;
	int m_iFlipGain;
	int m_iInvGain;
	float m_afTilt[2];
	int m_aiCropSize[2];
	int m_aiOutStack[2];
	float m_afMag[3];
	int m_iInFmMotion;
	float m_fGpuMemUsage;
	int m_iSplitSum;
	int m_iEerSampling;
	int m_iOutStarFile;
	int m_iTiffOrder;
	int m_iCorrInterp;
	//--------------------
	char m_acInMrcTag[32];
	char m_acInTifTag[32];
	char m_acInEerTag[32];
	char m_acInSuffixTag[32];
	char m_acInSkipsTag[32];
	char m_acGainMrcTag[32];
	char m_acDarkMrcTag[32];
	char m_acOutMrcTag[32];
	char m_acArchiveTag[32];
	char m_acDefectFileTag[32];
	char m_acDefectMapTag[32];
	char m_acFullSumMrcTag[32];
	char m_acInAlnTag[32];
	char m_acOutAlnTag[32];
	char m_acPatchesTag[32];
	char m_acIterTag[32];
	char m_acTolTag[32];
	char m_acBftTag[32];
	char m_acPhaseOnlyTag[32];
	char m_acTmpFileTag[32];
	char m_acLogDirTag[32];
	char m_acGpuIDTag[32];
	char m_acStackZTag[32];
	char m_acFourierBinTag[32];
	char m_acAlignTag[32];
	char m_acInitDoseTag[32];
	char m_acFmDoseTag[32];
	char m_acPixelSizeTag[32];
	char m_acKvTag[32];
	char m_acCsTag[32];
	char m_acAmpContTag[32];
	char m_acExtPhaseTag[32];
	char m_acThrowTag[32];
	char m_acTruncTag[32];
	char m_acSumRangeTag[32];
	char m_acGroupTag[32];
	char m_acFmRefTag[32];
	char m_acSerialTag[32];
	char m_acTiltTag[32];
	char m_acCropTag[32];
	char m_acOutStackTag[32];
	char m_acRotGainTag[32];
	char m_acFlipGainTag[32];
	char m_acInvGainTag[32];
	char m_acMagTag[32];
	char m_acInFmMotionTag[32];
	char m_acGpuMemUsageTag[32];
	char m_acUseGpusTag[32];
	char m_acSplitSumTag[32];
	char m_acFmIntFileTag[32];
	char m_acEerSamplingTag[32];
	char m_acOutStarTag[32];
	char m_acTiffOrderTag[32];
	char m_acCorrInterpTag[32];
private:
        CInput(void);
        void mPrint(void);
        int m_argc;
        char** m_argv;
        static CInput* m_pInstance;
};

class CCheckFreeGpus
{
public:
	static CCheckFreeGpus* GetInstance(void);
	static void DeleteInstance(void);
	//-----------------
	~CCheckFreeGpus(void);
	void SetAllGpus(int* piGpuIds, int iNumGpus);
	int GetFreeGpus(int* piFreeGpus, int iNumFreeGpus);
	void FreeGpus(void);
private:
	CCheckFreeGpus(void);
	void mClean(void);
	int mOpenFile(void);
	int mLockFile(int iFd);
	int mUnlockFile(int iFd);
	void mReadGpus(FILE* pFile);
	void mWriteGpus(FILE* pFile);
	bool mCheckActivePid(int iPid);
	//-----------------
	char m_acGpuFile[256];
	int* m_piGpuIds;
	int* m_piPids;
	int m_iNumGpus;
	int m_iPid;
	//-----------------
	static CCheckFreeGpus* m_pInstance;
};

class CGpuBuffer
{
public:
	CGpuBuffer(void);
	~CGpuBuffer(void);
	void Clean(void);
	void Create
	( size_t tFrmBytes, 
	  int iNumFrames, 
	  int iGpuID
	);
	void AdjustBuffer(int iNumFrames);
	void* GetFrame(int iFrame); // do not free
	//----------------------------------------
	size_t m_tFmBytes;
	int m_iNumFrames;
	int m_iNumGpuFrames;
	int m_iGpuID;
private:
	void mCalcGpuFrames(void);
	void mCreateCpuBuf(int iNumFrms);
	void mPrintAllocTimes(float* pfGBs, float* pfTimess);
	//---------------------------------------------------
	void* m_pvGpuFrames;
	void** m_ppvCpuFrames;
	int m_iMaxGpuFrms;
	int m_iMaxCpuFrms;
};

class CStackBuffer
{
public:
	CStackBuffer(void);
	~CStackBuffer(void);
	void Clean(void);
	void Create
	( int* piCmpSize,
	  int iNumFrames, 
	  int* piGpuIDs, 
	  int iNumGpus
	);
	void Adjust(int iNumFrames);
	//-----------------
	int GetStartFrame(int iNthGpu);// starting frame on GPU
	int GetNumFrames(int iNthGpu); // frame on GPU
	bool IsGpuFrame(int iNthGpu, int iFrame);
	//---------------------------------------
	// iFrame = 0 is the 1st frame on iNthGpu
	//---------------------------------------	
	cufftComplex* GetFrame(int iNthGpu, int iFrame);
	cufftComplex* GetFrame(int iAbsFrame);
	//------------------------------------
	int GetFrameGpu(int iFrame);
	void SetDevice(int iNthGpu);
	//--------------------------
	int* m_piGpuIDs;
	int m_iNumGpus;
	int m_aiCmpSize[2];
	int m_iNumFrames;  // all stack frames
	size_t m_tFmBytes;
private:
	int* mCalcFramesPerGpu(int iNumFrms);
	CGpuBuffer* m_pGpuBuffers;
};

enum EBuffer {tmp, sum, frm, xcf, pat};
enum EStkSize {img, cmp, pad};

class CBufferPool
{
public:
	static CBufferPool* GetInstance(void);
	static void DeleteInstance(void);
	~CBufferPool(void);
	void Clean(void);
	void SetGpus(int* piGpuIDs, int iNumGpus);
	void CreateBuffers(int* piStkSize);
	void AdjustBuffer(int iNumFrames);
	CStackBuffer* GetBuffer(EBuffer eBuf);
	//-------------------------------------------------
	// iFrame is relative the starting frame in nth GPU
	//-------------------------------------------------
	void* GetPinnedBuf(int iNthGpu);
	//------------------------------
	Util::CCufft2D* GetForwardFFT(int iNthGpu);
	Util::CCufft2D* GetInverseFFT(int iNthGpu);
	//-----------------------------------------	
	void SetDevice(int iNthGpu);
	//--------------------------------------------------
	int m_iNumSums;
	int m_iNumGpus;
	int* m_piGpuIDs;
	float m_afXcfBin[2];
	int m_aiStkSize[3];
private:
	CBufferPool(void);
	void mCreateSumBuffer(void);
	void mCreateTmpBuffer(void);
	void mCreateFrmBuffer(void);
	void mCreateXcfBuffer(void);
	void mCreatePatBuffer(void);
	CStackBuffer* m_pTmpBuffer;
	CStackBuffer* m_pSumBuffer;
	CStackBuffer* m_pFrmBuffer;
	CStackBuffer* m_pXcfBuffer;
	CStackBuffer* m_pPatBuffer;
	void* m_pvPinnedBuf;
	Util::CCufft2D* m_pCufft2Ds;
	static CBufferPool* m_pInstance;
};

class CLoadRefs
{
public:
	static CLoadRefs* GetInstance(void);
	static void DeleteInstance(void);
	~CLoadRefs(void);
	void CleanRefs(void);
	bool LoadGain(char* pcGainFile);
	bool LoadDark(char* pcDarkFile);
	void PostProcess(int iRotFact, int iFlip, int iInverse);
	bool AugmentRefs(int* piImgSize);
	float* m_pfGain;
	float* m_pfDark;
	int m_aiRefSize[2];
private:
	int mGetFileType(char* pcRefFile);
	void mClearGain(void);
	void mClearDark(void);
	//-----------------
	void mLoadGainMrc(char* pcMrcFile);
	void mLoadGainTiff(char* pcTiffFile);
	//-----------------
	void mRotate(float* gfRef, int* piRefSize, int iRotFact);
	void mFlip(float* gfRef, int* piRefSize, int iFlip);
	void mInverse(float* gfRef, int* piRefSize, int iInverse);
	//-----------------
	float* mToFloat(void* pvRef, int iMode, int* piSize);
	void mCheckDarkRef(void);
	float* mAugmentRef(float* pfRef, int iFact);
	CLoadRefs(void);
	//-----------------
	int m_aiDarkSize[2];
	bool m_bAugmented;
        //-----------------
	static CLoadRefs* m_pInstance;
};

class CProcessThread : public Util_Thread
{
public:
	CProcessThread(void);
	~CProcessThread(void);
	void DoIt(void* pvNewPackage);
	void AsyncDoIt(void* pvDataPackage);
	void Clear(void);
	void ThreadMain(void);
private:
	void mProcess(void);
	bool mCheckGain(void);
	void mApplyRefs(void);
	bool mDetectBadPixels(void);
	bool mCorrectBadPixels(void);
	void mAlignStack(void);
	void mFourierResize(void);
	void mSaveAlnSums(void);
	void* m_pvPackage;
	std::queue<void*> m_aProcQueue;
};

class CSaveSerialCryoEM : public Util_Thread
{
public:
        static CSaveSerialCryoEM* GetInstance(void);
        static void DeleteInstance(void);
        ~CSaveSerialCryoEM(void);
	void AsyncSave(DU::CDataPackage* pPackage);
	void ThreadMain(void);
private:
	CSaveSerialCryoEM(void);
	void mInit(void);
	void mClean(void);
	void mSaveSums(void);
	void mSaveStack(void);
	void mSaveImage
	( char* pcMrcFile, float* pfImg,
	  int* piImgSize, float fPixSize
	);
	void mSaveCtfStack(void);
	void mSaveCtfFit(void);
	DU::CDataPackage* m_pPackage;
	float* m_gfImg;
	void* m_pvGFindMinMax2D;
	void* m_pvGCalcMoment2D;
        std::queue<DU::CDataPackage*> m_aSaveQueue;
	static CSaveSerialCryoEM* m_pInstance;
};

class CGenStarFile
{
public:
	static CGenStarFile* GetInstance(void);
	static void DeleteInstance(void);
	~CGenStarFile(void);
	void OpenFile(char* pcInFile);
	void CloseFile(void);
	void SetStackSize(int* piStkSize);
	void SetHotPixels
	( unsigned char* pucBadMap, 
	  int* piMapSize, bool bPadded
	);
	void SetGlobalShifts(float* pfShifts, int iSize);
private:
	CGenStarFile(void);
	FILE* m_pFile;
	char m_acInMain[256];
	pthread_mutex_t m_aMutex;
	static CGenStarFile* m_pInstance;
};

class CMain
{
public:
	CMain(void);
	~CMain(void);
	bool DoIt(void);
private:
	bool mLoadRefs(void);
	bool mDoSingleCryoEM(void);
	bool mDoSerialCryoEM(void);
	void mDoSerialCryoEMMrc(void);
	void mDoSerialCryoEMTiff(void);
	void mDoSerialCryoEMEer(void);
	int mAsyncLoad(int iCount);
	void mOpenInAlnFile(void);
	void mOpenOutAlnFile(void);
	bool mCreateBufferPool(int* piStkSize, int iCount);
	bool mWaitProcessThread(void);
	bool mWaitSaveThread(void);
	CProcessThread m_aProcessThread;
	bool m_bHasGainFromFile;
	bool m_bHasDarkFromFile;
	char m_acLogFile[256];
};	//CMain

}
