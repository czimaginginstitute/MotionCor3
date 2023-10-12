#pragma once
#include "../Util/CUtilInc.h"
#include "../DataUtil/CDataUtilInc.h"
#include <Util/Util_Thread.h>
#include <Mrcfile/CMrcFileInc.h>
#include <cuda.h>

namespace MotionCor2
{
namespace MrcUtil
{

class CAnalyzeMrc
{
public:
	CAnalyzeMrc(void);
	~CAnalyzeMrc(void);
	bool IsMrc(char* pcMrcFile);
	bool IsTomoMrc(char* pcMrcFile);
	bool IsSerialTomoMrc(char* pcMrcFile);
	void GetMrcSize
	( char* pcMrcFile,
	  int* piMrcSize,
	  int* piNumStacks
	);
};

class CTiltAngles
{
public:
	CTiltAngles(void);
	~CTiltAngles(void);
	void Setup(float fStartTilt, float fTiltStep);
	bool Read(char* pcMrcFile);
	int GetAcqIndex(float fTilt, float* pfShift);
	int m_iNumTilts;
	int m_iHalf1;
	float m_fMaxTilt;
	float m_fMinTilt;
	float m_fTiltStep;
	float m_fStartTilt;
};

class CLoadCryoEMStack : public Util_Thread
{
public:
	CLoadCryoEMStack(void);
	~CLoadCryoEMStack(void);
	static void Clean(void);
	static bool OpenFile(int aiStkSize[3]);
	static void AsyncLoad(void);
	static void* GetPackage(void);
	static void GetStkSize(int* piStkSize);
	void ThreadMain(void);
private:
	void mLoadSingle(void);
	void mLoadInt(void);
	void mLoadSingle4Bits(void);
	void mLoadInt4Bits(void);
	Mrc::CLoadMrc* m_pLoadMrc;
	bool m_bLoaded;
	float mCalcMean(char* pcFrame);
};

class CLoadStack
{
public:
	static CLoadStack* GetInstance(void);
	static void DeleteInstance(void);
	~CLoadStack(void);
	bool OpenFile(char* pcMrcFile);
	bool Load(int iStack);
	DataUtil::CMrcStack* GetStack(bool bClean);
	int m_iNumStacks;
	int m_aiStkSize[3];
     char m_acMrcFile[256];
private:
	CLoadStack(void);
	bool mLoadSingle(void);
	bool mLoadSerial(void);
	float mCalcMean(char* pcFrame);
	void mPrintStackInfo(int* piStkSize, int iMode);
	bool m_bSerialFiles;
	DataUtil::CMrcStack* m_pMrcStack;
	static CLoadStack* m_pInstance;
};	//CLoadStacks

class CLoadRefs
{
public:
	static CLoadRefs* GetInstance(void);
	static void DeleteInstance(void);
	~CLoadRefs(void);
	void CleanRefs(void);
	bool LoadGain(char* pcMrcFile);
	bool LoadDark(char* pcMrcFile);
	void PostProcess(int iRotFact, int iFlip, int iInverse);
	bool AugmentRefs(int* piFmSize);
	float* m_pfGain;
	float* m_pfDark;
	int m_aiRefSize[2];
private:
	void mClearGain(void);
	void mClearDark(void);
	void mRotate(float* gfRef, int* piRefSize, int iRotFact);
	void mFlip(float* gfRef, int* piRefSize, int iFlip);
	void mInverse(float* gfRef, int* piRefSize, int iInverse);
	float* mToFloat(void* pvRef, int iMode, int* piSize);
	void mCheckDarkRef(void);
	float* mAugmentRef(float* pfRef, int iFact);
	CLoadRefs(void);
	//--------------
	int m_aiDarkSize[2];
	static CLoadRefs* m_pInstance;
};

class GAugmentRef
{
public:
	GAugmentRef(void);
	~GAugmentRef(void);
	void DoIt
	( float* gfInRef, int* piInSize,
	  float* gfOutRef, int* piOutSize
	);
};

class GApplyRefsToFrame
{
public:
	GApplyRefsToFrame(void);
	~GApplyRefsToFrame(void);
	void SetRefs
	( float* gfGain, // Already cropped if "-Crop" is enabled
	  float* gfDark  // Already cropped if "-Crop" is enabled
	);
	void SetSizes
	( int* piMrcSize,// They are larger than piFrmSize that is
	  int* piFrmSize,// the size after cropping.
	  bool bFrmPadded
	);
	void Unpack
	( unsigned char* gucPkdFrm,
	  unsigned char* gucRawFrm,
	  int* piFrmSize,
	  cudaStream_t stream=0
	);
	void DoIt
	( void* gvFrame, 
	  int iMrcMode, 
	  float* gfFrame,
	  cudaStream_t stream=0
	);
	void DoRaw
	( unsigned char* gucRawFrm, 
	  float* gfFrame,
	  cudaStream_t stream=0
	);
	void DoPkd
	( unsigned char* gucPkdFrm, 
	  float* gfFrame,
	  cudaStream_t stream=0
	);
	void DoShort
	( short* gsFrm, 
	  float* gfFrame,
	  cudaStream_t stream=0
	);
	void DoUShort
	( unsigned short* gusFrm, 
	  float* gfFrame,
	  cudaStream_t stream=0
	);
	void DoFloat
	( float* gfInFrm, 
	  float* gfOutFrm, 
	  cudaStream_t stream=0
	);
private:
	int m_aiMrcSize[2];
	int m_aiFrmSize[2];
	int m_iPadSizeX;
	int m_iMrcOffset;
	float* m_gfGain;
	float* m_gfDark;
};

class CApplyRefs 
{
public:
	CApplyRefs(void);
	~CApplyRefs(void);
	static void DoIt
	( DataUtil::CMrcStack* pMrcStack,
	  float* pfGain,
	  float* pfDark
	);
	void Run(int iNthGpu);
	void Wait(void);
private:
	void mCopyRefs(void);
	void mCorrectGpuFrames(void);
	void mCorrectCpuFrames(void); 
	void mApplyRefs(cufftComplex* gCmpFrm, int iStream);
	//--------------------------------------------------
	int m_iNthGpu;
	float* m_gfGain;
	float* m_gfDark;
	int m_iAbsFrame;
	cudaStream_t m_aStreams[2];
	void* m_pvMrcFrames[2];
	GApplyRefsToFrame m_aGAppRefsToFrame;
};

class G90Rotate2D
{
public:
	G90Rotate2D(void);
	~G90Rotate2D(void);
   void GetRotSize
     (  int* piSize,
        int iRotFactor,
        int* piRotSize
     );
	void Setup
	(  int* piImgSize,
	   int iRotFactor
	);
	void DoIt
	(  float* pfImg,
	   bool bGpu
	);
	float* GetRotImg(bool bClean);
	void GetRotImg(float* pfRotImg, bool bGpu);
	float* m_gfRotImg;
	int m_aiRotSize[2];
private:
	void* mCopyToDevice(void* pvData, int iBytes);
	int m_aiImgSize[2];
	int m_aiCosSin[2];
	int m_iImgBytes;
};	// G90Rotate2D

class GFlip2D
{
public:
	GFlip2D(void);
	~GFlip2D(void);
	void Vertical(float* pfImg, bool bGpu, int* piImgSize);
	void Horizontal(float* pfImg, bool bGpu, int* piImgSize);
private:
	float* mCopyToDevice(float* pfImg, int* piImgSize);
	void mVertical(float* gfImg, int* piImgSize);
	void mHorizontal(float* gfImg, int* piImgSize);
};	//GFlip2D

class GInverse2D
{
public:
	GInverse2D(void);
	~GInverse2D(void);
	void DoIt(float* pfImg, bool bGpu, int* piImgSize);
private:
	void mInverse(float* gfImg, int* piImgSize);
};

//-------------------------------------------------------------------
// 1. Sum all the frames in the stack. These frames are padded
//    if they are in real-space.
//-------------------------------------------------------------------
class CSumFFTStack 
{
public:
	CSumFFTStack(void);
	~CSumFFTStack(void);
	static void DoIt(int iBuffer, bool bSplitSum);
private:
	void mSumFrames(int iNthGpu);
	void mWait(void);
	void mSumGpuFrames(void);
	void mSumCpuFrames(void);
	//-----------------------
	int m_iNthGpu;
	int m_iStartFrm;
	cudaStream_t m_aStreams[2];
};

class CSaveSingleCryoEM
{
public:
	static CSaveSingleCryoEM* GetInstance(void);
	static void DeleteInstance(void);
	~CSaveSingleCryoEM(void);
	void OpenFile(char* pcMrcFile);
	void DoIt
	( DataUtil::CAlnSums* pAlnSums,
	  DataUtil::CMrcStack* pAlnStack,
	  float fPixelSize
	);
private:
	CSaveSingleCryoEM(void);
	void mSave(char* pcMrcFile, float* pfImg, int* piImgSize);
	void mSave(char* pcMrcFile, DataUtil::CMrcStack* pAlnStack);
	char m_acMrcFile[256];
	char m_acMrcFileStk[256];
	float m_fPixelSize;
	float* m_gfImg;
	void* m_pvGFindMinMax2D;
	void* m_pvGCalcMoment2D;
	static CSaveSingleCryoEM* m_pInstance;
};

class CAsyncSingleSave : public Util_Thread
{
public:
	CAsyncSingleSave(void);
	~CAsyncSingleSave(void);
	void DoIt
	(  char* pcMrcFile,
	   float* pfImg,
	   int* piImgSize,
	   float fPixelSize,
	   bool bClean,
	   bool bAsync
	);
	void ThreadMain(void);
private:
	void mSave(void);
	char m_acMrcFile[256];
	float* m_pfImg;
	int m_aiImgSize[2];
	float m_fPixelSize;
	bool m_bClean;
};

class G4BitImage
{
public:
	G4BitImage(void);
	~G4BitImage(void);
	static int GetPkdSize(int iSize);
	static void GetPkdImgSize(int* piRawSize, int* piPkdSize);
	void Pack
	( unsigned char* gucRawFrm, 
	  unsigned char* gucPkdFrm,
	  int* piRawSize
	);
	void Unpack
	( unsigned char* gucPkdFrm,
	  unsigned char* gucRawFrm,
	  int* piRawSize
	);
};

}}
