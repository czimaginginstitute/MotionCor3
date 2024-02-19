#pragma once
#include "../Util/CUtilInc.h"
#include "../DataUtil/CDataUtilInc.h"
#include "../MotionDecon/CMotionDeconInc.h"
#include "../Align/CAlignInc.h"
#include <Util/Util_Thread.h>
#include <cufft.h>

namespace MotionCor2
{
namespace Correct
{
namespace DU = DataUtil;

class GWeightFrame
{
public:
	GWeightFrame(void);
	~GWeightFrame(void);
	void Clean(void);
	//-------------------------------------------------
	// fInitDose: Dose received before 1st frame.
	// fFramDose: Dose received by each frame.
	// piStkSize[0]: Frame width, or size x
	// piStkSize[1]: Frame height, or size y
	// fPixelSize: in angstrom.
	// piStkSize[2]: Number of frames.
	//-------------------------------------------------
	void BuildWeight
	( float fPixelSize,
	  int iKv,
	  float* pfFmDose,
	  int* piStkSize,
	  float* gfWeightSum,
	  cudaStream_t stream = 0
	);
	//-------------------------------------------------
	// Weighting is performed in Fourier space
	//-------------------------------------------------
	void DoIt
	( cufftComplex* gCmpFrame,
	  int iFrame,
	  cudaStream_t stream = 0
	);
private:
	int m_aiCmpSize[2];
	int m_iNumFrames;
	float* m_pfFmDose; // accumulated dose
	float* m_gfWeightSum;
};

//-----------------------------------------------------------------------------
// 1. This class generates both global-motion corrected stack and the sum.
// 2. The corrected stack is saved in pGFFTStack. Therefore, pGFFTStack
//    stores the real and padded global-motion corrected frames. This
//    stack will be further corrected for local motion.
// 3. The returned sum is in Fourier space. The caller will need to
//    bin to the specfied resolution.
//-----------------------------------------------------------------------------
class CGenRealStack : public Util_Thread
{
public:
	CGenRealStack(void);
	~CGenRealStack(void);
	static void DoIt
	( EBuffer eBuffer,
	  bool bCorrectBilinear,
	  bool bMotionDecon,
	  Align::CStackShift* pStackShift = 0L
	);

	void ThreadMain(void);
	void Run(int iNthGpu);
private:
	void mDoGpuFrames(void);
	void mDoCpuFrames(void);
	void mAlignFrame(cufftComplex* gCmpFrm);
        void mCorrectBilinear(cufftComplex* gCmpFrm);
	void mMotionDecon(cufftComplex* gCmpFrm);
	//---------------------------------------
	Util::CCufft2D* m_pCufft2D;
	cudaStream_t m_aStream[2];
	MotionDecon::CInFrameMotion m_aInFrameMotion;
	int m_iNthGpu;
	int m_iAbsFrm;
};

class CCorrectFullShift 
{
public:
	CCorrectFullShift(void);
	virtual ~CCorrectFullShift(void);
	static void DoIt
	( Align::CStackShift* pStackShift,
	  DU::CDataPackage* pPackage
	);
protected:
	static void mSumPartialSums(void);
	static void mCorrectMag(void);
	static void mUnpadSums(void);
	static void mEstimateCtf(float* gfImg, int* piImgSize);
	//----------------------------
	static Align::CStackShift* m_pFullShift;
	static DU::CDataPackage* m_pPackage;
public:
	virtual void Run(int iNthGpu);
	virtual void Wait(void);
protected:
	void mInit(void);
	void mCorrectGpuFrames(void);
	void mCorrectCpuFrames(void);
	void mGenSums(cufftComplex* gCmpFrm);
	virtual void mAlignFrame(cufftComplex* gCmpFrm);
	void mMotionDecon(cufftComplex* gCmpFrm);
	void mDoseWeight(cufftComplex* gCmpFrm);
	void mSum(cufftComplex* gCmpFrm, int iNthSum);
	void mCropFrame(cufftComplex* gCmpFrm);
	void mCheckDoseWeight(void);
	void mCheckFrameCrop(void);
	//---------------------------
	MotionDecon::CInFrameMotion m_aInFrameMotion;
	GWeightFrame* m_pGWeightFrame;
	Util::CCufft2D* m_pForwardFFT;
	Util::CCufft2D* m_pInverseFFT;
	CStackBuffer* m_pFrmBuffer;
	CStackBuffer* m_pSumBuffer;
	CStackBuffer* m_pTmpBuffer;
	cudaStream_t m_aStreams[2];
	int m_iNthGpu;
	int m_iAbsFrm;
	static int m_aiInCmpSize[2];
	static int m_aiInPadSize[2];
	static int m_aiOutCmpSize[2];
	static int m_aiOutPadSize[2];
};

class GCorrectPatchShift : public CCorrectFullShift, public Util_Thread
{
public:
	GCorrectPatchShift(void);
	virtual ~GCorrectPatchShift(void);
	static void DoIt
	( Align::CPatchShifts* pPatchShifts,
	  DU::CDataPackage* pPackage
	);
	virtual void Run(int iNthGpu);
	void ThreadMain(void);
protected:
	void mCorrectCpuFrames(void);
	void mCorrectGpuFrames(void);
	virtual void mAlignFrame(cufftComplex* gCmpFrm);
	void mCalcMeanShift(int iStream);
	virtual void mMotionDecon(cufftComplex* gCmpFrm);
	//-----------------------------------------------
	static Align::CPatchShifts* m_pPatchShifts;
	float* m_gfPatShifts;
	bool* m_gbBadShifts;
	float* m_gfPatCenters;
	dim3 m_aBlockDim;
	dim3 m_aGridDim;
};	

//-------------------------------------------------------------------
// For correcting anisotropic magnification.
//-------------------------------------------------------------------
class GStretch
{
public:
	GStretch(void);
	~GStretch(void);
	void Setup(float fStretch, float fStretchAxis);
	float* GetMatrix(bool bGpu); // [out]
	void DoIt
	( float* gfInImg,
	  bool bPadded,
	  int* piFrmSize,
	  float* gfOutImg
	);
	void Unstretch(float* pfInShift, float* pfOutShift);
	//--------------------------------------------------
	float m_afMatrix[3];
private:
	float* mCopyMatrixToGpu(float* pfMatrix);
	void mCreateTexture(float* pfImg, bool bGpu, int* piImgSize);
	float m_fDet;
};

class CCorrMagThread : public Util_Thread
{
public:
        CCorrMagThread(void);
        ~CCorrMagThread(void);
        void Run
        (  DataUtil::CMrcStack* pMrcStack,
           float fStretch,
           float fStretchAxis,
           Util::CNextItem* pNextItem,
           int iGpuId
        );
        void ThreadMain(void);
private:
        DataUtil::CMrcStack* m_pMrcStack;
        float m_fStretch;
        float m_fStretchAxis;
        Util::CNextItem* m_pNextItem;
        int m_iGpuId;
};

}}
