#pragma once
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include "../DataUtil/CDataUtilInc.h"
#include <Util/Util_Thread.h>
#include <cufft.h>

namespace MotionCor2
{
namespace FindCtf
{
namespace MU = MotionCor2::MrcUtil;
namespace DU = MotionCor2::DataUtil;

class CCtfParam
{
public:
	CCtfParam(void);
	CCtfParam(CCtfParam &ctfParam);
	~CCtfParam(void);
	void Setup
	( float fKv, // kV 
	  float fCs, // mm
	  float fAmpContrast,
	  float fPixSize // Angstrom
	);
	void Setup(CCtfParam* pCtfParam);
	//---------------------------
	void SetDfMin(float fDfMin, bool bAngstrom);
	void SetDfMax(float fDfMax, bool bAngstrom);
	void SetDfs(float fDfMin, float fDfMax, bool bAngstrom);
	void SetAstAngle(float fAstAngle, bool bDegree);
	void SetExtPhase(float fExtPhase, bool bDegree);
	void SetPixSize(float fPixSiize);
	//---------------------------
	float GetWavelength(bool bAngstrom);
	float GetDfMax(bool bAngstrom);
	float GetDfMin(bool bAngstrom);
	float GetAstAng(bool bDegree);
	float GetExtPhase(bool bDegree);
	float GetPixSize(void);
	CCtfParam* GetCopy(void);
	//---------------------------
	float m_fWaveLen; // pixel
	float m_fKv;
	float m_fCs; // pixel
	float m_fAmpCont;
	float m_fAmpPhaseShift; // radian
	float m_fExtPhase;   // radian
	float m_fDfMax;   // pixel
	float m_fDfMin;   // pixel
	float m_fAstAng;  // Radian
	float m_fPixSize; // Angstrom
	float m_fScore;
	float m_fCtfRes;  // angstrom
};

class CCtfTheory
{
public:
	CCtfTheory(void);
	~CCtfTheory(void);
	void Setup
	( float fKv, // keV
	  float fCs, // mm
	  float fAmpContrast,
	  float fPixSize    // A
	);
	void Setup(CCtfParam* pCtfParam);
	//-------------------------------
	float Evaluate
	( float fFreq, // relative frequency in [-0.5, +0.5]
	  float fAzimuth
	);
	int CalcNumExtrema
	( float fFreq, // relative frequency in [-0.5, +0.5]
	  float fAzimuth
	);
	float CalcNthZero
	( int iNthZero,
	  float fAzimuth
	);
	float CalcDefocus
	( float fAzimuth
	);
	float CalcPhaseShift
	( float fFreq, // relative frequency [-0.5, 0.5]
	  float fAzimuth
	);
	float CalcFrequency
	( float fPhaseShift,
	  float fAzimuth
	);
private:
	float mCalcWavelength(float fKv);
	void mEnforce(void);
	CCtfParam* m_pCtfParam;
	float m_fPI;
};

class CCtfResults
{
public:
	static CCtfResults* GetInstance(void);
	static void DeleteInstance(void);
	~CCtfResults(void);
	void Clean(void);
	void Setup(int iNumImgs, int* piSpectSize);
	void SetTilt(int iImage, float fTilt);
	void SetDfMin(int iImage, float fDfMin);
	void SetDfMax(int iImage, float fDfMax);
	void SetAzimuth(int iImage, float fAzimuth);
	void SetExtPhase(int iImage, float fExtPhase);
	void SetScore(int iImage, float fScore);
	void SetSpect(int iImage, float* pfSpect);
	//----------------------------------------
	float GetTilt(int iImage);
	float GetDfMin(int iImage);
	float GetDfMax(int iImage);
	float GetAzimuth(int iImage);
	float GetExtPhase(int iImage);
	float GetScore(int iImage);
	float* GetSpect(int iImage, bool bClean);
	void SaveImod(void);
	void Display(int iNthCtf);
	void DisplayAll(void);
	//-----------------------
	int m_aiSpectSize[2];
	int m_iNumImgs;
private:
	CCtfResults(void);
	void mInit(void);
	float* m_pfDfMins;
	float* m_pfDfMaxs;
	float* m_pfAzimuths;
	float* m_pfExtPhases;
	float* m_pfScores;
	float* m_pfTilts;
	float** m_ppfSpects;
	static CCtfResults* m_pInstance;
};

class GCalcCTF1D
{
public:
	GCalcCTF1D(void);
	~GCalcCTF1D(void);
	void SetParam(CCtfParam* pCtfParam);
	void DoIt
	( float fDefocus,  // in pixel
	  float fExtPhase, // phase in radian from phase plate
	  float* gfCTF1D,
	  int iCmpSize
	);
private:
	float m_fAmpPhase;
};

class GCalcCTF2D
{
public:
	GCalcCTF2D(void);
	~GCalcCTF2D(void);
	void SetParam(CCtfParam* pCtfParam);
	void DoIt
	( float fDfMin, float fDfMax, float fAzimuth, 
	  float fExtPhase, // phase in radian from phase plate
	  float* gfCTF2D, int* piCmpSize
	);
	void DoIt
	( CCtfParam* pCtfParam,
	  float* gfCtf2D,
	  int* piCmpSize
	);
	void EmbedCtf
	( float* gfCtf2D, int* piCmpSize,
	  float fMinFreq, float fMaxFreq, // relative freq
	  float fMean, float fGain, // for scaling
	  float* gfFullSpect, bool bPadded
	); 

private:
	float m_fAmpPhase; // phase from amplitude contrast
};

class GLowpass2D
{
public:
	GLowpass2D(void);
	~GLowpass2D(void);
	void DoBFactor
	( cufftComplex* gInCmp,
	  cufftComplex* gOutCmp,
	  int* piCmpSize,
	  float fBFactor
	);
	cufftComplex* DoBFactor
	( cufftComplex* gCmp,
	  int* piCmpSize,
	  float fBFactor
	);
	void DoCutoff
	( cufftComplex* gInCmp,
	  cufftComplex* gOutCmp,
	  int* piCmpSize,
	  float fCutoff
	);
	cufftComplex* DoCutoff
	( cufftComplex* gCmp,
	  int* piCmpSize,
	  float fCutoff
	);
};	//GLowpass2D

class GCalcSpectrum
{
public:
	GCalcSpectrum(void);
	~GCalcSpectrum(void);
	void DoIt
	( cufftComplex* gCmp,
	  float* gfSpectrum,
	  int* piCmpSize,
	  bool bLog
	);
	void DoPad
	( float* gfPadImg,   // image already padded
	  float* gfSpectrum, // GPU buffer
	  int* piPadSize,
	  bool bLog
	);
	void Logrithm
	( float* gfSpectrum,
	  int* piSize
	);
	void GenFullSpect
	( float* gfHalfSpect, int* piCmpSize,
	  float* gfFullSpect, bool bFullPadded
	);
};

class GSpectralCC2D
{
public:
	GSpectralCC2D(void);
	~GSpectralCC2D(void);
	void SetSize(int* piSpectSize);
	int DoIt(float* gfCTF, float* gfSpect);
private:
	int m_aiSpectSize[2];
	float* m_gfCC;
	float* m_pfCC;
};	//GSpectralCC2D

class GBackground1D
{
public:
	GBackground1D(void);
	~GBackground1D(void);
	void SetBackground(float* gfBackground, int iStart, int iSize);
	void Remove1D(float* gfSpectrum, int iSize);
	void Remove2D(float* gfSpectrum, int* piSize);
	void DoIt(float* pfSpectrum, int iSize);
	int m_iSize;
	int m_iStart;
private:
	int mFindStart(float* pfSpectrum);
	float* m_gfBackground;
};

class GRemoveMean
{
public:
	GRemoveMean(void);
	~GRemoveMean(void);
	void DoIt
	(  float* pfImg,  // 2D image
	   bool bGpu,     // if the image is in GPU memory
	   int* piImgSize // image x and y sizes
	);
	void DoPad
	(  float* pfPadImg, // 2D image with x dim padded
	   bool bGpu,       // if the image is in GPU memory
	   int* piPadSize   // x size is padded size
	);
private:
	float* mToDevice(float* pfImg, int* piSize);
	float mCalcMean(float* gfImg);
	void mRemoveMean(float* gfImg, float fMean);
	int m_iPadX;
	int m_aiImgSize[2];
};

class GRmBackground2D
{
public:
	GRmBackground2D(void);
	~GRmBackground2D(void);
	void DoIt
	( float* gfInSpect, // half spact
	  float* gfOutSpect,
	  int* piCmpSize,
	  int iBoxSize // number of spectral pixels
	);
};

class GRadialAvg
{
public:
	GRadialAvg(void);
	~GRadialAvg(void);
	void DoIt(float* gfSpect, float* gfAverage, int* piCmpSize);
};

class GRoundEdge
{
public:
	GRoundEdge(void);
	~GRoundEdge(void);
	void SetMask
	(  float* pfCent,
	   float* pfSize
	);
	void DoIt
	(  float* gfImg,
	   int* piImgSize
	);

private:
	float m_afMaskCent[2];
	float m_afMaskSize[2];
};

class GCtfCC2D
{
public:
	GCtfCC2D(void);
	~GCtfCC2D(void);
	void Setup
	(  float fFreqLow,  // relative freq [0, 0.5]
	   float fFreqHigh, // relative freq [0, 0.5]
	   float fBFactor
	);
	void SetSize(int* piCmpSize); // half spectrum
	float DoIt(float* gfCTF, float* gfSpectrum);
private:
	float m_fFreqLow;
	float m_fFreqHigh;
	float m_fBFactor;
	int m_aiCmpSize[2];
	int m_iGridDimX;
	int m_iBlockDimX;
	float* m_gfRes;
};

class GCtfCC1D
{
public:
	GCtfCC1D(void);
	~GCtfCC1D(void);
	void SetSize(int iSize);
	void Setup
	(  float fFreqLow,   // relative freq [0, 0.5]
	   float fFreqHigh,  // relative freq [0, 0.5]
	   float fBFactor
	);
	float DoIt(float* gfCTF, float* gfSpectrum);
	float DoCPU
	(  float* gfCTF,
	   float* gfSpectrum,
	   int iSize
	);
private:
	int m_iSize;
	float* m_gfRes;
	float m_fFreqLow;
	float m_fFreqHigh;
	float m_fBFactor;
};

class CRescaleImage
{
public:
	CRescaleImage(void);
	~CRescaleImage(void);
	float* GetImage(void) { return m_gfPadImgN; }
	void DoIt(float* gfImg, int* piImgSize, DU::CDataPackage* pPackage);
	//---------------------------
	int m_aiImgSizeN[2];
	int m_aiPadSizeN[2];
	float m_fPixSizeN;
private:
	float* m_gfPadImgN;
	float m_fBinning;

};	// CRescaleImage

class CGenAvgSpectrum
{
public:
	CGenAvgSpectrum(void);
	~CGenAvgSpectrum(void);
	int GetSpectPixels(void);
	void SetSizes
	( int* piImgSize, bool bPadded, 
	  int iTileSize
	);
	//-------------------------------------------------------------
	// Note: gfImage and piImgSize in SetSizes must be consistent.
	// They as a whole must be either padded or not padded.
	//-------------------------------------------------------------
	void DoIt
	( float* gfImage, 
	  float* gfBuf, 
	  float* gfAvgSpect, // m_aiSpectSize
	  float* gfFullSpect // [iTileSize + 1, iTileSize]
	);
	int m_aiSpectSize[2]; // [iTileSize/2+1, iTileSize]
private:
	void mAverage(void);
	void mCalcTileSpectrum(int iTile);
	void mExtractPadTile(int iTile);
	void mRmBackground(void);
	void mLowpass(void);
	//-----------------
	Util::GCalcMoment2D* m_pGCalcMoment2D;
	float* m_gfImg;
	int m_aiImgSize[2]; // unpadded image size
	int m_aiNumTiles[2];
	int m_aiOffset[2];
	int m_iOverlap;
	float m_fOverlap;
	//-----------------
	float* m_gfFullSpect; // padded
	float* m_gfAvgSpect;
	float* m_gfTileSpect;
	float* m_gfPadTile;
	bool m_bPadded;     // whether or not m_gfImg is padded
};

class CFullSpectrum
{
public:
	CFullSpectrum(void);
	~CFullSpectrum(void);
     	void Create
	( float* gfHalfSpect,
	  float* gfCtfBuf,
	  int* piCmpSize,
	  CCtfParam* pCtfParam,
	  float* pfResRange,
	  float* gfFullSpect
	);
	void ToHost(float* pfFullSpect);
	int m_aiFullSize[2];
private:
	void mGenFullSpectrum(void);
	void mEmbedCTF(void);
	//-----------------
	float* m_gfHalfSpect;
	float* m_gfCtfBuf;
	float* m_gfFullSpect;
	CCtfParam* m_pCtfParam;
	int m_aiSpectSize[2];
	float m_afResRange[2];
	float m_fMean;
	float m_fStd;     
};

class CFindDefocus1D
{
public:
	CFindDefocus1D(void);
	~CFindDefocus1D(void);
	void Clean(void);
	void Setup1(int iSpectSize);
	void Setup2(CCtfParam* pCtfParam);
	void SetResRange(float afRange[2]); // angstrom
	void DoIt
	( float afDfRange[2],    // f0, delta angstrom
	  float afPhaseRange[2], // p0, delta degree
	  float* gfRadiaAvg
	);
	float m_fBestDf;
	float m_fBestPhase;
	float m_fMaxCC;
private:
	void mBrutalForceSearch(float afResult[3]);
	void mCalcCTF(float fDefocus, float fExtPhase);
	float mCorrelate(void);
	CCtfParam* m_pCtfParam;
	GCtfCC1D* m_pGCtfCC1D;
	GCalcCTF1D m_aGCalcCtf1D;
	float m_afResRange[2];
	float m_afDfRange[2];    // f0, delta in angstrom
	float m_afPhaseRange[2]; // p0, delta in degree
	float* m_gfRadialAvg;
	int m_iSpectSize;
	float* m_gfCtf1D;
};

class CFindDefocus2D 
{
public:
	CFindDefocus2D(void);
	~CFindDefocus2D(void);
	void Clean(void);
	void Setup1(CCtfParam* pCtfParam, int* piSpectSize);
	void Setup2(float afResRange[2]); // angstrom
	void Setup3
	( float fDfMean, float fAstRatio,  // angstrom 
	  float fAstAngle, float fExtPhase // degree and degree
	);
	//---------------------------
	void DoIt
	( float* gfSpect, 
	  float fPhaseRange  // degree
	);
	void Refine
	( float* gfSpect, float fDfMeanRange,
	  float fAstRange, float fAngRange,
	  float fPhaseRange
	);
	//---------------------------
	float GetDfMin(void);    // angstrom
	float GetDfMax(void);    // angstrom
	float GetAstRatio(void);
	float GetAngle(void);    // degree
	float GetExtPhase(void); // degree
	float GetScore(void);
	float GetCtfRes(void);   // angstrom
private:
	void mIterate(void);
	float mFindAstig(float* pfAstRange, float* pfAngRange);
	float mRefineAstMag(float fAstRange);
	float mRefineAstAng(float fAngRange);
	float mRefineDfMean(float fDfRange);
	float mRefinePhase(float fPhaseRange);
	//---------------------------
	float mCorrelate(float fAzimu, float fAstig, float fExtPhase);
	void mCalcCtfRes(void);
	//---------------------------
	void mGetRange
	( float fCentVal, float fRange,
	  float* pfMinMax, float* pfRange
	);
	//---------------------------
	float* m_gfSpect;
	float* m_gfCtf2D;
	int m_aiCmpSize[2];
	GCtfCC2D* m_pGCtfCC2D;
	GCalcCTF2D m_aGCalcCtf2D;
	CCtfParam* m_pCtfParam;
	//---------------------------
	float m_fDfMean;
	float m_fAstRatio;
	float m_fAstAngle;
	float m_fExtPhase;
	float m_fCCMax;
	float m_fCtfRes;
	//---------------------------
	float m_afPhaseRange[2];
	float m_afDfRange[2];
	float m_afAstRange[2];
	float m_afAngRange[2];
};

class CFindCtfBase
{
public:
	CFindCtfBase(void);
	virtual ~CFindCtfBase(void);
	void Setup1(int* piSpectSize);
	void Setup2(CCtfParam* pCtfParam);
	void SetDfRange(float fDfRange);
	void SetAstRange(float fAstRange);
	void SetAngRange(float fAngRange); // degree
	void SetPhaseRange(float fPhaseRange);
	void DoIt(float* gfSpect);
	CCtfParam* GetResult(void); // callers clean
	void GetResRange(float* pfResRange); 
protected:
	void mGetDfRange
	( float fInitDf, 
	  float fDfRange, 
	  float* pfDfRange
	);
	void mGetPhaseRange
	( float fInitPhase, 
	  float fPhaseRange,
	  float* pfPhaseRange
	);
	CCtfParam* m_pCtfParam0; // initial values;
	CCtfParam* m_pCtfParamN; // final results;
	float* m_gfSpect; // half spectrum centered at (0, N/2)
	int m_aiSpectSize[2];
	//--------------------
	float m_afResRange[2];
	float m_fDfRange;
	float m_fAstRange;
	float m_fAngRange;
	float m_fPhaseRange; // for searching extra phase in degree
};

class CFindCtf1D : public CFindCtfBase
{
public:
	CFindCtf1D(void);
	virtual ~CFindCtf1D(void);
	void Clean(void);
	void Setup1(int* piSpectSize);
	void Setup2(CCtfParam* pCtfParam);
	void DoIt(float* gfSpect);
	void Refine1D(float fInitDf, float fDfRange);
protected:
	void mFindDefocus(float* pfDfRange, float* pfPhaseRange);
	void mCalcRadialAverage(void);
	CFindDefocus1D* m_pFindDefocus1D;
	float* m_gfRadialAvg;
};

class CFindCtf2D : public CFindCtf1D
{
public:
	CFindCtf2D(void);
	virtual ~CFindCtf2D(void);
	void Clean(void);
	void Setup1(int* piSpectSize);
	void Setup2(CCtfParam* pCtfParam);
	void DoIt(float* gfSpect);
	void Refine
	( float afDfMean[2], 
	  float afAstRatio[2],
	  float afAstAngle[2],
	  float afExtPhase[2]
	);
private:
	void mGetResults(void);
	CFindDefocus2D* m_pFindDefocus2D;
};

class CFindCtfHelp
{
public:
	static float CalcAstRatio(float fDfMin, float fDfMax);
	static float CalcDfMin(float fDfMean, float fAstRatio);
	static float CalcDfMax(float fDfMean, float fAstRatio);
};

class CSaveCtfResults : public Util_Thread
{
public:
	static CSaveCtfResults* GetInstance(void);
	static void DeleteInstance(void);
	~CSaveCtfResults(void);
	void AsyncSave(void);
	void ThreadMain(void);
private:
	CSaveCtfResults(void);
	void mSaveImages(void);
	void mSaveFittings(void);
	char m_acInMrcFile[256];
	char m_acOutFolder[256];
	static CSaveCtfResults* m_pInstance;
};

//--------------------------------------------------------------
// gfImg: motion corrected sum and can be padded.
// gfBuf: must have the same or bigger as gfImg. It is used to
//        to host both the half spectrum, full spectrum.
//--------------------------------------------------------------
class CFindCtfMain
{
public:
	CFindCtfMain(void);
	~CFindCtfMain(void);
	static bool bEstimate(void);
	void DoIt
	( float* gfImg, int* piImgSize, bool bPadded,
	  float* gfBuf, 
	  DU::CDataPackage* pPackage
	);
	static bool m_bEstCtf;
private:
	void mGenSpectrums
	( float* gfImg, int* piImgSize, 
	  bool bPadded, float* gfBuf
	);
	void mFindCTF(void);
	void mEmbedCTF(void);
	void mAddSpectrumToPackage(void);
	//-----------------
	int m_aiSpectSize[2]; // [iTileSize/2+1, iTileSize]
	float* m_gfAvgSpect;
	float* m_gfFullSpect; // [iTileSize+2, iTileSize]
	float* m_gfExtBuf;
	float m_fPixSizeN;
	//-----------------
	CFindCtf2D* m_pFindCtf2D;
	DU::CDataPackage* m_pPackage;
};

}}
