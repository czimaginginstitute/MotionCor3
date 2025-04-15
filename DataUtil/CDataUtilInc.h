#pragma once
#include "../Util/CUtilInc.h"
#include <Util/Util_Thread.h>
#include <Mrcfile/CMrcFileInc.h>
#include <queue>
#include <cuda.h>

namespace MotionCor2
{
namespace DataUtil
{

class CMrcStack
{
public:
	CMrcStack(void);
	virtual ~CMrcStack(void);
	void Create
	( int iMode,
	  int* piStkSize  // x, y, z sizes
	);
	void Create(int iMode, int* piFmSize, int iNumFms);
	void DeleteFrame(int iFrame);
	void DeleteFrames(void);
	void* GetFrame(int iFrame);
	int GetPixels(void);
	size_t GetVoxels(void);
	float GetTiltA(void);
	float GetPixelSize(void);
	void GetTomoShift(float* pfShift);
	//--------------------------------
	int m_aiStkSize[3];
	int m_iMode;
	size_t m_tFmBytes;
	float m_fPixSize;
	float m_afExt[13];
	int m_iNumFloats;
	int m_iStack;
	int m_iAcqIndex;
protected:
	void** m_ppvFrames;
	int m_iBufSize;
};

class CAlnSums : public CMrcStack
{
public:
	CAlnSums(void);
	virtual ~CAlnSums(void);
	void Setup(bool bDoseWeight, bool bDoseSelected);
	void GetFileExt(int iIndex, char* pcExt);
private:
	char m_acExts[5][32];
};

class CCtfResult
{
public:
        CCtfResult(void);
	~CCtfResult(void);
        void Clean(void);
        void Setup(int iNumImgs, int* piSpectSize);
        void SetTilt(int iImage, float fTilt);
        void SetDfMin(int iImage, float fDfMin);
        void SetDfMax(int iImage, float fDfMax);
        void SetAzimuth(int iImage, float fAzimuth);
        void SetExtPhase(int iImage, float fExtPhase);
        void SetScore(int iImage, float fScore);
	void SetCtfRes(int iImage, float fCtfRes);
        void SetSpect(int iImage, float* pfSpect);
	//----------------------------------------
        void SaveImod(void);
        void Display(int iNthCtf);
        void DisplayAll(void);
	//--------------------
	int m_iNumImgs;
	CMrcStack* m_pSpectStack;
        float* m_pfDfMins;
        float* m_pfDfMaxs;
        float* m_pfAzimuths;
        float* m_pfExtPhases;
        float* m_pfScores;
	float m_fCtfReses;
        float* m_pfTilts;
private:
	void mInit(void);
	int m_iNumCols;
};

class CEntry
{
public:
	CEntry(void);
        ~CEntry(void);
        void Set(int iGroupSize, int iIntSize, float fFmDose);
        int m_iGroupSize;  // number of raw frame in this group
        int m_iIntSize;    // num of raw frames to be integrated
        float m_fFmDose;   // raw frame dose e/A2
};


class CReadFmIntFile
{
public:
	static CReadFmIntFile* GetInstance(void);
	static void DeleteInstance(void);
	~CReadFmIntFile(void);
	bool HasDose(void);
	bool NeedIntegrate(void);
	int GetGroupSize(int iEntry);
	int GetIntSize(int iEntry);
	float GetDose(int iEntry);
	void DoIt(void);
	int m_iNumEntries;
private:
	CReadFmIntFile(void);
	void mClean(void);
	CEntry** m_ppEntries;
	static CReadFmIntFile* m_pInstance;
};

class CFmIntParam
{
public:
	CFmIntParam(void);
	~CFmIntParam(void);
	void Setup(int iNumRawFms, int iMrcMode);
	//-----------------
	int GetIntFmStart(int iIntFrame);
	int GetIntFmSize(int iIntFrame);
	int GetNumIntFrames(void);
	float GetAccruedDose(int iIntFrame);
	float GetTotalDose(void);
	//-----------------
	static bool bIntegrate(void);
	static bool bDoseWeight(void);
	static bool bDWSelectedSum(void);
	bool bInSumRange(int iIntFrame);
	CFmIntParam* GetCopy(void);
	//-----------------
	int m_iNumIntFms;
	float* m_pfIntFmDose;  // dose of each int frame
	float* m_pfAccFmDose;  // accumulated dose of each int frame
	float* m_pfIntFmCents; // pseduo-time stamp of each int frame
private:
	void mSetup(void);          // no frame integration
	void mCalcIntFms(void);     // yes frame integration
	void mThrowIntFrames(void);
	void mClean(void);
	void mAllocate(void);
	void mCalcIntFmCenters(void);
	void mDisplay(void);
	//-----------------
	int* m_piIntFmStart;
	int* m_piIntFmSize;
	//-----------------
	int m_iNumRawFms;
	int m_iMrcMode;
};

class CFmGroupParam
{
public:
        CFmGroupParam(void);
	~CFmGroupParam(void);
	void Setup(int iBinZ, CFmIntParam* pFmIntParam);
        int GetGroupStart(int iGroup);
        int GetGroupSize(int iGroup);
        float GetGroupCenter(int iGroup);
	CFmGroupParam* GetCopy(void);
        int m_iNumGroups;
        int m_iNumIntFms;
	bool m_bGrouping;
	int m_iBinZ;
private:
        void mGroupByRawSize(CFmIntParam* pFmIntParam);
	void mFindMaxGroupRawFms(CFmIntParam* pFmIntParam);
        CFmGroupParam* mGetCopy(void);
        void mClean(void);
        void mAllocate(void);
	//---------------------------
        int* m_piGroupStart;
        int* m_piGroupSize;
        float* m_pfGroupCenters;
	int m_iMaxGroupRawFms;
};

class CDataPackage
{
public:
	CDataPackage(void);
	~CDataPackage(void);
	void SetInputFileName(char* pcFileName);
	char* GetInputFileName(bool bClean);
	void SetSerial(char* pcSerial);
	char* GetSerial(bool bClean);
	void SetRawStack(CMrcStack* pRawStack);
	CMrcStack* GetRawStack(bool bClean);
	//----------------------------------
	void SetAlnStack(CMrcStack* pAlnStack);
	CMrcStack* GetAlnStack(bool bClean);
	//----------------------------------
	void SetAlnSums(CAlnSums* pAlnSums);
	CMrcStack* GetAlnSums(bool bClean);
	//---------------------------------
	void SetCtfStack(CMrcStack* pCtfStack);
	CMrcStack* GetCtfStack(bool bClean);
	//----------------------------------
	void SetCtfParam(void* pvCtfParam);
	void* GetCtfParam(bool bClean);
	//-----------------------------
	void DeleteInputFile(void);
	void DeleteSerial(void);
	void DeleteRawStack(void);
	void DeleteAlnStack(void);
	void DeleteAlnSums(void);
	void DeleteCtfStack(void);
	void DeleteCtfParam(void);
	void DeleteFmIntParam(void);
	void DeleteFmGroupParams(void);
	void DeleteAll(void);
	//-------------------
	char* m_pcInFileName;
	char* m_pcSerial;
	CMrcStack* m_pRawStack;
	CMrcStack* m_pAlnStack;
	CAlnSums* m_pAlnSums;
	CMrcStack* m_pCtfStack;
	CFmIntParam* m_pFmIntParam;
	CFmGroupParam* m_pFmGroupParams;
	void* m_pvCtfParam;
	//------------------------------
	float m_fTilt;
};

class CStackFolder : public Util_Thread
{
public:
	static CStackFolder* GetInstance(void);
	static void DeleteInstance(void);
	~CStackFolder(void);
	void PushPackage(CDataPackage* pDataPackage);
	CDataPackage* GetPackage(bool bPop);
	void DeleteFront(void);
	int GetQueueSize(void);
	//---------------------
	bool ReadFiles(void);
	void ThreadMain(void);
private:
        CStackFolder(void);
        bool mReadSingle(void);
        bool mGetDirName(void);
        bool mOpenDir(void);
        int mReadFolder(bool bFirstTime);
        bool mAsyncReadFolder(void);
	char* mGetSerial(char* pcInputFile);
	char* mGetInPrefix(void);
	bool mCheckSkips(const char* pcString);
	void mClean(void);
	//-----------------
        char m_acDirName[256];
	char m_acPrefix[256];
	char m_acSuffix[256];
	char m_acSkips[256];
	//-----------------
	std::queue<CDataPackage*> m_aFileQueue;
        int m_ifd;
        int m_iwd;
        double m_dRecentFileTime;
        static CStackFolder* m_pInstance;
};

}}
