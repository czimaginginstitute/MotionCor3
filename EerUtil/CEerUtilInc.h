#pragma once
#include <tiffio.h>
#include <cuda.h>
#include "../DataUtil/CDataUtilInc.h"
#include <Util/Util_Thread.h>

namespace MotionCor2
{
namespace EerUtil
{
class GAddRawFrame
{
public:
	GAddRawFrame(void);
	~GAddRawFrame(void);
	void DoIt
	( unsigned char* gucFrm1,
	  unsigned char* gucFrm2,
	  unsigned char* gucSum,
	  unsigned int uiPixels,
	  cudaStream_t stream = 0
	);
};

class CLoadEerHeader
{
public:
        CLoadEerHeader(void);
        ~CLoadEerHeader(void);
        bool DoIt(TIFF* pTiff, int iEerSampling);
	int m_aiCamSize[2];      // TIFFTAG_IMAGEWIDTH TIFFTAG_IMAGEHEIGHT
	int m_aiFrmSize[2];
	int m_iNumFrames;
	int m_iNumBits;
	int m_iEerSampling;
private:
	bool mCheckError(void);
	unsigned short m_usCompression;
};

class CLoadEerFrames
{
public:
        CLoadEerFrames(void);
        ~CLoadEerFrames(void);
	void Clean(void);
	bool DoIt
	( int iFile, 
	  TIFF* pTiff, 
	  CLoadEerHeader* pLoadHeader
	);
	unsigned char* GetEerFrame(int iFrame);     // do not free
	int GetEerFrameSize(int iFrame);
private:
	void mReadFrame(int iFrame);
	TIFF* m_pTiff;
	unsigned char* m_pucFrames;
	int* m_piFrmStarts;
	int* m_piFrmSizes;
	int m_iNumFrames;
	int m_iBytesRead;
};

class CDecodeEerFrame
{
public:
	CDecodeEerFrame(void);
	~CDecodeEerFrame(void);
	void Setup(int* piCamSize, int iEerUpSampling);
	void Do7Bits
	( unsigned char* pucEerFrame,
	  int iEerFrameSize,
	  unsigned char* pucRawFrame
	);
	void Do8Bits
	( unsigned char* pucEerFrame,
	  int iEerFrameSize,
	  unsigned char* pucRawFrame
	);
	int m_aiFrmSize[2];
private:
	void mDo7BitsCounted(void);
	void mDo7BitsSuperRes(void);
	void mDo8BitsCounted(void);
	void mDo8BitsSuperRes(void);
	void mFindElectron(void);
	//-----------------------
	unsigned int m_uiCamPixels;
	unsigned char* m_pucEerFrame;
	unsigned char* m_pucRawFrame;
	unsigned int m_uiNumPixels;
	unsigned char m_ucS;
	unsigned int m_uiX;
	unsigned int m_uiY;
	int m_iUpSampling;
	int m_iEerFrameSize;
	int m_aiCamSize[2];
	int m_aiSuperResAnd[2];
	int m_aiSuperResShift[3];
};

class CRenderMrcStack 
{
public:
	CRenderMrcStack(void);
	~CRenderMrcStack(void);
	void DoIt
	( CLoadEerHeader* pLoadEerHeader,
	  CLoadEerFrames* pLoadEerFrames,
	  void* pvDataPackage  // DataUtil::CDataPackage
	);
private:
	void mRender(void* pvDataPackage);
	void mRenderInt(void* pvDataPackage);
	void mRenderFrame(int iMrcFrm, void* pvDataPackage);
	void mDecodeFrame(int iEerFrame, unsigned char* pucDecodedFrm);
	CDecodeEerFrame m_aDecodeEerFrame;
	CLoadEerHeader* m_pLoadEerHeader;
	CLoadEerFrames* m_pLoadEerFrames;
};

class CLoadEerMain : public Util_Thread
{
public:
	CLoadEerMain(void);
	~CLoadEerMain(void);
	static bool OpenFile(int aiStkSize[3]);
	static void AsyncLoad(void);
	static void* GetPackage(void);
	void Run(int iFile);
	void ThreadMain(void);
private:
	int m_iFile;
	TIFF* m_pTiff;
	CLoadEerHeader* m_pLoadEerHeader;
	CLoadEerFrames* m_pLoadEerFrames;
	int m_aiStkSize[3]; // superRes X, Y, rendered frames
	bool m_bLoaded;
	DataUtil::CMrcStack* m_pRenderedStack;
};
} //namespace EerUtil
} //namespace MotionCor2

