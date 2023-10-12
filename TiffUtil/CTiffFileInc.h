#pragma once
#include <tiffio.h>
#include "../MrcUtil/CMrcUtilInc.h"

namespace MotionCor2
{
namespace TiffUtil
{
class CLoadTiffHeader
{
public:
        CLoadTiffHeader(void);
        ~CLoadTiffHeader(void);
        bool DoIt(int iFile);
        int GetSizeX(void);
        int GetSizeY(void);
        int GetSizeZ(void);
        void GetSize(int* piSize, int iElems);
        int GetMode(void);
	float GetPixelSize(void);
        int GetTileSizeX(void);
        int GetTileSizeY(void);
        int GetNumTilesX(void);
        int GetNumTilesY(void);
	bool IsStrip(void);  // Read rows per strip
	bool IsTile(void);   // Read tiles
private:
	bool mReadImageSize(void);
	bool mReadMode(void);
	void mReadPixelSize(void);
	bool mReadRowsPerStrip(void);
	bool mReadTileSize(void);
	int m_aiImgSize[3];
	int m_iMode;
	float m_fPixelSize;
	int m_aiTileSize[2];
	int m_aiNumTiles[2];
	bool m_bReadImgSize;
	bool m_bReadMode;
	bool m_bRowsPerStrip;
	bool m_bTileSize;
	TIFF* m_pTiff;
};

class CLoadTiffImage
{
public:
        CLoadTiffImage(void);
        ~CLoadTiffImage(void);
        bool SetFile(int iFile);
        void* DoIt(int iNthImage);
        bool DoIt(int iNthImage, void* pvImage);
	int m_iMode;
	int m_aiSize[3];
private:
	bool mReadByStrip(int iNthImage, void* pvImage);
	bool mReadByTile(int iNthImage, void* pvImage);
        TIFF* m_pTiff;
	CLoadTiffHeader m_aLoadHeader;
        int m_iPixelBytes, m_iImgBytes;
};

class CLoadTiffStack : public Util_Thread
{
public:
	CLoadTiffStack(void);
	~CLoadTiffStack(void);
	static void Clean(void); 
	static bool OpenFile(int aiStkSize[3]); 
	static void AsyncLoad(void);
	static void* GetPackage(void);
	void Run(int iFile);
	void ThreadMain(void);
private:
	void mLoadSingle(void);
	void mLoadIntGpu(void);
	void mLoadIntCpu(void);
	int m_iFile;
	CLoadTiffImage* m_pLoadTiffImage;
	bool m_bLoaded;
	float m_fLoadTime;
};

class CTiffFolder
{
public:
	CTiffFolder(void);
	~CTiffFolder(void);
	bool ReadFileNames(char* pcPrefix, char* pcSuffix);
	void GetNthFile(int iNthFile, char* pcFileName);
	void GetNthSerial(int iNthFile, char* pcSerial);
	int m_iNumFiles;
private:
	bool mGetDirName(char* pcTemplate);
	bool mCountFiles(void);
	void mReadFileNames(void);
	void mClean(void);
	char* m_pcFileNames;
	int m_iBufSize;
	char m_acDirName[256];
	char m_acPrefix[256];
	char m_acSuffix[256];
};
        
} //namespace TiffUtil
} //namespace MotionCor2

