#pragma once
#include <wchar.h>

namespace Mrc {

enum EMrcMode
{
	eMrcUChar = 0,
	eMrcShort = 1,
	eMrcFloat = 2,
	eMrcUCharEM = 5,
	eMrcUShort = 6,
	eMrcInt = 7,
	eMrc4Bits = 101
};	//EMrcMode


class CMrcModes
{
public:
	CMrcModes(void) {}
	~CMrcModes(void) {}
	static int GetBits(int iMode)
	{	if(iMode == eMrcUChar) return 8;
		if(iMode == eMrcUCharEM) return 8;
		if(iMode == eMrcShort) return 16;
		if(iMode == eMrcUShort) return 16;
		if(iMode == eMrcFloat) return 32;
		if(iMode == eMrcInt) return 32;
		if(iMode == eMrc4Bits) return 4;
		return 0;
	}
};	//CMrcModes


class C4BitImage
{
public:
	C4BitImage(void);
	~C4BitImage(void);
	static int GetPkdSize(int iSize);
	static void GetPkdSize(int* piRawSize, int* piPkdSize);
	static size_t GetImgBytes(int iMode, int* piRawSize);
	static size_t GetLineBytes(int iMode, int iRawSize);
	static void* GetPkdBuf(int* piRawSize);
	static void* Pack(void* pvRawImg, int* piRawSize);
	static void Pack(void* pvRawImg, int* piRawSize, void* pvPkdImg);
	static void Unpack(void* pvPkdImg, void* pvRawImg, int* piRawSize);
};	//C4BitImage


class CMainHeader
{
public:
	CMainHeader(void);
	~CMainHeader(void);
	void SwapByte(void);
	int                     nx;                     // 1 - 4
	int                     ny;                     // 5 - 8
	int                     nz;                     // 9 - 12
	int                     mode;                   // 13 - 16
	int                     nxstart;                // 17 - 20
	int                     nystart;                // 21 - 24
	int                     nzstart;                // 25 - 28
	int                     mx;                     // 29 - 32
        int                     my;                     // 33 - 36
        int                     mz;                     // 37 - 40
        float                   xlen;                   // 41 - 44
        float                   ylen;                   // 45 - 48
        float                   zlen;                   // 49 - 52
        float                   alpha;                  // 53 - 56
        float                   beta;                   // 57 - 60
        float                   gamma;                  // 61 - 64
        int                     mapc;                   // 65 - 68
        int                     mapr;                   // 69 - 72
        int                     maps;                   // 73 - 76
        float                   amin;                   // 77 - 80
        float                   amax;                   // 81 - 84
        float                   amean;                  // 85 - 88
        int                     ispg;                   // 89 - 92
        int                     nsymbt;                 // 93 - 96
        short int               dvid;                   // 97 - 98
        short int               nblank;                 // 99 - 100
        int                     itst;                   // 101 - 104
        char                    blank[24];              // 105 - 128
        short int               numintegers;            // 129 - 130
        short int               numfloats;              // 131 - 132
        short int               sub;                    // 133 - 134
	short int               zfac;                   // 135 - 136
        float                   min2;                   // 137 - 140
        float                   max2;                   // 141 - 144
        float                   min3;                   // 145 - 148
        float                   max3;                   // 149 - 152
        float                   min4;                   // 153 - 156
        float                   max4;                   // 157 - 160
        short int               type;                   // 161 - 162
        short int               lensnum;                // 163 - 164
        short int               nd1;                    // 165 - 166
        short int               nd2;                    // 167 - 168
        short int               vd1;                    // 169 - 170
        short int               vd2;                    // 171 - 172
        float                   min5;                   // 173 - 176
        float                   max5;                   // 177 - 180
        short int               numtimes;               // 181 - 182
        short int               imgseq;                 // 183 - 184
        float                   xtilt;                  // 185 - 188
        float                   ytilt;                  // 189 - 192
        float                   ztilt;                  // 193 - 196
        short int               numwaves;               // 197 - 198
        short int               wave1;                  // 199 - 200
        short int               wave2;                  // 201 - 202
        short int               wave3;                  // 203 - 204
        short int               wave4;                  // 205 - 206
        short int               wave5;                  // 207 - 208
        float                   z0;                     // 209 - 212
        float                   x0;                     // 213 - 216
        float                   y0;                     // 217 - 220
        int                     nlabl;                  // 221 - 224
        char                    data[10][80];           // 225 - 1024
};	//CMainHeader

	
class CLoadMainHeader
{
public:
	CLoadMainHeader(void);	
	~CLoadMainHeader(void);
	void DoIt(int iFile);
	int GetSizeX(void);
	int GetSizeY(void);
	int GetSizeZ(void);
	void GetSize(int* piSize, int iElems);
	int GetStackZ(void);
	int GetMode(void);
	float GetLengthX(void);
	float GetLengthY(void);
	float GetMean(void);
	float GetMin(void);
	float GetMax(void);
	int GetNumInts(void);
	int GetNumFloats(void);
	int GetSymbt(void);
	char* GetNthLabel(int iNthLabel);	/* don't free */
	float GetPixelSize(void);
	int GetGainBytes(void);
	CMainHeader m_aHeader;
	bool m_bSwapByte;
};	// CLoadMainHeader


class CLoadExtHeader
{
public:
	CLoadExtHeader(void);
	~CLoadExtHeader(void);
	void SetFile(int iFile);
	void DoIt(int iNthHeader);
	void LoadGain(float* pfGain);
	void GetTilt(float* pfTilt, int iSize);
	void GetStage(float* pfStage, int iSize);
	void GetShift(float* pfShift, int iSize);
	float GetDefocus(void);
	float GetMean(void);
	float GetExposure(void);
	float GetTiltAxis(void);
	float GetPixelSize(void);
	float GetMag(void);
	float GetNthFloat(int iNthField);
	char* m_pcHeader;
	int m_iHeaderSize;
	int m_iNumInts;
	int m_iNumFloats;
	int m_iSymbt;
	int m_aiImgSize[3];
	bool m_bHasGain;
private:
	int m_iFile;
	int m_iNthHeader;
	bool m_bSwapByte;
};	// CLoadExtHeader


class CLoadImage
{
public:
	CLoadImage(void);
	~CLoadImage(void);
	void SetFile(int iFile);
	void* DoIt(int iNthImage);
	void DoIt(int iNthImage, void* pvImage);
	void DoMany(int iStartImg, int iNumImgs, void** ppvImages);
	void DoPart(int iNthImage, int* piOffset, 
		int* piPartSize, void* pvImage);
	void* GetBuffer(void);
	int m_aiImgSize[3];
	int m_iStackZ;
	int m_iMode;
private:
	void mSeek(int iNthImage, int iBytes);
	void mSwapBytes(void* pvImage, int iPixels);
	int m_iFile;
	int m_iSymbt;
	int m_iImgBytes;
	bool m_bSwapByte;
};	// CLoadImage


class CLoadMrc  
{
public:
	CLoadMrc();
	~CLoadMrc();
	bool OpenFile(char* pcFileName);
	bool OpenFile(wchar_t* pwcFileName);
	bool OpenFile(char* pcFileName, int iSerialNum);
	bool OpenFile(wchar_t* pwcFileName, int iSerialNum);
	void CloseFile(void);
	float GetPixelSize(void);
	CLoadMainHeader* m_pLoadMain;
	CLoadExtHeader* m_pLoadExt;
	CLoadImage* m_pLoadImg;
private:
	float m_fPixelSize;
	int m_iFile;
};	// CLoadMrc


class CSaveMainHeader
{
public:
	CSaveMainHeader(void);
	~CSaveMainHeader(void);
	void SetFile(int iFile);
	void Reset(void);
	void SetImgSize
	(  int* piImgSize,	// image size x and y
	   int iNumImgs,	// number of images per stack
	   int iNumImgStacks,   // number of stacks
	   float fPixelSize
	);
	void SetMode(int iMode);
	void SetMinMaxMean
	(  float fMin, 
	   float fMax, 
	   float fMean
	);
	void SetNumInts(int iNumInts);
	void SetNumFloats(int iNumFloats);
	void SetNthLabel(char* pcLabel, int iNthLabel);
	void SetNthLabel(wchar_t* pwcLabel, int iNthLabel);
	void SetSymbt(int iSymbt);
	void DoIt(void);
	void DoIt(CMainHeader* pHeader);
	int GetMode(void);
	void GetImgSize(int* piSize, int iElems);
	int GetNumInts(void);
	int GetNumFloats(void);
	int GetSymbt(void);
	int GetGainBytes(void);
	CMainHeader m_aHeader;
private:
	void mSetDefault(void);
	void mCheckByteOrder(void);
	int m_iFile;
	double m_dMeanSum;
	int m_iNumMeans;
};	// CSaveMainHeader


class CSaveExtHeader
{
public:
	CSaveExtHeader(void);
	~CSaveExtHeader(void);
	void SetFile(int iFile);
	void Setup(int iNumInts, int iNumFloats, int iNumHeaders);
	void Reset(void);
	void SetTilt(int iHeader, float* pfTilt, int iElems);
	void SetStage(int iHeader, float* pfStage, int iElems);
	void SetShift(int iHeader, float* pfShift, int iElems);
	void SetDefocus(int iHeader, float fDefocus);
	void SetExp(int iHeader, float fExp);
	void SetMean(int iHeader, float fMean);
	void SetTiltAxis(int iHeader, float fTiltAxis);
	void SetPixelSize(int iHeader, float fPixelSize);
	void SetMag(int iHeader, float fMag);
	void SetNthFloat(int iHeader, int iNthField, float fVal);
	void SetHeader(int iHeader, char* pcHeader, int iSize);
	void DoIt(void);
	void SaveGain(int iOffset, float* pfGain, int iBytes);
private:
	void mSetFloatField(int iHeader, int iField, float fVal);
	char* m_pcHeaders;
	int m_iNumInts;
	int m_iNumFloats;
	int m_iNumHeaders;
	int m_iFile;
	int m_iHeaderBytes;
};	// CSaveExtHeader


class CSaveImage
{
public:
	CSaveImage(void);
	~CSaveImage(void);
	void SetFile(int iFile);
	void SetMode(int iMode);
	void SetImgSize(int* piSize);
	void SetSymbt(int iSymbt);
	void DoIt(int iNthImage, void* pvImage);
	void DoIt(int iStartImg, int iNumImgs, void* pvImages);
private:
	int m_iMode;
	int m_aiImgSize[2];
	int m_iSymbt;
	int m_iFile;
};	// CSaveImage


class CSaveMrc  
{
public:
	CSaveMrc();
	~CSaveMrc();
	bool OpenFile(char* pcFileName, int iSerialNum);
	bool OpenFile(wchar_t* pcFileName, int iSerialNum);
	bool OpenFile(char* pcFileName);
	bool OpenFile(wchar_t* pcFileName);
	void Flush(void);
	void CloseFile(void);
	void SetMode(int iMode);
	void SetImgSize
	(  int* piImgSize, 
	   int iNumImgs,	// number of images per stack
	   int iNumImgStacks,	// number of stacks
	   float fPixelSize
	);
	void SetExtHeader
	(  int iNumInts, 
	   int iNumFloat, 
	   int iGainBytes
	);
	void DoIt(int iNthImg, void* pvImage);
	void DoIt
	( int iStartImg, int iNumImgs, 
	  float fMin, float fMax, 
	  float fMean, void* pvImgs
	);
	void SaveMinMaxMean(float fMin, float fMax, float fMean);
	void DoGain(float* pfGain);
	CSaveMainHeader* m_pSaveMain;
	CSaveExtHeader* m_pSaveExt;
	CSaveImage* m_pSaveImg;
private:
	void mOpenFile(char* pcFileName);
	float mCalcMean(void* pvImage);
	float mCalcMin(void* pvImage);
	float mCalcMax(void* pvImage);
	int m_iFile;
	int m_iNumInts;
	int m_iNumFloats;
	int m_iGainBytes;
};	// CSaveMrc


class CFindMrc  
{
public:
	CFindMrc();
	virtual ~CFindMrc();
	void SetFileName(char* pcFileName);
	int FindFromStage(float fStageX, float fStageY);
	int FindFromShift(float fShiftX, float fShiftY);
	float m_fDist;
private:
	int mOpenFile(void);
	char* m_pcFileName;
	int m_iNumSections;
};	// CFindMrc


class CReviseExtHeader
{
public:
	CReviseExtHeader();
	virtual ~CReviseExtHeader();
	bool OpenFile(char* pcFileName);
	bool OpenFile(wchar_t* pwcFileName);
	void CloseFile(void);
	void Load(int iNthHeader);
	void SetStage(float* pfStage, int iElems);
	void SetShift(float* pfShift, int iElems);
	void SetFloat(float fValue, int iNField);
	void Save(void);
private:
	CLoadExtHeader m_aLoadExt;
	CSaveExtHeader m_aSaveExt;
	int m_iFile;
	int m_iHeader;
};	// CReviseMrc


class CMrcScale
{
public:
	CMrcScale(void);
	~CMrcScale(void);
	void Setup(int iNewMode, float fScale);
	void* DoIt(float* pfData, int iPixels);
	void DoIt(float* pfData, int iPixels, void* pvData);
private:
	void mToUChar(float* pfData, void* pvData);
	void mToShort(float* pfData, void* pvData);
	void mToUShort(float* pfData, void* pvData);
	void mToInt(float* pfData, void* pvData);
	void mToFloat(float* pfData, void* pvData);
	int m_iPixels;
	int m_iNewMode;
	float m_fScale;
};	// CMrcScale


class CVerticalFlip
{
public:
	CVerticalFlip(void);
	~CVerticalFlip(void);
	void* DoIt(void* pvImage, int* piSize, int iMode);
	void DoIt(void* pvImage, int* piSize, int iMode, void* pvBuf);
private:
	void mDoUChar(void* pvImage, void* pvBuf);
	void mDoShort(void* pvImage, void* pvBuf);
	void mDoUShort(void* pvImage, void* pvBuf);
	void mDoFloat(void* pvImage, void* pvBuf);
	void mDoInt(void* pvImage, void* pvBuf);
	void mDo4Bits(void* pvImage, void* pvBuf);
	int m_aiSize[2];
	int m_iPixels;
	int m_iMode;
};

}
