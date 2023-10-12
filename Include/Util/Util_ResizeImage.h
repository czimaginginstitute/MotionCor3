#pragma once

class Util_ResizeImage
{
public:

	Util_ResizeImage(void);

	~Util_ResizeImage(void);

	void SetOldSize(int iSizeX, int iSizeY);

	void SetNewSize(int iSizeX, int iSizeY);

	void SetMode(int iMode);

	void* DoIt(void* pvImage);

private:

	void* mDoShort(void* pvImage);

	void* mDoUShort(void* pvImage);

	void* mDoFloat(void* pvImage);

	void mFillZeros(void* pvImage);

	void mFindCommonAreaOld(int* piDim);

	void mFindCommonAreaNew(int* piDim);

	void* mAllocMemory(void);

	short m_sMode;

	short m_sOldSize[2];

	short m_sNewSize[2];

};
