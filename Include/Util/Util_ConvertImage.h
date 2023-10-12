#pragma once

//=============================================================================
//
// This class converts an image from one mrc mode to another.
//
//=============================================================================
class Util_ConvertImage
{
public:

	Util_ConvertImage(void);

	~Util_ConvertImage(void);

	void SetSize(int iSizeX, int iSizeY);

	void SetImage(void* pvImage, int iMode);

	void* DoIt(int iNewMode);

	void DoIt(int iNewMode, void* pvImage);

private:

	void mConvertShort(int iNewMode, void* pvBuf);

	void mConvertUShort(int iNewMode, void* pvBuf);

	void mConvertFloat(int iNewMode, void* pvBuf);

	void* mAllocMemory(int iNewMode);

	int m_iSize[2];

	int m_iMode;

	void* m_pvImage;
};
