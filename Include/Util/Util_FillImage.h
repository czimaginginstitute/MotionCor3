#pragma once

//=============================================================================
//
// Fill the pixels of zero count with the robust mean that is calculated by
// excluding the pixels of zero count.
//
//=============================================================================
class Util_FillImage
{
public:

	Util_FillImage(void);

	Util_FillImage(int iSizeX, int iSizeY);

	~Util_FillImage(void);

	void SetSize(int iSizeX, int iSizeY);

	void DoIt(void* pvImage, int iMode);

private:

	void mFillUChar(void* pvImage);

	void mFillShort(void* pvImage);

	void mFillFloat(void* pvImage);

	void mFillUShort(void* pvImage);

	int m_iSizeX;

	int m_iSizeY;

	float m_fMean;
};
