#pragma once

//=============================================================================
//
// This class copies the overlapped area from the source image to the target
// image. The overlapped area is determined based upon the image sizes and
// locations. The image location is given by the coordinate of its center.
//
//=============================================================================
class Util_CopyImage
{
public:

	Util_CopyImage(void);

	~Util_CopyImage(void);

	void SetSrcSize(int iImageX, int iImageY);

	void SetSrcImage(void* pvImage, int iMode);

	void SetTgtSize(int iImageX, int iImageY);

	void SetTgtImage(void* pvImage, int iMode);

	//---------------------------------------------------------------
	// piSrcPos: the coordinate of the source image center.
	// piTgtPos: the coordinate of the target image center.
	//---------------------------------------------------------------
	void DoIt(int* piSrcPos, int* piTgtPos);

	void DoIt(float* pfSrcPos, float* pfTgtPos);

private:

	float* mCalcSrcOffset(float* pfSrcPos, float* pfTgtPos);

	void mDoItShort(float* pfSrcOffset);

	void mDoItUShort(float* pfSrcOffset);

	void mDoItFloat(float* pfSrcOffset);

	void mConvertSrcImage(void);

	int m_iSrcSize[2];

	int m_iTgtSize[2];

	int m_fSrcPos[2];

	int m_fTgtPos[2];

	int m_iSrcMode;

	int m_iTgtMode;

	void* m_pvSrcImage;

	void* m_pvTgtImage;
};
