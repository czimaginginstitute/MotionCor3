#pragma once

//=============================================================================
//
// This class is used to shift an image such that a given point (iCentX, 
// iCentY) becomes the new center. This point is relative to the current 
// center. The positive directions point to right and top, recpectively.
//
//=============================================================================
class Util_ShiftImage
{
public:

	Util_ShiftImage(void);

	~Util_ShiftImage(void);

	//---------------------------------------------------------------
	// Set the input image size.
	//---------------------------------------------------------------
	void SetSize(int iImageX, int iImageY);

	//---------------------------------------------------------------
	// Set the input image.
	//---------------------------------------------------------------
	void SetImage(void* pvImage, int iMode);

	//---------------------------------------------------------------
	// Shift the image such that the point (iCentX, iCentY) becomes
	// the new center. [out]
	//---------------------------------------------------------------
	void* DoIt(int iCentX, int iCentY);

	int m_iImageX;

	int m_iImageY;

private:

	void mAllocBuffer(void);

	int* mCalcSrcStart(void);

	int* mCalcTgtStart(void);

	int* mCalcSize(void);

	void mShiftShort(void);

	void mShiftUShort(void);

	void mShiftFloat(void);

	int m_iMode;

	int m_iCenter[2];

	void* m_pvImage;

	void* m_pvBuffer;
};
