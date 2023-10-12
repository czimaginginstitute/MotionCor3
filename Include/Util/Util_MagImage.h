#pragma once

//=============================================================================
//
// This class magnifies the input image when the input magnification is bigger
// than one or shrinks the image when the mag is less than 1.
//
//============================================================================= 
class Util_MagImage
{
public:

	Util_MagImage(void);

	~Util_MagImage(void);

	void SetSize(int iImageX, int iImageY);

	void SetMode(int iMode);

	void SetMag(float fMag);

	void* DoIt(void* pvImage);	/* out */

private:

	void* mGetBuffer(void);

	void* mDoItShort(void* pvImage);

	void* mDoItFloat(void* pvImage);

	void* mDoItUShort(void* pvImage);

	float mCalcX(int x);

	float mCalcY(int y);

	int m_iSize[2];

	short m_sMode;

	float m_fMag;

	int m_iHalf[2];

};
