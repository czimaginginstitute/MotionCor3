#pragma once

//=============================================================================
//
// Rotate an image counter-clockwise by the given angle. Note that it is the
// image to be rotated, not the coordinate system.
//
//=============================================================================
class Util_RotateImage
{
public:

	Util_RotateImage(void);

	~Util_RotateImage(void);

	void SetSize(int iImageX, int iImageY);

	void SetMode(int iMode);

	void SetMag(float fMag);

	void SetAngle(float fAngle);

	void* DoIt(void* pvImage);

private:

	void* mDoShort(void* pvImage);

	void* mDoFloat(void* pvImage);

	void* mDoUShort(void* pvImage);

	void* mGetBuffer(void);

	float mCalcX(int x, int y);

	float mCalcY(int x, int y);

	int m_iSize[2];

	short m_sMode;

	void* m_pvImage;

	int m_iHalf[2];

	float m_fCos;

	float m_fSin;

	float m_fMag;
};
