#pragma once

class Util_StretchImage
{
public:

	Util_StretchImage(void);

	~Util_StretchImage(void);

	void SetSize(int iSizeX, int iSizeY);

	void SetAngles(float fAngle, float fRefAngle);

	void SetTiltAxis(float fTiltAxis);

	void* DoIt(void* pvImage, int iMode);

	void Unstretch(float* pfShift);

private:

	void mCalcStretchFactor(void);

	void mCalcMatrix(void);

	void* mStretchUChar(void);

	void* mStretchShort(void);

	void* mStretchFloat(void);

	void* mStretchUShort(void);

	float mCalcX(int x, int y);

	float mCalcY(int x, int y);

	unsigned char mWarpUChar(float fX, float fY);

	short mWarpShort(float fX, float fY);

	float mWarpFloat(float fX, float fY);

	unsigned short mWarpUShort(float fX, float fY);

	int m_iSize[2];

	int m_iCent[2];

	float m_fAngle;

	float m_fRefAngle;

	float m_fTiltAxis;

	float m_fStretch;

	void* m_pvImage;

	int m_iMode;

	double* m_pdMatrix;

	double m_dDet;
};
