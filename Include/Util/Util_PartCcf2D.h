#pragma once

//=============================================================================
//
// This class is used to calculate linear correlation coefficient between a
// pair of two dimensional images.
//
//=============================================================================
class Util_PartCcf2D
{
public:

	Util_PartCcf2D(void);

	~Util_PartCcf2D(void);

	void SetSize(int iImageX, int iImageY);

	void SetShift(float fShiftX, float fShiftY);

	float DoIt(void* pvImage1, void* pvImage2, int iMode);

private:

	void mDelete(void* pvData, int iMode);

	int m_iSize[2];

	int m_iMode;

	int m_iCent[2];
};
