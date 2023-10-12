#pragma once

//=============================================================================
//
// This class is used to crop a partial image from an input image at a given 
// location and with a given size.
//
//=============================================================================
class Util_CropImageBase;

class Util_CropImage
{
public:

	Util_CropImage(void);

	~Util_CropImage(void);

	//---------------------------------------------------------------
	// Set the input image size.
	//---------------------------------------------------------------
	void SetSize(int iImageX, int iImageY);

	//---------------------------------------------------------------
	// Set the input image.
	//---------------------------------------------------------------
	void SetImage(void* pvImage, int iMode);

	//---------------------------------------------------------------
	// Set the size of a cropped image. 
	//---------------------------------------------------------------
	void SetCropSize(int iSizeX, int iSizeY);

	//---------------------------------------------------------------
	// Crop an image centered at the specified location. iCentX, 
	// iCentY must be inside the image. They are relative to the 
	// image center. The upper and right are the positive direction.
	// [out]
	//---------------------------------------------------------------
	void* DoIt(int iCentX, int iCentY);

	int m_iSize[2];

private:

	int m_iImageX;

	int m_iImageY;

	Util_CropImageBase* m_pCropImage;
};

//=============================================================================
//
//
//=============================================================================
class Util_CropImageBase
{
public:

	Util_CropImageBase(void);

	virtual ~Util_CropImageBase(void);

	void SetSize(int iImageX, int iImageY);

	void SetCropSize(int iSizeX, int iSizeY);

	virtual void SetImage(void* pvImage);

	virtual void* DoIt(int iCentX, int iCentY);

protected:

	void mSetCenter(int iCentX, int iCentY);

	bool mValidate(void);

	int* mCalcSrcStart(void);

	int* mCalcTgtStart(void);

	int* mCalcSize(void);

	int m_iImageX;

	int m_iImageY;

	int m_iSize[2];

	int m_iCenter[2];
};

//=============================================================================
//
//
//=============================================================================
class Util_CropImageShort : public Util_CropImageBase
{
public:

	Util_CropImageShort(void);

	virtual ~Util_CropImageShort(void);

	void SetImage(void* pvImage);

	void* DoIt(int iCentX, int iCentY);

private:

	void mAllocBuffer(void);

	short* m_psImage;

	short* m_psBuffer;
};

//=============================================================================
//
//
//=============================================================================
class Util_CropImageUShort : public Util_CropImageBase
{
public:

	Util_CropImageUShort(void);

	virtual ~Util_CropImageUShort(void);

	void SetImage(void* pvImage);

	void* DoIt(int iCentX, int iCentY);

private:

	void mAllocBuffer(void);

	unsigned short* m_psImage;

	unsigned short* m_psBuffer;
};

//=============================================================================
//
//
//=============================================================================
class Util_CropImageFloat : public Util_CropImageBase
{
public:

	Util_CropImageFloat(void);

	virtual ~Util_CropImageFloat(void);

	void SetImage(void* pvImage);

	void* DoIt(int iCentX, int iCentY);

private:

	void mAllocBuffer(void);

	float* m_pfImage;

	float* m_pfBuffer;
};
