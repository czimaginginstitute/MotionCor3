#pragma once

class Util_Vector2D
{
public:

	Util_Vector2D(void);

	Util_Vector2D(float x, float y);

	~Util_Vector2D(void);

	void Set(float x, float y);

	void Set(Util_Vector2D* pVector2D);

	float GetLength(void);

	void Magnify(float fMag);

	float DotProduct(Util_Vector2D* pVector2D);

	//---------------------------------------------------------------
	// Rotate this vector counter-clockwise fRotation degree.
	//---------------------------------------------------------------
	void Rotate(float fRotation);

	void Add(float fValX, float fValY);

	void Subtract(float fValX, float fValY);

	float x;

	float y;
};
