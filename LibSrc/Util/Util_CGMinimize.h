#pragma once

//-------------------------------------------------------------------
// 1. This is the base class that must be implemented for the
//    function of which the minimum will be searched.
// 2. Eval is the function where customer function will be 
//    implemented.
// 3. Once Eval is implemented it can be used to calculate the
//    function value at the given point pfPoint.
// 4. Function value can also be evaluated by calling RelativeEval
//    to which caller passes the relative distance fDist. 
// 5. fDist in RelativeEval is the distance relative to the
//    reference point m_pfRef along the direction m_pfDir
//-------------------------------------------------------------------
class Util_CGFunction
{
public:
	Util_CGFunction(void);
	virtual ~Util_CGFunction(void);
	void SetDim(int iDim);
	void SetRef(float* pfRef, float* pfDir);
	virtual float Eval(float* pfPoint);
	float RelativeEval(float fDist);
	void GetPoint(float fDist, float* pfPoint);
	int m_iDim;
protected:
	void mClean(void);
	float* m_pfRef;
	float* m_pfDir;
	float* m_pfPoint;
};


//-------------------------------------------------------------------
// 1. Given a function func, and given distinct initial points ax
//    and bx, this routine Util_Bracket::DoIt searches in the
//    downhill direction (defined by the function as evaluated at
//    the initial points) and returns new points m_ax, m_bx, m_cx
//    that bracket a minimum of the function.
// 2. Also retured are the function values at these three points
//-------------------------------------------------------------------
class Util_Bracket
{
public:
	Util_Bracket(void);
	~Util_Bracket(void);
	void SetFunc(Util_CGFunction* pFunc);
	bool DoIt
	(  float* pfRefPoint,
	   float* pfRefDir,
	   float ax,
	   float bx
	);
	float m_ax;  // relative point a
	float m_bx;  // relative point b
	float m_cx;  // relative point c
	float m_fa;  // function val at a
	float m_fb;  // function val at b
	float m_fc;  // function val at c
	int m_iDim;  // dimensiotn of function
private:
	float m_fGold;
	float m_fGLimit;
	float m_fTiny;
	Util_CGFunction* m_pFunc;
};

//-------------------------------------------------------------------
// 1. Given a function Util_CGFunction, and given a bracketing
//    triplet of abscissas ax, bx, cx (such that bx is between
//    ax and cx, and f(bx) is less than both f(ax) and f(cx),
//    Util_Brent::DoIt isolates the minimum to a fractional
//    precision of about fTol using Brent's method. The abscissa
//    of the minimum is returned as xmin, and the minimum function
//    value is returned as brent, the returned function value.
//-------------------------------------------------------------------
class Util_Brent
{
public:
	Util_Brent(void);
	~Util_Brent(void);
	void SetFunc
	(  Util_CGFunction* pFunc
	);
	float DoIt
	(  float* pfRef,
	   float* pfDir,
	   float ax,
	   float bx,
	   float cx,
	   float fTol,
	   float *xmin
	);
	int m_iDim;
private:
	void mClean(void);
	float* m_pfRef;
	float* m_pfDir;
	Util_CGFunction* m_pFunc;
	int m_iIterations;
	float m_fCGold;
	float m_fEps;
};

//-------------------------------------------------------------------
// 1. Minimization of a function func of n variables (implemented
//    in Util_CGFunction).
// 2. Input consists of an initial starting point p[1..n], an
//    initial matrix xi[1..n][1..n], whose rows contain the initial
//    set of directions (usually the n unit vectors), and fTol, the
//    fractional tolerance in the function value such that failure
//    to decrease by more than this amount on one iteration signals
//    doneness. 
// 3. On output, p is set to the best point found, xi is the then-
//    current direction set.
//-------------------------------------------------------------------
class Util_CGPowell
{
public:
	Util_CGPowell(void);
	~Util_CGPowell(void);
	void SetFunc
	(  Util_CGFunction* pFunc
	);
	float DoIt
	(  float *p,
	   float *xi,
	   float fTol
	);
	int m_iDim;
private:
	float mLinmin(float *p, float *xi);
	float mSqr(double dVal);
	float m_fTiny;
	int m_iIterations;
	Util_CGFunction* m_pFunc;
};

class Util_TestPowell : public Util_CGFunction
{
public:
	Util_TestPowell(void);
	virtual ~Util_TestPowell(void);
	float Eval(float* pfPoint);
	void DoIt(void);
private:

};


