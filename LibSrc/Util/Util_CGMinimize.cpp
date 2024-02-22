#include "Util_CGMinimize.h"
#include <math.h>
#include <memory.h>
#include <stdio.h>

static float sSign(double dVal, double dFlag)
{
	float fVal = (float)fabs(dVal);
	if(dFlag >= 0) return fVal;
	else return -fVal;
}

Util_CGFunction::Util_CGFunction(void)
{
	m_iDim = 0;
	m_pfRef = 0L;
	m_pfDir = 0L;
	m_pfPoint = 0L;
}

Util_CGFunction::~Util_CGFunction(void)
{
	mClean();
}

void Util_CGFunction::SetDim(int iDim)
{
	mClean();
	m_iDim = iDim;
	m_pfRef = new float[m_iDim];
	m_pfDir = new float[m_iDim];
	m_pfPoint = new float[m_iDim];
}

void Util_CGFunction::SetRef(float* pfRef, float* pfDir)
{
	int iBytes = m_iDim * sizeof(float);
	memcpy(m_pfRef, pfRef, iBytes);
	memcpy(m_pfDir, pfDir, iBytes);
}

//---------------------------------------------------------
// 1. This method evaluates the function at the point that 
//    is (fDist) apart from the reference point (m_pfRef) 
//    and along the given direction (m_pfDir).
// 2. fDist: the distance between the reference point 
//    (m_pfRef) and the point where the function is 
//    evaluated.
//---------------------------------------------------------
float Util_CGFunction::RelativeEval(float fDist)
{
	this->GetPoint(fDist, m_pfPoint);
	float fVal = this->Eval(m_pfPoint);
	return fVal;
}

//----------------------------------------------------------
// 1. This function must overriden in the child class to 
//    implement customer function.
// 2. pfPoint: The input point of m_iDim dimension.
// 3. Return:  the value of the function
//---------------------------------------------------------
float Util_CGFunction::Eval(float* pfPoint)
{
	return 0.0f;
}

//---------------------------------------------------------
// 1. Determine the point given its relative location to 
//    the reference point along the pre-specified 
//    direction (m_pfDir).
//---------------------------------------------------------
void Util_CGFunction::GetPoint(float fDist, float* pfPoint)
{
	for(int i=0; i<m_iDim; i++)
	{	pfPoint[i] = m_pfRef[i] + fDist * m_pfDir[i];
	}
}

void Util_CGFunction::mClean(void)
{
	if(m_pfRef != 0L) delete[] m_pfRef;
	if(m_pfDir != 0L) delete[] m_pfDir;
	if(m_pfPoint != 0L) delete[] m_pfPoint;
	m_pfRef = 0L;
	m_pfDir = 0L;
	m_pfPoint = 0L;
}


//-------------------------------------------------------------------
// 1. Given a function class Util_CGFunction, and given distinct
//    initial points ax and bx, this class searches the downhill
//    direction (defined by the function as evaluated at initial
//    points ax and bx and return the new points ax, bx, cx that
//    bracket a minimum of the function.
//-------------------------------------------------------------------
Util_Bracket::Util_Bracket(void)
{
	m_iDim = 0;
	m_pFunc = 0L;
	m_fGold = 1.618034;
	m_fGLimit = 100.0f;
	m_fTiny = (float)1e-20;
}

Util_Bracket::~Util_Bracket(void)
{
}

void Util_Bracket::SetFunc(Util_CGFunction* pFunc)
{
	m_pFunc = pFunc;
	m_iDim = m_pFunc->m_iDim;
}

bool Util_Bracket::DoIt
(  float* pfRefPoint,
   float* pfRefDir,
   float ax,
   float bx
)
{	m_pFunc->SetRef(pfRefPoint, pfRefDir);
	//------------------------------------
	m_ax = ax;
	m_bx = bx;
	m_fa = m_pFunc->RelativeEval(ax);
	m_fb = m_pFunc->RelativeEval(bx);
	if(m_fb > m_fa)
	{	float fDum = m_ax;
		m_ax = m_bx; m_bx = fDum;
		//-----------------------
		fDum = m_fa; 
		m_fa = m_fb; m_fb = fDum;
	}
	//-------------------------------
	m_cx = m_bx  + m_fGold * (m_bx - m_ax);
	m_fc = m_pFunc->RelativeEval(m_cx);
	//---------------------------------
	while(m_fb > m_fc)
	{	float r = (m_bx - m_ax) * (m_fb - m_fc);
		float q = (m_bx - m_cx) * (m_fb - m_fa);
		float fmax = fabs(q - r);
		if(fmax < m_fTiny) fmax = m_fTiny;				
		float u = m_bx - ((m_bx - m_cx) * q - (m_bx - m_ax) * r) 
			/ (2.0f * sSign(fmax, q-r));
		float ulim = m_bx + m_fGLimit * (m_cx - m_bx);
		//--------------------------------------------
		float fu = 0.0f;
		if((m_bx - u) * (u - m_cx) > 0.0f)
		{	fu = m_pFunc->RelativeEval(u);
			if(fu < m_fc)
			{	m_ax = m_bx;  m_bx = u;
				m_fa = m_fb;  m_fb = fu;
				break;
			}
			else if(fu > m_fb)
			{	m_cx = u;  m_fc = fu;
				break;
			}
			u = m_cx + m_fGold * (m_cx - m_bx);
			fu = m_pFunc->RelativeEval(u);
		}
		else if((m_cx - u) * (u - ulim) > 0.0f)
		{	fu = m_pFunc->RelativeEval(u);
			if(fu < m_fc)
			{	float u1 = m_cx + m_fGold * (m_cx - m_bx);
				m_bx = m_cx; m_cx = u; u = u1;
				m_fb = m_fc; m_fc = fu;
				fu = m_pFunc->RelativeEval(u);
			}
		}
		else if((u - ulim) * (ulim - m_cx) >= 0.0f)
		{	u = ulim;
			fu = m_pFunc->RelativeEval(u);
		}
		else
		{	u = m_cx + m_fGold * (m_cx - m_bx);
			fu = m_pFunc->RelativeEval(u);
		}
		m_ax = m_bx;  m_bx = m_cx;  m_cx = u;
		m_fa = m_fb;  m_fb = m_fc;  m_fc = fu;
	}
	return true;
}


Util_Brent::Util_Brent(void)
{
	m_pfRef = 0L;
	m_pfDir = 0L;
	m_iIterations = 100;
	m_fCGold = 0.3819660f;
	m_fEps = (float)1e-10;
}

Util_Brent::~Util_Brent(void)
{
	mClean();
}

void Util_Brent::SetFunc(Util_CGFunction* pFunc)
{
	m_pFunc = pFunc;
	m_iDim = pFunc->m_iDim;
	//---------------------
	mClean();
	m_pfRef = new float[m_iDim];
	m_pfDir = new float[m_iDim];
}

//---------------------------------------------------------
//  1. Given a bracketing triplet of abscissa ax, bx, cx
//     (bx is between ax and cx), and f(bx) is less than
//     both f(ax) and f(bx), this routine isolates the
//     minimum to a fractional precision of about fTol
//     using Brent method.
//  2. The abscissa of the minimum is returned at xmin
//  3. DoIt returns the function value at the minimum.
//--------------------------------------------------------- 
float Util_Brent::DoIt
(  float* pfRef,
   float* pfDir,
   float ax,
   float bx,
   float cx,
   float fTol,
   float *xmin
)
{	m_pFunc->SetRef(pfRef, pfDir);
	//----------------------------
	float d, etemp, fu, fv, fw, fx;
	float p, q, r;
	float tol1, tol2;
	float  x, u, v, w, xm;
	float e = 0.0f;
	//-------------
	float a = (ax < cx) ? ax : cx;
	float b = (ax > cx) ? ax : cx;
	x = w = v = bx;
	fw = fv = fx = m_pFunc->RelativeEval(x);
	//--------------------------------------
	for(int iter=1; iter<=m_iIterations; iter++)
	{	xm = 0.5f * (a + b);
		tol1 = fTol * (float)fabs(x) + m_fEps;
		tol2 = 2.0f * tol1;
		if(fabs(x - xm) <= (tol2 - 0.5f * (b - a)))
		{	*xmin = x;
			return fx;
		}
		//----------------
		if(fabs(e) > tol1)
		{	r = (x - w) * (fx - fv);
			q = (x - v) * (fx - fw);
			p = (x - v) * q - (x - w) * r;
			q = 2.0f * (q - r);
			if(q > 0.0f) p = -p;
			q = (float)fabs(q);
			etemp = e;
			e = d;
			if(fabs(p) >= fabs(0.5*q*etemp)
			   || p <= q*(a-x)
			   || p >= q*(b-x))
			{	e = (x >= xm) ? a-x : b-x;
				d = m_fCGold * e;
			}
			else
			{	d = p / q;
				u = x + d;
				if(u-a < tol2 || b-u < tol2)
				{	d = sSign(tol1, xm-x);
				}
			}	
		}
		else
		{	e = (x >= xm) ? a-x : b-x;
			d = m_fCGold * e;
		}
		u = (fabs(d) >= tol1) ? x+d : x+sSign(tol1, d);
		fu = m_pFunc->RelativeEval(u);
		//----------------------------
		if(fu <= fx)
		{	if(u >= x) a = x;
			else b = x;
			v = w;   w = x;   x = u;
			fv = fw; fw = fx; fx = fu;
		}
		else
		{	if(u < x) a = u;
			else b = u;
			if(fu <= fw || w == x)
			{	v = w;   w = u;
				fv = fw; fw = fu;
			}
			else if(fu <= fv || v == x || v == w)
			{	v = u;
				fv = fu;
			}
		}
	}
	fprintf
	(  stderr, "Util_Brent: %s\n",
	   "exceed maximum iterations."
	);
	*xmin = x;
	return fx;
}

void Util_Brent::mClean(void)
{
	if(m_pfRef != 0L) delete[] m_pfRef;
	if(m_pfDir != 0L) delete[] m_pfDir;
	m_pfRef = 0L;
	m_pfDir = 0L;
}

//-------------------------------------------------------------------
//
//-------------------------------------------------------------------
Util_CGPowell::Util_CGPowell(void)
{
	m_fTiny = (float)1e-25;
	m_iIterations = 200;
}

Util_CGPowell::~Util_CGPowell(void)
{
}

void Util_CGPowell::SetFunc(Util_CGFunction* pFunc)
{
	m_pFunc = pFunc;
	m_iDim = pFunc->m_iDim;
}

float Util_CGPowell::DoIt
(  float *p,
   float *xi,
   float fTol
)
{	float* pt = new float[m_iDim];
	float* ptt = new float[m_iDim];
	float* xit = new float[m_iDim];
	//-----------------------------
	int iBytes = sizeof(float) * m_iDim;
	memcpy(pt, p, iBytes);
	//--------------------
	float fret = m_pFunc->Eval(p);
	//----------------------------
	for(int iter=1;; ++iter)
	{	float fp = fret;
		float fptt = 0.0f;
		int ibig = 0;
		float del = 0.0f;
		//---------------
		for(int i=0; i<m_iDim; i++)
		{	memcpy(xit, xi + i * m_iDim, iBytes);
			fptt = fret;
			fret = mLinmin(p, xit);
			if((fptt - fret) > del)
			{	del = fptt - fret;
				ibig = i;
			}
		}
		//-----------------------
		double dTmp = fTol * (fabs(fp) + fabs(fret)) + m_fTiny;
		if(2.0f * (fp - fret) <= dTmp) break;
		//-----------------------------------
		if(iter == m_iIterations)
		{	fprintf
			(  stderr, "Util_CGPowell: %s\n",
			   "exceeds maximum iterations."
			);
			break;
		}
		//------------
		for(int j=0; j<m_iDim; j++)
		{	ptt[j] = 2.0f * p[j] - pt[j];
			xit[j] = p[j] - pt[j];
			pt[j] = p[j];
		}	
		fptt = m_pFunc->Eval(ptt);
		//------------------------
		if(fptt < fp)
		{	float t = 2.0f * (fp - 2.0f * fret + fptt)
				* mSqr(fp - fret - del)
				- del * mSqr(fp - fptt);
			if(t < 0)
			{	fret = mLinmin(p, xit);
				float* xin = xi + (m_iDim - 1) * m_iDim;
				memcpy(xi + ibig * m_iDim, xin, iBytes);
				memcpy(xin, xit, iBytes);
			}
		}
	}
	if(pt != 0L) delete[] pt;
	if(ptt != 0L) delete[] ptt;
	if(xit != 0L) delete[] xit;
	return fret;	
}


float Util_CGPowell::mLinmin
(  float *p,
   float *xi
)
{       Util_Bracket aBracket;
	aBracket.SetFunc(m_pFunc);
        aBracket.DoIt(p, xi, 0.0f, 1.0f);
        float ax = aBracket.m_ax;
        float xx = aBracket.m_bx;
        float bx = aBracket.m_cx;
        float fa = aBracket.m_fa;
        float fx = aBracket.m_fb;
        float fb = aBracket.m_fc;
	//-----------------------
	Util_Brent aBrent;
        aBrent.SetFunc(m_pFunc);
        float xmin = 1.0f;
	float fTol = (float)2e-4;
        float fret = aBrent.DoIt(p, xi, ax, xx, bx, fTol, &xmin);
	//-------------------------------------------------------
	for(int i=0; i<m_iDim; i++)
        {       xi[i] *= xmin;
                p[i] += xi[i];
        }
	return fret;
}

float Util_CGPowell::mSqr(double dVal)
{
	return (float)(dVal * dVal);
}


Util_TestPowell::Util_TestPowell(void)
{
	this->SetDim(2);
}

Util_TestPowell::~Util_TestPowell(void)
{
}

float Util_TestPowell::Eval(float* pfPoint)
{
	float x1 = pfPoint[1] - pfPoint[0] * pfPoint[0];
	float x2 = 1.0f - pfPoint[0];
	float fVal = 100.0f * x1 * x1 + x2 * x2;
	return fVal;
}

//-------------------------------------------------------------------
// 1. Ans: x(0) = 1.0, x(1) = 1.0, min = 0.0
//-------------------------------------------------------------------
void Util_TestPowell::DoIt(void)
{
	Util_CGPowell aPowell;
	aPowell.SetFunc(this);
	//--------------------
	float *p = new float[m_iDim];
	float *xi = new float[m_iDim * m_iDim];
	p[0] = 0.0f;
	p[1] = 0.0f;
	xi[0] = 1.0f; xi[1] = 0.0f;
	xi[2] = 0.0f; xi[3] = 1.0f;
	float fTol = (float)1e-4;
	float fMin = aPowell.DoIt(p, xi, fTol);
	//-------------------------------------
	printf("Sol: %9.3e  %9.3e  %9.3e\n", p[0], p[1], fMin);
	printf("Ans: %9.3e  %9.3e  %9.3e\n", 1.0f, 1.0f, 0.0f);
}
