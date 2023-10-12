#pragma once
#include "../CMainInc.h"
#include "../Util/CUtilInc.h"
#include "../MrcUtil/CMrcUtilInc.h"
#include <cuda.h>

namespace MotionCor2
{
namespace BadPixel
{

class GCorrectBad
{
public:
        GCorrectBad(void);
        ~GCorrectBad(void);
        void SetWinSize(int iSize);
        void GDoIt
	( float* gfFrame,
	  unsigned char* gucBadMap,
	  int* piFrmSize,
	  bool bPadded,
          cudaStream_t stream=0
	);
private:
        int m_iWinSize;
};	//GCorrectBad

class CCorrectMain : public Util::CMultiGpuBase 
{
public:
	CCorrectMain(void);
	~CCorrectMain(void);
	void DoIt(int iDefectSize);
private:
	void mCorrectFrames(int iNthGpu);
	void mCorrectFrame(int iFrame);
	int m_aiPadSize[2];
	int m_iDefectSize;
	int m_iCurGpu;	
};

}}
