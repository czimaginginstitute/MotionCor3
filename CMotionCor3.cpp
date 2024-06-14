#include "CMainInc.h"
#include "TiffUtil/CTiffFileInc.h"
#include <Mrcfile/CMrcFileInc.h>
#include <Util/Util_Time.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <string.h>
#include <nvToolsExt.h>

using namespace MotionCor2;

bool mCheckSame(void);
bool mCheckLoad(void);
bool mCheckSave(char* pcMrcFile);
bool mCheckFreeGpus(void);
bool mCheckGPUs(void);
void mCheckPeerAccess(void);

enum ERetCode 
{	eSuccess = 0,
	eNoFreeGPUs = 1,
	eInputOutputSame = 2,
	eFailLoad = 3,
	eFailSave = 4,
	eNoValidGPUs = 5,
	eFailProcess = 6
};
	

int main(int argc, char* argv[])
{
	nvtxRangePushA("MotionCor2");
	CInput* pInput = CInput::GetInstance();
	if(argc == 2)
	{	if(strcasecmp(argv[1], "--version") == 0)
		{	printf("MotionCor3 version 1.1.2\n"
			       "Built on Jun 11 2024\n");
			return eSuccess;
		}
		if(strcasecmp(argv[1], "--help") == 0)
		{	printf("\nUsage: MotionCor3 Tags\n");
			pInput->ShowTags();
			return eSuccess;
		}
	}
	pInput->Parse(argc, argv);
	//-----------------
	bool bHasFreeGpus = mCheckFreeGpus();
	if(!mCheckFreeGpus()) return eNoFreeGPUs;
	//-----------------
	cuInit(0);
	if(mCheckSame()) return eInputOutputSame;
	//-----------------
	bool bLoad = mCheckLoad();
	if(!bLoad) return eFailLoad;
	//-----------------
	bool bSave = mCheckSave(pInput->m_acOutMrcFile);
	if(!bSave) return eFailSave;
	//-----------------
	bool bGpu = mCheckGPUs();
	if(!bGpu) return eNoValidGPUs;
	//-----------------
	Util_Time aTimer;
	aTimer.Measure();
	CMain aMain;
	bool bSuccess = aMain.DoIt();
	//-----------------
	CCheckFreeGpus* pCheckFreeGpus = CCheckFreeGpus::GetInstance();
	pCheckFreeGpus->FreeGpus();
	CCheckFreeGpus::DeleteInstance();
	//-----------------
	nvtxRangePop();
	float fSecs = aTimer.GetElapsedSeconds();
	printf("Total time: %f sec\n", fSecs);
	if(!bSuccess) return eFailProcess;
	else return eSuccess;
}

//--------------------------------------------------------------------
// 1. Do not check the free gpu file in /tmp folder if users do not
//    activate -UseGpus option in command line.
// 2. Do not check when users specifies all the GPUs using -UseGpus
//    in command line.
// 3. Check only when -UseGpus specifies less GPUs than the provided
//    GPUs in the command line using -Gpu
// 4. This is added per request from Github users.
//--------------------------------------------------------------------
bool mCheckFreeGpus(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iUseGpus >= pInput->m_iNumGpus) return true;
	if(pInput->m_iUseGpus <= 0) return true;
	//-----------------
	CCheckFreeGpus* pCheckFreeGpus = CCheckFreeGpus::GetInstance();
	pCheckFreeGpus->SetAllGpus(pInput->m_piGpuIds, pInput->m_iNumGpus);
	int iFreeGpus = pCheckFreeGpus->GetFreeGpus(
	   pInput->m_piGpuIds, pInput->m_iUseGpus);
	//-----------------
        if(iFreeGpus <= 0)
        {       fprintf(stderr, "Error: All GPUs are in use, quit.\n\n");
                return false;
        }
	//-----------------
        pInput->m_iNumGpus = iFreeGpus;
	return true;
}

bool mCheckSame(void)
{	
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iSerial == 1) return false;
	//--------------------------------------
	int iRet = strcasecmp(pInput->m_acInMrcFile, 
		pInput->m_acOutMrcFile);
	if(iRet != 0) return false;
	//-------------------------
	fprintf
	(  stderr, "mCheckSame: %s\n  Input: %s\n  Output: %s\n\n",
	   "input and output files are the same.",
	   pInput->m_acInMrcFile,
	   pInput->m_acOutMrcFile
	);
	return true;
}

bool mCheckLoad(void)
{
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iSerial == 1) return true;
	//-------------------------------------
	if(pInput->IsInTiff()) return true;
	else if(pInput->IsInMrc()) return true;
	else if(pInput->IsInEer()) return true;
	//-------------------------------------
	fprintf(stderr, "Error: no valid input file name.\n"
	   "   mCheckLoad: \n\n");
	return false;
}

bool mCheckSave(char* pcMrcFile)
{
	CInput* pInput = CInput::GetInstance();
        if(pInput->m_iSerial == 1) return true;
        //-------------------------------------
	Mrc::CSaveMrc aSaveMrc;
	bool bSave = aSaveMrc.OpenFile(pcMrcFile);
	if(bSave)
	{	remove(pcMrcFile);
		return true;
	}
	//------------------
	fprintf(stderr, "Error:cannot open output MRC file.\n"
	   "   mCheckSave: %s\n\n", pcMrcFile);
	return false;
}
	
bool mCheckGPUs(void)
{
	CInput* pInput = CInput::GetInstance();
	int* piGpuIds = new int[pInput->m_iNumGpus];
	int* piGpuMems = new int[pInput->m_iNumGpus];
	//-------------------------------------------
	int iCount = 0;
	cudaDeviceProp aDeviceProp;
	//-------------------------
	for(int i=0; i<pInput->m_iNumGpus; i++)
	{	int iGpuId = pInput->m_piGpuIds[i];
		cudaError_t tErr = cudaSetDevice(iGpuId);
		if(tErr != cudaSuccess)
		{	printf
			(  "Info: skip device %d, %s\n",
			   pInput->m_piGpuIds[i],
			   cudaGetErrorString(tErr)
			);
			continue;
		}
		piGpuIds[iCount] = iGpuId;
		cudaGetDeviceProperties(&aDeviceProp, iGpuId);
		piGpuMems[iCount] = (int)(aDeviceProp.totalGlobalMem
			/ (1024 * 1024));
		iCount++;
	}
	//---------------
	for(int i=0; i<iCount; i++)
	{	pInput->m_piGpuIds[i] = piGpuIds[i];
		pInput->m_piGpuMems[i] = piGpuMems[i];
		printf
		(  "GPU %d memory: %d MB\n", 
		   pInput->m_piGpuIds[i],
		   pInput->m_piGpuMems[i]
		);
	}
	printf("\n");
	pInput->m_iNumGpus = iCount;
	if(piGpuIds != 0L) delete[] piGpuIds;
	if(piGpuMems != 0L) delete[] piGpuMems;
	if(iCount > 0) return true;
	//-------------------------
	fprintf(stderr, "mCheckGPUs: no valid device detected.\n\n");
	return false;
}

void mCheckPeerAccess(void)
{
	cudaError_t cuErr;
	CInput* pInput = CInput::GetInstance();
	if(pInput->m_iNumGpus == 1) return;
	//---------------------------------
	for(int i=0; i<pInput->m_iNumGpus; i++)
	{	cudaSetDevice(pInput->m_piGpuIds[i]);
		for(int j=0; j<pInput->m_iNumGpus; j++)
		{	if(j == i) continue;
			cuErr = cudaDeviceEnablePeerAccess
			( pInput->m_piGpuIds[j], 0 );
			if(cuErr == cudaSuccess) continue;
			printf("mCheckGPUs: GPU %d cannot access %d memory\n"
				"   %s.\n\n", pInput->m_piGpuIds[i], 
				pInput->m_piGpuIds[j], 
				cudaGetErrorString(cuErr));
		}
	}
	cudaSetDevice(pInput->m_piGpuIds[0]);
}
