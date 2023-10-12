#include "CMainInc.h"
#include "Util/CUtilInc.h"
#include <stdio.h>
#include <string.h>
#include <memory.h>

using namespace MotionCor2;

CInput* CInput::m_pInstance = 0L;

CInput* CInput::GetInstance(void)
{
	if(m_pInstance != 0L) return m_pInstance;
	m_pInstance = new CInput;
	return m_pInstance;
}

void CInput::DeleteInstance(void)
{
	if(m_pInstance == 0L) return;
	delete m_pInstance;
	m_pInstance = 0L;
}

CInput::CInput(void)
{
	strcpy(m_acInMrcTag, "-InMrc");
	strcpy(m_acInTifTag, "-InTiff");
	strcpy(m_acInEerTag, "-InEer");
	strcpy(m_acInSuffixTag, "-InSuffix");
	strcpy(m_acOutMrcTag, "-OutMrc");
	strcpy(m_acArchiveTag, "-ArcDir");
	strcpy(m_acGainMrcTag, "-Gain");
	strcpy(m_acDarkMrcTag, "-Dark");
	strcpy(m_acDefectFileTag, "-DefectFile");
	strcpy(m_acInAlnTag, "-InAln");
	strcpy(m_acOutAlnTag, "-OutAln");
	strcpy(m_acDefectMapTag, "-DefectMap");
	strcpy(m_acFullSumMrcTag, "-FullSum");
	strcpy(m_acPatchesTag, "-Patch");
	strcpy(m_acIterTag, "-Iter");
	strcpy(m_acTolTag, "-Tol");
	strcpy(m_acBftTag, "-Bft");
	strcpy(m_acPhaseOnlyTag, "-PhaseOnly");
	strcpy(m_acTmpFileTag, "-TmpFile");
	strcpy(m_acLogDirTag, "-LogDir");
	strcpy(m_acGpuIDTag, "-Gpu");
	strcpy(m_acStackZTag, "-StackZ");
	strcpy(m_acFourierBinTag, "-FtBin");
	strcpy(m_acAlignTag, "-Align");
	strcpy(m_acInitDoseTag, "-InitDose");
	strcpy(m_acFmDoseTag, "-FmDose");
	strcpy(m_acPixelSizeTag, "-PixSize");
	strcpy(m_acKvTag, "-kV");
	strcpy(m_acCsTag, "-Cs");
	strcpy(m_acAmpContTag, "-AmpCont");
	strcpy(m_acExtPhaseTag, "-ExtPhase");
	strcpy(m_acThrowTag, "-Throw");
	strcpy(m_acTruncTag, "-Trunc");
	strcpy(m_acSumRangeTag, "-SumRange");
	strcpy(m_acGroupTag, "-Group");
	strcpy(m_acFmRefTag, "-FmRef");
	strcpy(m_acSerialTag, "-Serial");
	strcpy(m_acTiltTag, "-Tilt");
	strcpy(m_acCropTag, "-Crop");
	strcpy(m_acOutStackTag, "-OutStack");
	strcpy(m_acRotGainTag, "-RotGain");
	strcpy(m_acFlipGainTag, "-FlipGain");
	strcpy(m_acInvGainTag, "-InvGain");
	strcpy(m_acMagTag, "-Mag");
	strcpy(m_acInFmMotionTag, "-InFmMotion");
	strcpy(m_acGpuMemUsageTag, "-GpuMemUsage");
	strcpy(m_acUseGpusTag, "-UseGpus");
	strcpy(m_acSplitSumTag, "-SplitSum");
	strcpy(m_acFmIntFileTag, "-FmIntFile");
	strcpy(m_acEerSamplingTag, "-EerSampling");
	strcpy(m_acOutStarTag, "-OutStar");
	strcpy(m_acTiffOrderTag, "-TiffOrder");
	strcpy(m_acCorrInterpTag, "-CorrInterp");
	//---------------------------------------
	m_aiNumPatches[0] = 0;
	m_aiNumPatches[1] = 0;
	m_aiNumPatches[2] = 0;  // overlapping in percentage
	m_iIterations = 15;
	m_fTolerance = 0.1f;
	m_afBFactor[0] = 500.0f;
	m_afBFactor[1] = 100.0f;
	m_iPhaseOnly = 0;
	m_iStackZ = 0;
	m_fFourierBin = 1.0f;
	m_iAlign = 1;
	m_fInitDose = 0.0f;
	m_fFmDose = 0.0f;
	m_fPixelSize = 0.0f;
	m_iKv = 300;
	m_fCs = 2.7;
	m_fAmpCont = 0.07f;
	m_fExtPhase = 0.0f;
	m_iFmRef = -1;
	m_aiThrow[0] = 0;
	m_aiThrow[1] = 0;
	m_afSumRange[0] = 3.0f;
	m_afSumRange[1] = 25.0f;
	m_aiGroup[0] = 1; m_aiGroup[1] = 4;
	m_iSerial = 0;
	m_aiOutStack[0] = 0;
	m_aiOutStack[1] = 1;
	m_iRotGain = 0;
	m_iFlipGain = 0;
	m_iInvGain = 0;
	m_piGpuIds = 0L;
	m_piGpuMems = 0L;
	m_iNumGpus = 0;
	m_iUseGpus = 0;
	m_afTilt[0] = 0.0f;
	m_afTilt[1] = 0.0f;
	m_afMag[0] = 1.0f;
	m_afMag[1] = 1.0f;
	m_afMag[2] = 0.0f;
	m_iInFmMotion = 0;
	m_fGpuMemUsage = 0.75f;
	m_iSplitSum = 0;
	m_iEerSampling = 1;
	m_iOutStarFile = 0;
	m_iTiffOrder = 1;
	m_iCorrInterp = 0;
	memset(m_aiCropSize, 0, sizeof(m_aiCropSize));
}

CInput::~CInput(void)
{
	if(m_piGpuIds != 0L) delete[] m_piGpuIds;
	if(m_piGpuMems != 0L) delete[] m_piGpuMems;
}

void CInput::ShowTags(void)
{
	printf("%-15s\n"
	   "  1. Input MRC file that stores dose fractionated stacks.\n"
	   "  2. It can be a MRC file containing a single stack collected\n"
           "     in Leginon or multiple stacks collected in UcsfTomo.\n"
	   "  3. It can also be the path of a folder containing multiple\n"
	   "     MRC files when %s option is turned on.\n\n",
	   m_acInMrcTag, m_acSerialTag);
	//------------------------------
	printf("%-15s\n"
	   "  1. Input TIFF file that stores a dose fractionated stack.\n",
           m_acInTifTag);
	//---------------
	printf("%-15s\n"
	   " 1. Input EER file that stores a dose fractionated stack.\n",
	   m_acInEerTag);
	//---------------
	printf("%-15s\n"
	   "  1. Output MRC file that stores the frame sum.\n"
	   "  2. It can be either a MRC file name or the prefix of a series\n"
	   "     MRC files when %s option is turned on.\n\n",
	   m_acOutMrcTag, m_acSerialTag);
	//-------------------------------
	printf
	(  "%-15s\n"
	   " 1. Path of the archive folder that holds the archived raw\n"
	   "    stacks with each pixel packed into 4 bits.\n"
	   " 2. The archived stacks are saved in MRC file with the gain\n"
	   "    reference saved in the extended header.\n"
	   " 3. The rotated and/or flipped gain reference will be saved\n"
	   "    if -RotGain and or -FlipGain are enabled.\n\n",
	   m_acArchiveTag
	);
	//---------------
	printf
	(  "%-15s\n"
	   " 1. MRC file for global-motion corrected, unweighted sum.\n"
	   " 2. This file is generated as soon as the global motion\n"
	   "    correction is completed while the program continues\n"
	   "    lengthy local motion correction. This file allows users\n"
	   "    to perform CTF estimate to gain quick feedback on the\n"
	   "    image quality.\n"
	   " 3. This file is temporary, when the next stack is processed,\n"
	   "    its content will be overwritten.\n\n",
	   m_acFullSumMrcTag
	);
	//------------------
	printf
	(  "%-15s\n"
	   "1. Defect file stores entries of defects on camera.\n"
	   "2. Each entry corresponds to a rectangular region in image.\n"
	   "   The pixels in such a region are replaced by neighboring\n"
	   "   good pixel values.\n"
	   "3. Each entry contains 4 integers x, y, w, h representing\n"
	   "   the x, y coordinates, width, and heights, respectively.\n\n",
	   m_acDefectFileTag
	);
	//------------------
	printf
	( "%-15s\n"
	  "  1. Specify the path to the directory where the alignment file\n"
	  "     will be loaded.\n"
	  "  2. The alignment file is a text file that stores the program\n"
	  "     setting and measured global and local motion. This file\n"
	  "     is created with %s option.\n"
	  "  3. Once the alignment file is loaded, the alignment procedure\n"
	  "     will be bypassed with the loaded alignment data applied\n"
	  "     to generate motion-corrected images.\n\n",
	  m_acInAlnTag, m_acOutAlnTag
	);
	printf
	( "%-15s\n"
	  "  1. Specify the path to the directory where the alignment file\n"
	  "     will be saved.\n"
	  "  2. The alignment file is a text file that stores the program\n"
	  "     setting and measured global and local motion. This file can\n"
	  "     be reloaded next time into MotionCor2 that will bypass\n"
	  "     the alignment process.\n\n", m_acOutAlnTag
	);
	//-------------------------------------------------
	printf
	( "%-15s\n"
	  "1. Defect map is a binary (0 or 1) map where defective pixels\n"
	  "   are assigned value of 1 and good pixels have value of 0.\n"
	  "2. The defective pixels are corrected with a random pick of\n"
	  "   good pixels in its neighborhood.\n"
	  "3. This is map must have the same dimension and orientation\n"
	  "   as the input movie frame.\n"
	  "4. This map can be provided as either MRC or TIFF file that has\n"
	  "   MRC mode of 0 or 5 (unsigned 8 bit).\n\n",
	  m_acDefectMapTag
	);
	//-----------------
	printf
	( "%-15s\n"
	  "  1. Serial-processing all MRC files in a given folder whose\n"
	  "     name should be specified following %s.\n"
	  "  2. The output MRC file name emplate should be provided\n"
	  "     folllowing %s\n"
  	  "  3. 1 - serial processing, 0 - single processing, default.\n"
	  "  4. This option is only for single-particle stack files.\n\n",
	  m_acSerialTag, m_acInMrcTag, m_acOutMrcTag
	);
	//------------------------------------------
	printf("%-15s\n", m_acGainMrcTag);
	printf("   MRC file that stores the gain reference. If not\n");
	printf("   specified, MRC extended header will be visited\n");
	printf("   to look for gain reference.\n\n");
	//-------------------------------------------
	printf("%-15s\n", m_acDarkMrcTag);
	printf("  1. MRC file that stores the dark reference. If not\n");
	printf("     specified, dark subtraction will be skipped.\n");
	printf("  2. If -RotGain and/or -FlipGain is specified, the\n");
	printf("     dark reference will also be rotated and/or flipped.\n\n");
	//---------------------------------------------------------------------
	printf("%-15s\n", m_acTmpFileTag);
	printf("   Temporary image file for debugging.\n\n");
	//---------------------------------------------------
	printf("%-15s\n", m_acLogDirTag);
	printf("  1. Log directory storing log files. Log files have the\n"
	  "     same file names as the output MRC files but with mrc\n"
	  "     replaced with log.\n\n"); 
	//---------------------------------
	printf("%-15s\n", m_acPatchesTag);
	printf("  1. It follows by  number of patches in x and y dimensions.\n");
	printf("  2. The default values are 1 1, meaning only full-frame\n");
	printf("     based alignment is performed.\n\n");
	//-----------------------------------------------
	printf("%-15s\n", m_acIterTag);
	printf("   Maximum iterations for iterative alignment,\n");
	printf("   default 5 iterations.\n\n");
	printf("%-15s\n", m_acTolTag);
	printf("   Tolerance for iterative alignment,\n");
	printf("   default 0.5 pixel.\n\n");
	printf("%-15s\n", m_acBftTag);
	printf("   B-Factor for alignment, default 100.\n\n");
	printf("%-15s\n", m_acPhaseOnlyTag);
	printf("   Only phase is used in cross correlation.\n");
	printf("   default is 0, i.e., false.\n\n");
	//------------------------------------------
	printf("%-15s\n", m_acFourierBinTag);
	printf("   Binning performed in Fourier space, default 1.0.\n\n");
	//----------------------------------------------------------------
	printf("%-15s\n", m_acInitDoseTag);
	printf("   Initial dose received before stack is acquired\n\n");
	printf("%-15s\n", m_acFmDoseTag);
	printf("   Frame dose in e/A^2. If not specified, dose\n");
	printf("   weighting will be skipped.\n\n");
	printf("%-15s\n", m_acPixelSizeTag);
	printf("   Pixel size in A of input stack in angstrom. If not\n");
	printf("   specified, dose weighting will be skipped.\n\n");
	printf("%-15s\n", m_acKvTag);
	printf("   High tension in kV needed for dose weighting.\n");
	printf("   Default is 300.\n\n");
	//-----------------------------------------------------------
	// Input for CTF estimation
	//-----------------------------------------------------------
	printf("%-15s", m_acCsTag);
	printf("   1. Spherical aberration in mm. The default is set to\n"
	       "      zero, meaning NO CTF estimation.\n\n");
	printf("%-15s", m_acAmpContTag);
	printf("   1. Amplitude contrast. The default is 0.07.\n\n");
	printf("%-15s", m_acExtPhaseTag);
	printf("   1. Extra phase shift in degree. The default is 0 degree,\n"
	       "      meaning NO estimation of extra phase shift.\n"
	       "   2. If a positive value is given, extra phase shift will\n"
	       "      estimated in a range centered at the given value. The\n"
	       "      range is limited within [0, 180] degrees.\n\n");
	//------------------------------------------------------------
	printf("%-15s\n", m_acAlignTag);
	printf("   Generate aligned sum (1) or simple sum (0)\n\n");
	//----------------------------------------------------------
	printf("%-15s\n", m_acThrowTag);
	printf("   Throw initial number of frames, default is 0\n\n");
	printf("%-15s\n", m_acTruncTag);
	printf("   Truncate last number of frames, default is 0\n\n");
	//------------------------------------------------------------
	printf("%-15s\n", m_acSumRangeTag);
	printf("   1. Sum frames whose accumulated doses fall in the\n");
	printf("      specified range. The first number is the minimum\n");
	printf("      dose and the second is the maximum dose.\n");
	printf("   2. The default range is [3, 25] electrons per square\n");
	printf("      angstrom.\n");
	//------------------------------------------------------------------
	printf("%-15s\n", m_acGroupTag);
	printf("   1. Group every specified number of frames by adding\n");
	printf("      them together. The alignment is then performed\n");
	printf("      on the group sums. The so measured motion is\n");
	printf("      interpolated to each raw frame.\n");
	printf("   2. The 1st integer is for gobal alignment and the\n");
	printf("      2nd is for patch alignment.\n\n");
	//-----------------------------------------------
	printf("%-15s\n", m_acCropTag);
	printf("   1. Crop the loaded frames to the given size.\n");
	printf("   2. By default the original size is loaded.\n\n");
	//----------------------------------------------------------
	printf("%-15s\n", m_acFmRefTag);
	printf("   Specify a frame in the input movie stack to be the\n");
	printf("   reference to which all other frames are aligned. The\n");
	printf("   reference is 1-based index in the input movie stack\n");
	printf("   regardless how many frames will be thrown. By default\n");
	printf("   the reference is set to be the central frame.\n\n");
	//-------------------------------------------------------------
	printf("%-15s\n", m_acOutStackTag);
	printf("   1. It is followed by two integers used to specify if\n");
	printf("      the aligned stack will be generated.\n");
	printf("   2. When the 1st integer is set to 1, the aligned stack\n");
	printf("      will be created.\n");
	printf("   3. The 2nd integer specifies the z binning, i.e, the\n");
	printf("      number of aligned frames to be summed in each\n");
	printf("      output frame in the aligned stack.\n\n");
	//-----------------------------------------------------
	printf("%-15s\n", m_acRotGainTag);
	printf("   Rotate gain reference counter-clockwise.\n");
	printf("   0 - no rotation, default,\n");
	printf("   1 - rotate 90 degree,\n");
	printf("   2 - rotate 180 degree,\n");
	printf("   3 - rotate 270 degree.\n\n");
	//--------------------------------------
	printf("%-15s\n", m_acFlipGainTag);
	printf("   Flip gain reference after gain rotation.\n");
	printf("   0 - no flipping, default,\n");
	printf("   1 - flip upside down, \n");
	printf("   2 - flip left right.\n\n");
	//------------------------------------
	printf("%-15s\n", m_acInvGainTag);
	printf("   Inverse gain value at each pixel (1/f). If a orginal\n");
	printf("   value is zero, the inversed value is set zero.\n");
	printf("   This option can be used together with flip and\n");
	printf("   rotate gain reference.\n\n");
	//--------------------------------------
	printf("%-15s\n", m_acMagTag);
	printf("   1. Correct anisotropic magnification by stretching\n");
	printf("      image along the major axis, the axis where the\n");
	printf("      lower magificantion is detected.\n");
	printf("   2. Three inputs are needed including magnifications\n");
	printf("      along major and minor axes and the angle of the\n");
	printf("      major axis relative to the image x-axis in degree.\n");
	printf("   3. By default no correction is performed.\n\n");
	//---------------------------------------------------------
	printf("%-15s\n", m_acInFmMotionTag);
	printf("   1. 1 - Account for in-frame motion.\n");
	printf("      0 - Do not account for in-frame motion.\n\n");
	//----------------------------------------------------------
	printf("%-15s\n", m_acGpuIDTag);
	printf("   GPU IDs. Default 0.\n");
	printf("   For multiple GPUs, separate IDs by space.\n");
	printf("   For example, %s 0 1 2 3 specifies 4 GPUs.\n\n",
		   m_acGpuIDTag);
	//-----------------------
	printf("%-15s\n", m_acGpuMemUsageTag);
	printf("   1. GPU memory usage, default 0.5, meaning 50%% of GPU\n");
	printf("      memory will be used to buffer movie frames.\n");
	printf("   2. The value should be between 0 and 0.5. When 0 is given,\n");
	printf("      all movie frames are buffered on CPU memory.\n\n");
	//---------------------------------------------------------------
	printf("%-15s\n", m_acUseGpusTag);
	printf("   1. Specify number of GPUs out of free GPUs to use in the \n");
	printf("      current process. By default, all free GPUs are used\n");
	printf("   2. If less free GPUs are found than the specified number,\n");
	printf("      the process will continue. If zero is found, the\n");
	printf("      the process will quit.\n\n");
	//-----------------------------------------
	printf("%-15s\n", m_acSplitSumTag);
	printf("   1. Generate odd and even sums using odd and even frames\n");
	printf("      respectively when this option is enabled.\n\n");
	//------------------------------------------------------------
	printf("%-15s\n", m_acOutStarTag);
	printf("   1. Generate the star file for Relion 4 polishing. By\n");
	printf("      Default, it is diaabled. Set 1 to enable.\n\n");
}

void CInput::Parse(int argc, char* argv[])
{
	m_argc = argc;
	m_argv = argv;
	//------------
	memset(m_acInMrcFile, 0, sizeof(m_acInMrcFile));
	memset(m_acInTifFile, 0, sizeof(m_acInTifFile));
	memset(m_acInEerFile, 0, sizeof(m_acInEerFile));
	memset(m_acInSuffix, 0, sizeof(m_acInSuffix));
	memset(m_acGainMrc, 0, sizeof(m_acGainMrc));
	memset(m_acDarkMrc, 0, sizeof(m_acDarkMrc));
	memset(m_acOutMrcFile, 0, sizeof(m_acOutMrcFile));
	memset(m_acArchiveFolder, 0, sizeof(m_acArchiveFolder));
	memset(m_acDefectFile, 0, sizeof(m_acDefectFile));
	memset(m_acInAlnFolder, 0, sizeof(m_acInAlnFolder));
	memset(m_acOutAlnFolder, 0, sizeof(m_acOutAlnFolder));
	memset(m_acDefectMap, 0, sizeof(m_acDefectMap));
	memset(m_acFullSumMrc, 0, sizeof(m_acFullSumMrc));
	memset(m_acTmpFile, 0, sizeof(m_acTmpFile));
	memset(m_acLogDir, 0, sizeof(m_acLogDir));
	memset(m_acFmIntFile, 0, sizeof(m_acFmIntFile));
	//----------------------------------------------
	int aiRange[2];
	Util::CParseArgs aParseArgs;
	aParseArgs.Set(argc, argv);
	aParseArgs.FindVals(m_acInMrcTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acInMrcFile);
	//-------------------------------------------
	aParseArgs.FindVals(m_acInTifTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acInTifFile);
	//-------------------------------------------
	aParseArgs.FindVals(m_acInEerTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acInEerFile);
	//-------------------------------------------
	aParseArgs.FindVals(m_acInSuffixTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acInSuffix);
	//------------------------------------------
	aParseArgs.FindVals(m_acOutMrcTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acOutMrcFile);
	//--------------------------------------------
	aParseArgs.FindVals(m_acFmIntFileTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acFmIntFile);
	//-------------------------------------------
	aParseArgs.FindVals(m_acArchiveTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acArchiveFolder);
	//-----------------------------------------------
	aParseArgs.FindVals(m_acFullSumMrcTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acFullSumMrc);
	//--------------------------------------------
	aParseArgs.FindVals(m_acGainMrcTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acGainMrc);
	//-----------------------------------------
	aParseArgs.FindVals(m_acDarkMrcTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acDarkMrc);
	//-----------------------------------------
	aParseArgs.FindVals(m_acDefectFileTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acDefectFile);
	//--------------------------------------------
	aParseArgs.FindVals(m_acDefectMapTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acDefectMap);
	//-------------------------------------------
	aParseArgs.FindVals(m_acTmpFileTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acTmpFile);
	//-----------------------------------------
	aParseArgs.FindVals(m_acLogDirTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acLogDir);
	int iSize = strlen(m_acLogDir);
	if(iSize > 0 && m_acLogDir[iSize-1] != '/')
	{	strcat(m_acLogDir, "/");
	}
	//------------------------------
	aParseArgs.FindVals(m_acInAlnTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acInAlnFolder);
	//---------------------------------------------
	aParseArgs.FindVals(m_acOutAlnTag, aiRange);
	aParseArgs.GetVal(aiRange[0], m_acOutAlnFolder);
	if(strlen(m_acOutAlnFolder) > 0)
	{	memset(m_acInAlnFolder, 0, sizeof(m_acInAlnFolder));
	}
	//----------------------------------------------------------
	aParseArgs.FindVals(m_acEerSamplingTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iEerSampling);
	//-------------------------------------------
	aParseArgs.FindVals(m_acPatchesTag, aiRange);
	aParseArgs.GetVals(aiRange, m_aiNumPatches);
	if(m_aiNumPatches[0] <= 1) m_aiNumPatches[0] = 0;
	if(m_aiNumPatches[1] <= 1) m_aiNumPatches[1] = 0;
	if(m_aiNumPatches[2] < 0) m_aiNumPatches[2] = 0;
	if(m_aiNumPatches[2] > 100) m_aiNumPatches[2] = 100;
	//--------------------------------------------------
	aParseArgs.FindVals(m_acIterTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iIterations);
	//------------------------------------------
	aParseArgs.FindVals(m_acTolTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fTolerance);
	//-----------------------------------------
	aParseArgs.FindVals(m_acBftTag, aiRange);
	if(aiRange[1] > 2) aiRange[1] = 2;
	aParseArgs.GetVals(aiRange, m_afBFactor);
	//---------------------------------------
	aParseArgs.FindVals(m_acPhaseOnlyTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iPhaseOnly);
	//-----------------------------------------
	aParseArgs.FindVals(m_acFourierBinTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fFourierBin);
	//------------------------------------------
	aParseArgs.FindVals(m_acInitDoseTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fInitDose);
	//----------------------------------------
	aParseArgs.FindVals(m_acFmDoseTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fFmDose);
	//--------------------------------------
	aParseArgs.FindVals(m_acPixelSizeTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fPixelSize);
	//-----------------------------------------
	aParseArgs.FindVals(m_acKvTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iKv);
	//----------------------------------
	// Input for CTF estimation
	//----------------------------------
	aParseArgs.FindVals(m_acCsTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fCs);
	//-------------
	aParseArgs.FindVals(m_acAmpContTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fAmpCont);
	//-------------
	aParseArgs.FindVals(m_acExtPhaseTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fExtPhase);
	//----------------------------------------
	aParseArgs.FindVals(m_acAlignTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iAlign);
	//-------------------------------------
	aParseArgs.FindVals(m_acThrowTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, m_aiThrow+0);
	//---------------------------------------
	aParseArgs.FindVals(m_acTruncTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, m_aiThrow+1);
	//---------------------------------------
	aParseArgs.FindVals(m_acSumRangeTag, aiRange);
	if(aiRange[1] > 2) aiRange[1] = 2;
	aParseArgs.GetVals(aiRange, m_afSumRange);
	if(m_afSumRange[0] < 0) m_afSumRange[0] = 0;
	//------------------------------------------
	aParseArgs.FindVals(m_acGroupTag, aiRange);
	if(aiRange[1] > 2) aiRange[1] = 2;
	aParseArgs.GetVals(aiRange, m_aiGroup);
	if(m_aiGroup[0] < 1) m_aiGroup[0] = 1;
	if(m_aiGroup[1] < 1) m_aiGroup[1] = 1;
	//------------------------------------
	aParseArgs.FindVals(m_acFmRefTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iFmRef);
	//-------------------------------------
	aParseArgs.FindVals(m_acOutStackTag, aiRange);
	if(aiRange[1] > 2) aiRange[1] = 2;
	aParseArgs.GetVals(aiRange, m_aiOutStack);
	//----------------------------------------
	aParseArgs.FindVals(m_acCropTag, aiRange);
	if(aiRange[1] > 2) aiRange[1] = 2;
	aParseArgs.GetVals(aiRange, m_aiCropSize);
	m_aiCropSize[0] = m_aiCropSize[0] / 2 * 2;
	m_aiCropSize[1] = m_aiCropSize[1] / 2 * 2;
	//----------------------------------------
	aParseArgs.FindVals(m_acSerialTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iSerial);
	//--------------------------------------
	aParseArgs.FindVals(m_acRotGainTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iRotGain);
	//---------------------------------------
	aParseArgs.FindVals(m_acFlipGainTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iFlipGain);
	//----------------------------------------
	aParseArgs.FindVals(m_acInvGainTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iInvGain);
	//---------------------------------------
	if(m_piGpuIds != 0L) delete[] m_piGpuIds;
	aParseArgs.FindVals(m_acGpuIDTag, aiRange);
	if(aiRange[1] >= 1)
	{	m_iNumGpus = aiRange[1];
		m_piGpuIds = new int[m_iNumGpus];
		aParseArgs.GetVals(aiRange, m_piGpuIds);
	}
	else
	{	m_iNumGpus = 1;
		m_piGpuIds = new int[m_iNumGpus];
		m_piGpuIds[0] = 0;
	}
	m_piGpuMems = new int[m_iNumGpus];
	memset(m_piGpuMems, 0, sizeof(int) * m_iNumGpus);
	//-----------------------------------------------
	aParseArgs.FindVals(m_acUseGpusTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iUseGpus);
	if(m_iUseGpus <= 0) m_iUseGpus = m_iNumGpus;
	if(m_iUseGpus > m_iNumGpus) m_iUseGpus = m_iNumGpus;
	//--------------------------------------------------
	aParseArgs.FindVals(m_acTiltTag, aiRange);
	if(aiRange[1] > 2) aiRange[1] = 2;
	aParseArgs.GetVals(aiRange, m_afTilt);
	//------------------------------------
	m_afMag[0] = 1.0f;
	m_afMag[1] = 1.0f;
	aParseArgs.FindVals(m_acMagTag, aiRange);
	if(aiRange[1] == 3) aParseArgs.GetVals(aiRange, m_afMag);
	//-------------------------------------------------------
	m_iInFmMotion = 0;
	aParseArgs.FindVals(m_acInFmMotionTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iInFmMotion);
	//------------------------------------------
	aParseArgs.FindVals(m_acGpuMemUsageTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_fGpuMemUsage);
	if(m_fGpuMemUsage < 0) m_fGpuMemUsage = 0.0f;
	else if(m_fGpuMemUsage > 0.8) m_fGpuMemUsage = 0.8f;
	//--------------------------------------------------
	aParseArgs.FindVals(m_acSplitSumTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iSplitSum);
	//----------------------------------------
	if(m_iSplitSum != 0)
	{	if(m_fGpuMemUsage > 0.5f) m_fGpuMemUsage = 0.5f;
	}
	//------------------------------------------------------
	aParseArgs.FindVals(m_acOutStarTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iOutStarFile);
	//-------------------------------------------
	aParseArgs.FindVals(m_acTiffOrderTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iTiffOrder);
	//---------------------------------------------
	aParseArgs.FindVals(m_acCorrInterpTag, aiRange);
	if(aiRange[1] > 1) aiRange[1] = 1;
	aParseArgs.GetVals(aiRange, &m_iCorrInterp);
	mPrint();
}

void CInput::GetBinnedSize(int* piImgSize, int* piBinnedSize)
{
	piBinnedSize[0] = piImgSize[0];
	piBinnedSize[1] = piImgSize[1];
	if(m_fFourierBin <= 0) return;
	//----------------------------
	piBinnedSize[0] = (int)(piImgSize[0] / m_fFourierBin + 0.5f);
	piBinnedSize[1] = (int)(piImgSize[1] / m_fFourierBin + 0.5f);
	piBinnedSize[0] = piBinnedSize[0] / 2  * 2;
	piBinnedSize[1] = piBinnedSize[1] / 2 * 2;
}

bool CInput::IsInMrc(void)
{	
	if(strlen(m_acInMrcFile)) return true;
	else return false;
}

bool CInput::IsInTiff(void)
{
	if(strlen(m_acInTifFile)) return true;
	else return false;
}

bool CInput::IsInEer(void)
{
	if(strlen(m_acInEerFile)) return true;
	else return false;
}

float CInput::GetFinalPixelSize(void)
{
	float fFinalSize = m_fPixelSize * m_fFourierBin;
	if(m_afMag[0] > m_afMag[1]) fFinalSize /= m_afMag[0];
	else fFinalSize /= m_afMag[1];
	return fFinalSize;
}

float CInput::GetFinalPixelSize(float fPixelSize)
{
	float fFinalSize = fPixelSize * m_fFourierBin;
        if(m_afMag[0] > m_afMag[1]) fFinalSize /= m_afMag[0];
        else fFinalSize /= m_afMag[1];
        return fFinalSize;
}

void CInput::mPrint(void)
{
	printf("\n");
	printf("%-15s  %s\n", m_acInMrcTag, m_acInMrcFile);
	printf("%-15s  %s\n", m_acInTifTag, m_acInTifFile);
	printf("%-15s  %s\n", m_acInEerTag, m_acInEerFile);
	printf("%-15s  %s\n", m_acInSuffixTag, m_acInSuffix);
	printf("%-15s  %s\n", m_acOutMrcTag, m_acOutMrcFile);
	printf("%-15s  %s\n", m_acFmIntFileTag, m_acFmIntFile);
	printf("%-15s  %s\n", m_acArchiveTag, m_acArchiveFolder);
	printf("%-15s  %s\n", m_acFullSumMrcTag, m_acFullSumMrc);
	printf("%-15s  %s\n", m_acGainMrcTag, m_acGainMrc);
	printf("%-15s  %s\n", m_acDarkMrcTag, m_acDarkMrc);
	printf("%-15s  %s\n", m_acDefectFileTag, m_acDefectFile);
	printf("%-15s  %s\n", m_acDefectMapTag, m_acDefectMap);
	printf("%-15s  %s\n", m_acInAlnTag, m_acInAlnFolder);
	printf("%-15s  %s\n", m_acOutAlnTag, m_acOutAlnFolder);
	printf("%-15s  %s\n", m_acTmpFileTag, m_acTmpFile);
	printf("%-15s  %s\n", m_acLogDirTag, m_acLogDir);
	printf("%-15s  %s\n", m_acFmIntFileTag, m_acFmIntFile);
	printf("%-15s  %d\n", m_acSerialTag, m_iSerial);
	printf("%-15s  %d\n", m_acEerSamplingTag, m_iEerSampling);
	printf("%-15s  %d  %d  %d\n", m_acPatchesTag,
           m_aiNumPatches[0], m_aiNumPatches[1], m_aiNumPatches[2]);
	printf("%-15s  %d\n", m_acIterTag, m_iIterations);
	printf("%-15s  %.2f\n", m_acTolTag, m_fTolerance);
	printf("%-15s  %.2f %.2f\n", m_acBftTag,
           m_afBFactor[0], m_afBFactor[1]);
	printf("%-15s  %d\n", m_acPhaseOnlyTag, m_iPhaseOnly);
	printf("%-15s  %.2f\n", m_acFourierBinTag, m_fFourierBin);
	printf("%-15s  %.2f\n", m_acInitDoseTag, m_fInitDose);
	printf("%-15s  %.2f\n", m_acFmDoseTag, m_fFmDose);
	printf("%-15s  %.2f\n", m_acPixelSizeTag, m_fPixelSize);
	printf("%-15s  %d\n", m_acKvTag, m_iKv);
	printf("%-15s  %.2f\n", m_acCsTag, m_fCs);
	printf("%-15s  %.2f\n", m_acAmpContTag, m_fAmpCont);
	printf("%-15s  %.2f\n", m_acExtPhaseTag, m_fExtPhase);
	printf("%-15s  %d\n", m_acThrowTag, m_aiThrow[0]);
	printf("%-15s  %d\n", m_acTruncTag, m_aiThrow[1]);
	printf("%-15s  %.2f  %.2f\n", m_acSumRangeTag,
	   m_afSumRange[0], m_afSumRange[1]);
	printf("%-15s  %d\n", m_acSplitSumTag, m_iSplitSum);
	printf("%-15s  %d  %d\n", m_acGroupTag, m_aiGroup[0], m_aiGroup[1]);
	printf("%-15s  %d\n", m_acFmRefTag, m_iFmRef);
	printf("%-15s  %d  %d\n", m_acOutStackTag, 
	   m_aiOutStack[0], m_aiOutStack[1]);
	printf("%-15s  %d\n", m_acRotGainTag, m_iRotGain);
	printf("%-15s  %d\n", m_acFlipGainTag, m_iFlipGain);
	printf("%-15s  %d\n", m_acInvGainTag, m_iInvGain);
	printf("%-15s  %d\n", m_acAlignTag, m_iAlign);
	printf("%-15s  %.2f  %.2f\n", m_acTiltTag, m_afTilt[0], m_afTilt[1]);
	printf("%-15s  %.2f  %.2f  %.2f\n", m_acMagTag, m_afMag[0],
	   m_afMag[1], m_afMag[2]);
	printf("%-15s  %d\n", m_acInFmMotionTag, m_iInFmMotion);
	printf("%-15s  %d  %d\n", m_acCropTag, m_aiCropSize[0], 
	   m_aiCropSize[1]);
	printf("%-15s", m_acGpuIDTag);
	for(int i=0; i<m_iNumGpus; i++)
	{	printf("  %d", m_piGpuIds[i]);
	}
	printf("\n");
	printf("%-15s  %d\n", m_acUseGpusTag, m_iUseGpus);
	printf("%-15s  %.2f\n", m_acGpuMemUsageTag, m_fGpuMemUsage);
	printf("%-15s  %d\n", m_acOutStarTag, m_iOutStarFile);
	printf("%-15s  %d\n", m_acTiffOrderTag, m_iTiffOrder);
	printf("%-15s  %d\n", m_acCorrInterpTag, m_iCorrInterp);
	printf("\n\n");
}
