04/14/2015
----------
MrcUtil::CMrcStack::GetVoxels(void)
    1. Its return type has been changed from int to size_t to avoid
       overflow when handling a stack 7676 x 7420 x 38

Align::CFitPatchShifts3D::mCalcThreshold
    1. dStd calculation error. Forgot to divide dStd by iCount.


04/15/2015
----------
1. Comparison between phase shift interpolation and CUDA Texture 
   interpolation.
   1) In GCorrectFrame.cu, add fFullShiftX and fFullShiftY in 
      mGCorrect3D function. Use these two values to replace
      the 3D fit values for texture interpolation.
   2) Compare so generated sum versus sum by full frame alignment
      that is phase shift based. They are almost identical. The
      ctffind scores are 0.18405 (phase shift) and 0.18354 
      (texture interpolation).
   
2. Bug in GFFTStack.cu::DoIt
   1) if(bClean || pfCmpFrame != 0L) should be
      if(bClean && pfCmpFrame != 0L) 

04/16/2015
----------
1. For patch based alignment, we made the following changes
   1) We first crop FFT stack based upon user input.
   2) Once the full frame alignment is done, correction is applied
      to the FFT stack. This stack is then transformed back to
      real space.
   3) Patches are extracted from this corrected stack. Alignment
      is done each patch.
   4) Therefore, the mean motion of all patches should be zero since
      the global motion has been corrected.
   5) This improves the score of CTFFIND.

2. Comparison has been made between phase shift interpolation versue
   CUDA texture interpolation based upon full frame alignment. It 
   has been found the CUDA texture interpolation yields a slightly
   worse CTFFIND score 0.18146 versus 0.18367.

3. It looks like when the fit model is used to calculate the shift, we
   should not use to exceed the fitting range.

04/17/2015
----------
1. We rollback Change 1 made on 04/16/2015.
   1) The alignment including full, focused, and patched alignments
      are performed on the original data set. User specified Fourier
      binning is performed on the aligned sum.
   2) However, Fourier cropping is still used during cross correlation
      to speedup the alignment process. The alignment results are 
      scaled back to the original pixel.
2. The resson we want to work on the unbinned data set is due to the
   smearing effect caused by CUDA texture interpolation.
3. The major change is in patch based alignment. 
   1) After full frame alignment, the measured shifts are corrected 
      right away by phase shift.
   2) Individual pataches are than extracted from this corrected
      stack. The aubsequent alignments are performed on these
      patches to obtain local motions.
   3) Local motions are corrected using texture interpolation.
   4) Tests on Klim's super res stacks shows more than 10% improvements
      on CTFFIND score by patch based alignment as opposed to full
      frame alignment. 

04/24/2015
----------
1.  Added -throw flag. 
    1) This flag throws away the specified number of frames at the 
       begining of each stack. These frames will not be loaded from 
       the input MRC file.
    2) As a result the initial dose of the stack is equal to the
       product of the frame dose and the number of frames thrown.

2.  Revised CParseArgs::FindVals
    1) Changed the way how to detect next tag. The criterion is
       to check the first and the second character. The first one
       must be '-' and the second be letter.   

04/29/2015
----------
1. Renamed the package from DfCorr to UcsfDfCorr

2. Added -FmRef flag.
    1) -FmRef 0 indicates to align stack to the first frame.
    2) All other values align to the central frame.
    3) The central frame is the one that has the mean shift.


05/01/2015
----------
1. Extended bad pixel patch detection on multiple GPUs.
     1) Revised CInput.cpp to read multiple GPUs.
     2) Revised BadPixel/CDetectPatch to split the image sum into
        multiple vertical stripes. Let each CDetectThread handle
        a stripe.
     3) Added BadPixel/CDetectThread. It detects bad pixel patches
        on a given stripe.

05/07/2015
----------
1. Extended Fourier transform of stack on multiple GPU.


05/12/2015
----------
1. Apply gain reference to stack.
     1) If gain refernece does not exist, convert non-float stack to
        float stack.
     2) Now we can apply gain reference to short/unsigned short stacks.

05/13/2015
----------
1. Focused alignment is found to have a bug. The corrected images are 
   very grainy.
     1) Revised CFocusedAlign.cpp according to CFullAlign.cpp

05/14/2015
----------
1. When processing TOMO DF tilt series, it is found that the summed image
   show artifacts since the second stack. There is no artifact in the
   first stack.
   1) In CProcessThread::mCorrectBadPixels(void), add
      BadPixel::CDetectMain::DeleteInstance(). This fixes the problem.
   2) It is likely CDetectMain has uncleared state that propagates into
      next execution.

05/18/2015
----------
1. The same bug was not fixed on 05/14/2015.
   1) Another bug was found in BadPixel::CDetectHot::DoIt. When m_pucBadMap
      is allocated, memset(m_pucBadMap, 0, sizeof(char)) creates a bug.
   2) Correction: memset(m_pucBadMap, 0, iPixels * sizeof(char))

05/19/2015
----------
1. Align::CStackShift::GetCentralFrame(void)
   1) Return m_iNumFrames / 2 instead according to the request.

05/21/2015
----------
1. In CPatchAlignThread, add a mechanism that repeats the alignment with
   a patch of bigger size if the current alignment fails.
2. The new size if twice as much as the old size, but will not exceed
   half of the full size.

06/03/2015
----------
1. Use the 3rd order polynomial instead of the 2nd order in z direction.
   Align::CFitShifts3D.cpp has been changed accordingly.
   Align::GCorrectStack.cu has been changed accordingly.

06/06/2015
----------
1. Add a function that can batch-process all MRC files in a folder for
   single particle data sets.
2. Users provide a template that is the full path truncated before the
   serial number in the file. This template is specified after -InMrc.
3. Users needs to specify -Serial 1 to enable the sequential process.
  
06/27/2015
----------
1. Extensively use cudaDeviceEnablePeerAccess for multi-GPU computation
   in summing FFT stack. 

07/14/2015
----------
1. Add -Kv for dose weighting.
2. Add -Trunc for truncating last a few frames


07/17/2015
----------
1. Add -bft for B-factor
2. Change cuda interpolation from clamp to self-implemented wrapping.
   This reduces the artifact along the edge.

08/04/2015
----------
1. For single-particle cryoEM stacks, if users specify -kV -FmDose and
   -PixSize, both dose weighted and unweighted sums will be generated.
2. For single-particle cryoEM stacks, no extended header will be 
   saved. The output MRC file contains only 1024 bytes main header.

08/05/2015
----------
1. Add -Tilt for tomographic stacks. This is only required for dose
   weighting in order to determine the initial dose at each tilt.
2. -Tilt should be followed by start angle and tilt step angle.
3. To enable dose weighting, users also need to provide parameters
   for -kV -PixSize -FmDose.

08/13/2015
----------
1. Add -Group that groups every specified number of frames by summing
   them together. This is equivalent to bin them in time domain.
2. The alignment is then performed on the time-binned stack.
3. The shifts of the orginal stack is obtained by interpolation of
   the shifts measured on the binned stack. 

08/30/2015
----------
1. Add support to TIFF input file. Users use -InTiff tag to designate
   the following file is TIFF file.
2. -Serial is not supported for TIFF input.

09/11/2015
----------
1. Add -InSuffix for serial drift correction of multiple stack files
   in a folder. When -InSuffix is present, valid files are those whose
   names must contain the suffix that follows -InSuffix.
2. Implement gridding correction in Util/GGriddingCorrect.cu. It can
   be called in Align/GCorrectStack.cu. A test was performed and showed
   that CTF estimate scores are deteriorated and the detectable Thon
   ring has a minor improvement. It is disabled due to limited 
   benefit and significant computational cost.
3. In Align/CFullAlign.cpp, disabled resetting shift to zero when alignment
   does not converge. This practice seems to deteriorate patch alignment
   accuracy.

09/19/2015
----------
1. Add -Square tag that allows to trim frames to square. The new frames
   have the smaller dimesnsion of the input stack. If the smaller dimension
   has an odd size, an extra pixel will be truncate to ensure the new
   frames have even pixels in each dimension.

09/21/2015
----------
1. Replace -Square with -Crop that is followed by the cropped size in x
   and y dimensions.
2. Add a new criterion for the determination of tilt series. If the first
   float in the ext header (tilt angle) is non-zero, the input file name
   is deemed to be a tilt series.

11/04/2015
----------
1. Implemented logging functions for CFullAlign, CFocusedAlign, and
   CPatchAlign. These functions are enabled when -LogFile tag is
   followed with a full path to log file. 
   Note: Don't include ".log" at the end.

11/14/2015
----------
1. Add check for user's mistake that enters the same file name for input
   and output file.

12/07/2015
----------
1. Change made inside CSumStack.cpp::DoIt. When multiple GPUs are used,
   cudaMemcpy is used to copy the sum of partial stack to the first
   GPU instead of addressing the memory of various GPUs.
   This fix now makes the code working for multiple GPUs of different
   K10 cards. However, on K80 cards, the code works only on single
   GPU. The results are wrong when 2 or more GPUs are used.

12/27/2015
----------
1. CBinStackInTime.cpp. Discard fitting the measured shifts of time-binned
   frames to polynomial function. Use linear interpolation and extra-
   polation for end points instead. Polynomial fitting has a large errors
   when the shifts at early exposure change rapidly.

01-11-2016
----------
1. Change the 3D fitting strategy for stacks containing large numbers
   of frames collected with high dose (> 30e/A^2).
   1) For patch based alignment, two 3D models are used, one for early
      frames and one for the later frames.
   2) The measured shifts are therefore fit and the corresponding frames
      are corrected based upon these two different models.
2. The affected classes are: CPatchShifts, CFitPatchShifts3D, and
   GCorrectStack.cu  

03-16-2016
----------
1. Implement -trunc function that truncates the specified trailing
   frames from the stack when it is loaded from a MRC file 

04-29-2016
----------
1. Implement -trunc function when a TIFF stack is loaded.

07-06-2016
----------
1. Rename UcsfDfCorr to MotionCor2
2. Add -cudart shared to makefile

07-30-2016
----------
1. Bug in BadPixel/GLocalCC.cu. In mGLocalCC, when fStd is negative, the old
   code sets it to (float)1e-30. This could lead very large fCC value.
   Fix: set negative fStd zero. the corresponding fCC is set zero too.

08-08-2016
----------
1. Add -OutStack function that generates motion-corrected stack (unweighted)
   when it is enabled by -OutStack 1.
2. The differences between image sums are around 1e-5 between this version
   and the older version.

08-09-2016
----------
1. Add -InitDose that specifies the dose received before the stack has been
   acquired.
2. Allow user to specify -Gpu 1,2,3 in addition to -Gpu 1 2 3.

08-09-2016
----------
1. Enable -OutStack for simple sum operation. When -OutStack is enabled,
   MotionCor2 generates a raw stack that is gain applied and bad pixel
   detected.

08-19-2016
----------
1.  Add "-RotGain" that rotates the gain reference counter clockwise 
    before it is applied to frames. 
       0: no rotation, 1: rotate 90 degree, 2: rotate 180 degree
       3: rotate 270 degree
2.  Add "-FlipGain" that flip the gain reference before it is applied
    to frames.
       0: no flipping, 1: flip upside down, 2: flip left right.
3.  When both -RotGain and -FlipGain are set, rotation is done first
    and then flipping is performed.
4.  Rotate 90 degree is the same as "DM::Process::Rotate Right". This has
    been verified. 
5.  DM rotation given in "DM::Camera::Configure Camera" conforms to
    counter clockwise convention, according to Tom Sha
6.  When both rotation and flip are set, DM rotates first and then flip
    according to Tom Sha
   
09-21-2016
----------
1.  According to Sjors's suggestion, write error message to stderr.
2.  Add memory check for insufficient memory in CLoadStack.
3.  Return non-zero value when program exists abnormally.


10-03-2016
----------
1.  Fix a bug in CAnalyzeMrc::IsTomoMrc. Use angular range instead of
    angular step to determine if the input is tomo mrc. If the range
    is less equal than 2 deg, it is deemed not tomo mrc.

10-10-2016
----------
1. Fix a bug in CPatchShifts::mCreateFitStackShift3D.
   When the fit shift is calculate, mistakenly use absolute frame number
   as z. It must be float z = (float)(i - m_aFit3D.m_iRefFrm);

10-14-2016
----------
1. Bug: reported by MRC lab, the min, max, mean are not saved in MRC header.
   This bug is caused by calling CSaveMainHeader.SetMin, SetMax, SetMean
   directly. It should be called from CSaveMrc.DoIt().

10-18-2016
----------
1. Revision: In accordance with new MRC format given in JSB 2015, Mrcfile
   module has been revised including its API. The most significant change
   is pixel size is needed in filling the main header. As a result,
   1) CSaveTomoMrc, CSaveSingleCryoEM, CSaveSerialCryoEM in MrcUtil have
      been revised.
   2) CProcessThread has been revised to include a member m_fPixelSize.

10-19-2016
----------
1. 1) Issue: The fit shift does not match very well with the measured shift
      fort stacks containing not many frames.
   2) Cause: In CPatchShifts::Fit3D(void) the iRefFrm was set to frame 3.
      This causes the fitting in the frame range of [0:3) has not enough
       data for fitting.
   3) Solution: Change the smallest iRefFrm to frame 7. We tested on various
      stacks the fitted shifts are closer to the measured shifts.

11-09-2016
----------
1. Changed to weighted fitting of measured local motion. This change was
   made in Align/CFitPatchShifts3D.cpp

11/14/2016
----------
1. Changed -FmRef. Now the default is -1 that aligns to the central frame.
   Users can specify any frame as the reference to which all other frames
   are aligned to.

01/30/2017
----------
1. Add correction of anisotropic magnification.
2. Add archiving function that packs 8-bit stack into 4-bit stack

03/15/2017
----------
1. In mag correction, change to adopt the output from Tim Grant's
   program mag_distortion_estimate. -Mag major_scale minor_scale
   major_axis
2. Since mag_distortion_estimate uses the coordinate system where
   the y-axis points downward and MotionCor2's y-axis points
   upward, MotionCor2 changes the input major_axis to -major_axis
   in CAlignParam.cpp

03/21/2017
----------
1. In CProcessThread, add check for mismatch between gain size
   and stack size. If they do not match, quit.

05/09/2017
----------
1. Removed CAlignBase::mFlipImages. We flip/rotate gain reference, thus
   do not need to flip images anymore.
2. Also removed m_bGainFlipped from CAlignParam.
3. Add a new feature that saves the full-frame motion-corrected sum
   right after global motion correction but before local motion
   correction. The major changes are in CPatchAlign.cpp and
   Correct/CGenRealStack that returns the motion corrected sum and
   CInput.cpp that provides a new tag "-FullSum" followed by the path
   of the MRC file storing this sum.

06/01/2017
---------- 
1. Revised TiffUtil/CLoadTiffHeader.cpp, CLoadTiffImage.cpp.
2. Removed TiffUtil/CTiffHeader.cpp
3. Tested the new Tiff file loading code and compared with the old code,
   they are identical.
4. Tiff loading loaded rows per strip, not tile by tile. Therefore, loading
   by tiles are not tested.

06/09/2017
----------
1. Forgot to perform mag correction when only full-frame motion correction
   is specified. 
2. Add mCorrectMag(m_gCmpSumDW, m_aiCmpSize) in CCorrFullShiftThread.cpp

06/19/2017
----------
1. Revised in-frame motion correction. This function contains tw-step
   down-weighting. It first performs directional down-weighting based
   upon the global motion. In the second step, isotropic down-weighting
   is performed based upon the estimate of local motion.

06/26/2017
----------
1. Bug: when users do not provide gain reference, MotionCor2 thinks the
   dimension of gain reference mismatches the stack dimension and thus
   aborts except for stacks of MRC mode 2. Note that no gain reference
   is allowed, MotionCor2 should skip gain correction and proceed.
2. Add mCheckGain in CProcessThread to fix this bug.

06/30/2017
----------
1. Add function for serial processing TIFF stacks.

07/13/2017
----------
1. Add a function to load binary defect map. The defect map created by
   clip (David Mastronarde) is not 0-1 binary map but 128-129. Masking
   with 0x1 solves this problem.

09/18/2017
----------
1. Much improved computational efficiency by buffering as many frames
   in the GPU memory as possible.
2. Very important, from now on no GPU can be shared by two processes. 
3. -Patch can take an optional third parameter for overlapping between
   adjacent patches.

11/02/2017 - Version 1.0.2
--------------------------
1. Fixed a bug in Util/CFourierCrop2D.cpp. When fBin = 1.0f, the input gCmp
   is transformed into real space and not transformed back to Fourier space.
   This becomes a bug when aligned stack is generated without Fourier
   cropping. The generated DW weighted sum is actually the real part of
   Fourier transform. The fix is to buffer the gCmp in the CPU memory
   and then copy back to gCmp.
  
11/06/2017 - Version 1.0.3
--------------------------
1. Fixed Bug: When -DefectFile is enabled, the program crashed near the
   end of execution. This bug is in CDetectMain::mLoadDefectFile. When
   m_pucBadMap is allocated, the code mistakenly calls memset with
   the sizeof(int) * iPixels. It should be sizeof(char) * iPixels.
2. Fixed Bug: In Correct/GWeightFrame.cu, convert the initial dose
   to number of pre-exposed frames m_iPreexposedFrames = (int)(fInitDose
   / fFrameDose + 0.5f). The weights are caluclated based upon the
   collected frames plus m_iPreexposedFrames. The first frame in the 
   stack corresponding to m_iPreexposedFrames in the total exposure.

12/06/2017
----------
1. Fixed Bug: In BadPixel/GCorrectBad.cu::mGCorrect, next = next 
   * 1103515245 + 12345 seems overflow. Replaced with (next * 7)
   % iWinSize. This change solved the improper correction of a
   defect line in Garrett's tomo data set.

01/09/2017
----------
1. In this version CFitShifts3D.cpp has been changed. In the
   fitting model the highest order in x-y increased to 3rd
   order whereas the highest order in t is reduced to 2nd
   order. The higher order of the polynomial is 5.
2. In this version GCorrPatchShiftThread.cu has been changed.
   We use Gaussian-distance based interpolation in 3x3 pixels
   instead of bilinear interpolation.
3. The polynomial model has been changed. The highest order in t
   has been increased to 3. The highest order of polynomial is 6. 

01/13/2018
----------
1. Fixed Bug: In BadPixel/CCorrectMain.cpp::ThreadMain, m_gucBadMap has
   the padded size, not frame size. Use GFFTStack::m_aiPadSize instead
   of m_aiStkSize.

02/03/2018
----------
1. Bug in BadPixel/CLocalCCMap::mLocalCC:
   if(m_gfCC != 0L) cudaFree(m_gfCC)
   m_gfCC is never set to NULL before calling this function.
   Fix: add m_gfCC = 0L in constructor.

03/28/2018
----------
1. In this version GCorrPatchShiftThread.cu has been changed.
   The use of Gaussian-distance based interpolation in 3x3 pixels
   has been abandoned. Switch back to bilinear interpolation.
2. In Align/GCorrelateSum.cu, the amplitude is replaced by its square
   root to avoid floating point overflow. This happens when frame
   counts are high and there are large number of pixels in K3 frames.
   If not doing so, the inverse Fourier transform yields wrong results.

05/03/2018
----------
1. Make this version the official release and assigned version 1.1.0.
2. This is the fastest version that performs the global and local
   alignments on Fourier cropped stack. Combined with Gatan's invented
   performance improvement (reduced creation/destroy of FFT plans,
   copying only the central sub-area of XCF image), this version achieves
   ~6x speedup as opposed to version 1.0.

05-22-2020
----------
1. This version is created that fixed the memory issue. This bug is inside
   CGpuBuffer::mCalcGpuFrames that did subtraction between two size_t
   integers that are unsigned.
2. The second bug is in Proj/Mrcfile that forgot to swap bytes when needed.
3. Reduced Tiff loading threads to 2 threads only.
4. CGenRealStack and CCorrPatchShifts use multi-threads instead of
   multi-streams according to nVidia Guillaume's recommendation.
5. makefile supports only compute_50 and up.

06-15-2020
----------
1. Version 1.3.3 is created from version 1.3.2
2. Fix the bug in CFullAlign.cpp


06-24-2020
----------
1. Version 1.3.3: Util/CRemoveSpikes1D.cpp: 
   1) Polynomial fitting uses the sqrt(t) instead of t as the base term.
   2) Shifts in x and y directions are used together to determine the
      spikes in the measured shifts.
   3) Single fit has replaced the windown-based multiple fits.
2. Align/CStackShift.cpp:
   1) Replaced m_pfShifts with m_pfShiftXs and m_pfShiftYs

07-15-2020
----------
1. 1.3.3_Int1: made based upon 1.3.3
   1) This version replaces 3D fitting with diatance based interpolation.
   2) Util/CRemoveSpikes1D uses the square root of x at the base term for
      polynomial fitting for removing outliers in iterative measurement.
2. TiffUtil/CLoadTiffStack: add check of file size in OpenFile.
3. Correct/GCorrectPatchShift.cu: In randomizing location, 8111*x or
   8111*y may cause some overflow issue. Use 811 * x, 811 * y instead.
4. TiffUtil/CLoadTiffHeader: Use TIFFNumberOfDirectories instead.

08-03-2020
----------
1.3.3_Int1
1. CMain.cpp::mDoSerialCryoEMTiff and mDoSerialCryoEMMrc:
   Put mCreateBuferPool right behind m_aProcessThread.WaitForExit(-1.0f).
   This fixed the problem the crash during batch processing of movies
   that has different number of frames.
2. Fixed the memory leak in Align/CFullAlign::Align. m_pFullShift was
   created twice. The second instance created inside if-else block
   hides the first instance created before.

08-06-2020
----------
1.3.3_Int1
1. Added support for EER movies. No support for batch processing of EER yet.
2. Added 2D removal of spikes in CPatchShifts.cpp.
3. Revised 1D removal of spikes in Util/CRemoveSpikes1D.cpp. The fitted shifts
   replace the measured ones only when they are smaller to avoid erroneous
   fitting.

08-13-2020
----------
1.4.0 is created by renaming 1_3.3_Int1-bak-08-06-2020
1. Util::GFourierResize2D: Remove the two lines that set gCmpOut[iOut] to 0.
   This initialization makes the summing to assignment.

08-18-2020
----------
1.4.0 Bug:
1. Util/GFourierResize2D::DoIt: bSum and stream were both default parameter.
   But in Correct/CCorrectFullShift::mCropFrame only m_aStream[0], the 2nd
   default param appears. This causes the generation of aligned stack goes
   wrong. 
   Fix: change bSum in GFourierResize2D as a non-default param.

08-19-2020
----------
1.4.0 Algorithm change
1. Disabled in Correct/CGenRealStack::mCorrectBilinear().

08-23-2020
----------
1.4.0
1. Util::GCalcMoment2D: Allocates a small internal GPU buffer to host the
   intermedial summing results.
2. Add Util/GFindMinMax2D.cu. These two changes are for the calculation
   of min, max, and mean needed in MRC header. There were used to be
   calculated on CPU.
3. MrcUtil/CSaveSingleCryoEM.cpp has been modified to reflect these changes.
4. Fixed a bug in Align/CAlignParam::GetFrameRef. iFrameRef could be larger
   than the number of frames after truncation.

08-24-2020
----------
1.4.0
1. Revised Util/CRemoveSpikes1D.cpp. Check outliers first based upon RMS and
   then based upon the difference between the fit and measured values.

09-04-2020
----------
1.4.0
1. Added Spline fitting class CSplineFit1D.cpp Util
2. Revised Util/CRemoveSpikes.cpp to detect and remove spikes based upon
   1D spline fitting.

09-05-2020
----------
1.4.0
1. Revised Tiff/CLoadTiffStack.cpp: When running in single mode, render
   frames on GPU. In batch mode, render frames on CPU to avoid laboring
   the first GPU with both rendering and motion correction.

09-15-2020
----------
1.4.0
1. Implemented non-uniform grouping. The number of rendered frames to be
   grouped for measuring shift is based upon number of raw frames the
   rendered frames contain. A new class CFmGroupParam.cpp is added.

09-17-2020
----------
1.4.0
1. CStackFolder bug; opendir returned pointer was not closed. Fixed.

09-18-2020
----------
1.4.0
1. Reimplemented CFmIntegrateParam.cpp to handle a case where a data set
   contains movies of different number of frames. This change affects
   CFmGroupParam.cpp, CMain.cpp.

09-24-2020
----------
1.4.0
1. Restored -UseGpus by uncommenting the two lines in CMotionCor2.cpp.
2. Implemented non-uniform grouping for movies collected with non-
   uniform exposures. In CFmGroupParam, added grouping based upon
   doses since variable exposures beget variable frame doses.

09-25-2020
----------
1.4.0
1. Implemented sequential processing for EER movies
2. Fixed bug in EerUtil/CRenderMrcStack.cpp: initialize each MRC frame with
   zeroes before it is used in CDecodeEerFrame.cpp.

09-30-2020
----------
1.4.0
1. Moved printf for total time to before return statement.


04-01-2021
----------
1.4.3
1. Copied from 1.4.2. 
2. In MrcUtil/CApplyRefs.cpp: free raw frame memory after it is gain corrected.
      CApplyRefs uses cuda stream and removes threads.
      CApplyRefs does not allocate pinned memory for buffering raw frame.

05-12-2021
----------
1. 1.4.3.1: Add -OutStack 1 binZ where binZ specifies number of frames to be
   summed into a output frame before it is saved into output MRC file.
   CInput.cpp CFmGroupParam.cpp Align/CZbinStack.cpp Align/CMain.cpp

08-07-2021
----------
1.4.4 from 1.4.3.1
1. In Util/CRemoveSpikes1D.cpp, the tolerance for replacing the raw shifts
   is changed to be variable. In the first 20 frames, it is 2 pixels. It is
   reduced to 1 pixel from 21 to 50 frames and 0.5 afterwards.

10-22-2021
----------
1.4.5 from 1.4.4
1. Bug in Projs/Util/Util_Thread.cpp::WaitForExit. It should return true
   only when the iRet is 0. Corrected Util_Thread.cpp code. 

11-03-2021
----------
1.4.6 from 1.4.5
1) Replace "-LogFile" with "-LogDir". Users specify the directory where the
   log files are stored. The names of log files are the output MRC
   file name but ended with .log.
1.4.7 from 1.4.6
1) To implement screening for bad local measurements per frame. The bad
   measurements are not included in the calculation of local shift at
   each pixel. 

11-23-2021
----------
1.4.7: Made sure the split sums can be generated when dose weighting is
       not enabled.

1.4.8
-----
1. CGpuBuffer.cpp: change from bulk-allocation of pinned memory to 
   frame by frame allocation.
2. Align/CTransformStack.cpp: change to one GPU one thread model.

1.4.9: 03-15-2022
-----------------
1. Add output star file for Relion polishing
2. Bug fix: CMain.cpp::mDoSerialCryoEM. Forgot to return true at the end.

1.4.10: 05-03-2022
------------------
1. Added a function that detects features in the field of view. Patch centers
   are only selected at locations where there are features.
   1) New files: Align::CDetectFeatures.cpp, Align::CPatchCenters.cpp
   2) Affected: Align::CFullAlign.cpp, Align::CExtractPatch.cpp,
      Align::CMeasurePatches.cpp.
2. Bug fix: Util/GFindMinMax2D.cu: mGFindMin1D<<<1, m_aBlockDim, ...>>>
   when the m_aGridDim.x is smaller than m_aBlockDim.x, the kernel will
   access memory (m_gfBuf) beyound allocation. Replaced m_aBlockDim
   with m_aGridDim.
   Same correction for mGFindMax1D.
   Same correction for Util::GCalcMoment2D.cu
3. Renamed 1.4.10 to 1.5.0 on 05/31/2022

1.5.1: 08-18-2022
------------------
1. Used C++ queue to implement Load, Process, and Save queues. Initially the
   queue is populated with all the movie file names. The load thread then
   retieves the front package in the Load queue and loads the stack. The
   front package is then popped and pushed into the process queue. When
   the motion correction is done, the front package in the Process queue
   is popped and pushed into the Save queue for squential saving.
2. Major changed in CStackFolder.cpp, MrcUtil/CDataPackage.cpp, CMain.cpp

1.5.2: 08-22-2022
-----------------
1. Implemented inotify event handling in CStackFolder.cpp that detects the
   new file based on ON_CLOSE_WRITE.
2. When a new file is put in the load queue, its timestamp is used for 
   loading only newer files.
3. Add "-TiffOrder -1". This setting reads EER tiff file from last frame
   to the first frame (backward). The default is 1 and reads forward.

1.5.2: 10_25_2022
-----------------
Bug fix:
1. EerUtil/CLoadEerFrames.cpp: when reading frames backward, the counting of
   bytes read is wrong. Fix: added m_iBytesRead and increment its value with
   new bytes being read.
2. Bug fix: Saving alignment failed in CPatchAlign::mCalcPatchShifts.
   m_pFullShift was set 0L after it is passed to m_pPatchShifts.
   1) CSaveAlign change: DoGlobal and DoLocal

1.5.4: 12-27-2022
-----------------
1. Reimplemented "Group" function. The shift interpolation to each raw frame
   is done in each iteration of the iterative alignment process rather than
   at the end of all iterations.
2. "-Group" takes two parameters, one for full alignment, one for patch
   alignment.
3. Reimplemented interpolation compensation. Added "-CorrInterp". 

1.6.0: 01-23-2023
-----------------
1. Reversion from 1.5.5g as 1.6.0.
2. Added Align/CEarlyMotion.cpp. This class implements a function that
   refines the early frame motion in the first group. Now we can use
   large group sizes (>= 4) for patch measurements.
3. Added refinement in Align/CIterativeAlign.cpp.
4. Added a smoothing function in Align/CStackShift.cpp
5. Revised Align/CAlignedSum.cpp. It can generated partial sums.
6. Revised xcf and pat buffers that contain raw frames insteaded 
   of group sums.
7. Revised Align/CAlignStack.cpp accordingly where the aligned group sums
   are calculated prior to measuring the shifts of group sums.

1.6.1: 01-30-2023
-----------------
1. Correct/GWeightFrame.cu: use linear interpolation to voltage other than
   300, 200, 120 kVs.
2. Bug fixes: Tmp Buffer size may not the same as the Frm buffer size. This
   causes bugs in Correct/CCorrectFullShift.cpp. BadPixel/CDetectMain.cpp,
   MrcUtil/CApplyRefs.cpp, BadPixel/CLocalCCMap.cpp.

1.6.2: 02-15-2023
-----------------
Renamed MotionCor2_1.6.1d to MotionCor2_1.6.2
1. Default input to -Group is 1 2
   Default -Tol is 0.1
   Default -Iter is 15

1.6.3: 02-17-2023
------------------
Bug Fix:
1. TiffUtil/CLoadTiffImage.cpp: line 94.
   Even if tBytes != iStripBytes, the image can still be loaded property, so
   this check is removed.
2. BadPixel/GCorrectBad.cu:
   When a good pixel is not found in neighborhood, choose anyone in the image.

1.6.4: 03-29-2023
-----------------
1. Replace texture based bilinear interpolation with self implementation
   due to the dropped support to texture in Cuda 12.1. The affected code
   is mainly in Correct/GStretch.cu. Correct/CCorrectFullShift.cpp is
   changed accordingly. Util/CCufft2D.cpp is changed to include
   out-of-place FFT.
2. The 3rd column in the frame integration file is now optional. If it
   is not provided, the frame dose uses the value from the command line
   option "-FmDose". If neither provides the value, the dose weighting
   is disabled.

1.6.5: 04-22-2023
-----------------
Bug fix:
1. Align/CAlignBase.cpp::mCreateAlnSums:
   It did not check if pPackage->m_pAlnSums is null or not and created
      a new pAlnSums regardless.
   Fix: Check if pPackage contains m_pAlnSums. If exist, check if its
      sizes match the new aligned sums. If both are the same, reuse. 
      Otherwise, create a new one.
2. Align/CMeasurePatchShifts.cpp::mCalcPatchShift:
   It did not free the new CStackShift.
   Fix: free the new CStackShift.

06-02-2023
----------
1. Use the libtiff installed with conda. makefile and makefile11 are revised
   to point the correct path to libtiff include and lib.

06-08-2023
----------
Bug fix:
1. When CProcessThread.cpp calls CSaveSerialCryoEM::Save(bAsync=true),
   CProcessThread may end before CSaveSerialCryoEM thread starts. mWaitSaveThread
   in CMain.cpp may return instaneously.
   Fix: pop the package out of the process queue before CSaveSerialCryoEM thread
        is invoked in CSaveSerialCryoEM::Save().
Changes:
1. CStackShift::DisplayShifts: Roll back to the old format such that Relion
   can parse it.

08-09-2023
----------
Changes:
1. CBufferPool.cpp: allow odd and even sums to be calculated for simple sums.
   MrcUtil/CSumFFTStack.cpp: add odd/evn sum calculation.
   Align/CSimpleSum.cpp: add odd/evn sum calculation

MotionCor2 1.7.0 [09-25-2023]
-----------------------------
1. Integrated GCtfFind into MotionCor2. Added a folder FindCtf. The entry
   point is in Correct/CCorrectFullShift.cpp

MotionCor3 1.0.0 [10-04-2023]
-----------------------------
1. Renamed to MotionCor3
2. Bug fix: Correct/CCorrectFullShift.cpp::mEstimateCtf: pass the gpu
   buffer frame from Tmp buffer stack.
3. Bug fix: FindCtf/CFindDefocus1D::mBrutalForceSearch: check if iPsSteps
   is 1. If so, fPsStep is set 0.

MotionCor3 1.0.1 [10-12-2023]
-----------------------------
1. Align/CAlignMain.cpp::DoIt: Forgot m_pPackage = pPackage
2. MrcUtil/CLoadCryoEMStack::OpenFile: DU::CDataPackage* pPackage =
      s_pStackFolder->GetPackage(bPop). Correct: s_pPackage = ......
3. Fixed the bug that produces a black ODD/EVN image when both -Align 0
   and -SplitSum 1 are present.

MotionCor3_1.1.0 [01-31-2024]
-----------------------------
1. Created LibSrc folder that has Util, Mrcfile, Include, and Lib subfolders.
   - Util has the source code of libutil.a that can be generated by running
     "make clean" followed by "make all" inside Util.
   - Mrcfile has the source code of libmrcfile.a that can be generated by
     running "make clean" followed by "make all" inside Mrcifle.
   - MotionCor3 makefile and makefile11 use LibSrc/Include and LibSrc/Lib.
     MotionCor2/Include and MotionCor3/Lib have been deleted.
2. Deleted libcuutil.a and cleaned up "#include <CuUtil/xxxx> in MotionCor3
   source code.
3. Added GFFT1D.cu in Util.
4. FindCtf/CGenAvgSpectrum::mRmBackground: fMinFreq = 1.0f/30.0f instead of
   1.0f/15.0f.
5. FindCtf/CFindCtfBase.cpp: m_adResRange change to [15, 2.5].

MotionCor3_1.1.1 [02-18-2024]
-----------------------------
1. Reimplemented DataUtil/CFmIntParam.cpp renamed from CFmIntegrateParam.
   The new implementation takes into accound of the case wheren the sum
   of the 1st column is less than the input movie stack size. (02/18/2024)
2. Added -InSkips in the command line for skipping any inputs files whose
   file names contain the specified strings.
3. DataUtil/CBufferPool::Adjust: m_pPatBuffer will be NULL when patch
   align is not specified. Check NULL before calling Adjust.
4. Bug fix: DataUtil/CGpuBuffer::AdjustBuffer:
   if(iNumFrames <= m_iMaxGpuFrms)
   {    if(iNumFrames < m_iMaxGpuFrms)  <----- wrong!
        {       m_iNumGpuFrames = iNumFrames;
        } <----------------------------------- wrong!
        m_iNumFrames = iNumFrames;
        return;
   }

MotionCor3_1.1.2 [06-11-2024]
-----------------------------
1. Revised CCheckFreeGpus.cpp:
   1) When -UseGpus is not present in command line, do not check. This
      means all specified GPUs following -GPU will be used.
   2) When -UseGpus specifies the same as or more GPUs than -Gpu provides,
      do not check. This means all specified GPUs following -GPU will be used.
   3) This is based on Github user request.

MotionCor3_1.1.3 [10-17-2024]
-----------------------------
1. Revised CGenStarFile.cpp:
   1) Fixed spelling errors and added FtBin entry.

MotionCor3_1.1.4 [10-23-2024]
-----------------------------
1. Bug fix: Align/CLoadAlign::mReadSetting(): bug in reading m_aiStkSize.


MotionCor3_1.1.5 [01-20-2025]
-----------------------------
1. 1) Bug fix: Align/CPatchShifts::SetRawShift: Indexing error in 
      setting raw shifts.
   2) Bug fix: DataUtil/CFmGroupParam.cpp: Incorrect calculating the
      group center based on the total raw frames.
   3) Bug fix: Align/CEarlyMotion.cpp: Confusion of node frame and node center.
      Node frame refers to the index of the corresponding integrated frame.
      Node center refers to the middle raw frame of integrated in this
      integrated frame.
2. Increased B-factor from 10 to 100 in Correct/GCorrectPatchShift.cu improves
   local motion correction.

MotionCor3_1.1.6 [04-12-2025]
-----------------------------
1. Bug Fix:
2. Changes:
   1) Correct/GCorrectPatchShift.cu: implemented upsampling to make the
      interpolation on upsampling grid and Fourier-crop to original grid.

MotionCor3_1.1.7 [04-12-2025]
-----------------------------
1. Bug Fix:
2. Changes:
   1) Merge the implementation in GCtfFind into FindCtf. Note that the local
      CTF measurement on tilt images has not been implemented.

MotionCor3_1.2.0 [04-18-2025]
-----------------------------
1. Bug Fix:
   1) FindCtf/GCalcCTF2D::mGEmbedCtf: negative frequency mapping to positive
      frequency. Corrected: iY = (iCmpY - y) % iCmpY
   2) FindCtf/CFindDefocus2D::mRefinePhase: correct upper limit.
2. Changes:
   1) Added Thon ring resolution estimation. 
   2) FindCtf/CFindDefocus2D: reduced lowpass strength from 100 to 40 to
      account for more high frequency signals.
   3) CRescaleImage.cpp in FindCtf to expand Thon distance for high defocus images
      collected at high magnifications (< 1.25 A).

MotionCor3 1.2.1 [05-14-2025]
-----------------------------
1. Bug Fix:
   1) FindCtf/GCalcCTF2D: It calculates CTF^2, inconsistent with GCalcCTF1D,
      which calculates CTF.
   2) Because of 1), both GCtfCC1D and GCtf2D have been revised to correlate
      with |CTF| - 0.5.
   3) Because of 1), GSpectralCC2D has been changed to correlate against
      |CTF| - 0.5f.
2 Changes:
   1) FindCtf/CFindDefocus2D: The B-factor has been dropped from 40 to 16
      to include more high-frequency information. This has been found more
      accurate in the estimation of small phase shift in LPP.

MotionCor3 1.2.2 [05-19-2025]
-----------------------------
1. Bug Fix:
2: Changes:
   1) makefile11: add compute capability 9.0 for H100 and H200.
   2) Added CSaveMovieDone.cpp to track what movie files have been processed.
