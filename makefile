PRJHOME = $(shell pwd)
CONDA = $(HOME)/miniconda3
CUDAHOME = $(HOME)/nvidia/cuda-10.1
CUDAINC = $(CUDAHOME)/include
CUDALIB = $(CUDAHOME)/lib64
PRJINC = $(PRJHOME)/LibSrc/Include
PRJLIB = $(PRJHOME)/LibSrc/Lib
#-----------------------------
CUSRCS = ./Util/GAddFrames.cu \
	./Util/GCalcMoment2D.cu \
	./Util/GFindMinMax2D.cu \
	./Util/GFFTUtil2D.cu \
	./Util/GFourierResize2D.cu \
	./Util/GPartialCopy.cu \
	./Util/GPositivity2D.cu \
	./Util/GRoundEdge.cu \
	./Util/GNormalize2D.cu \
	./Util/GPad2D.cu \
	./Util/GPhaseShift2D.cu \
	./Util/GCorrLinearInterp.cu \
	./Util/GThreshold2D.cu \
	./BadPixel/GCombineBadMap.cu \
	./BadPixel/GCorrectBad.cu \
	./BadPixel/GDetectHot.cu \
	./BadPixel/GDetectPatch.cu \
	./BadPixel/GLabelPatch.cu \
	./BadPixel/GLocalCC.cu \
	./MrcUtil/G4BitImage.cu \
	./MrcUtil/G90Rotate2D.cu \
	./MrcUtil/GAugmentRef.cu \
	./MrcUtil/GApplyRefsToFrame.cu \
	./MrcUtil/GFlip2D.cu \
	./MrcUtil/GInverse2D.cu \
	./EerUtil/GAddRawFrame.cu \
	./Align/GCorrelateSum2D.cu \
	./Align/GNormByStd2D.cu \
	./Align/GCC2D.cu \
	./Correct/GCorrectPatchShift.cu \
	./Correct/GStretch.cu \
	./Correct/GWeightFrame.cu \
	./MotionDecon/GDeconFrame.cu \
	./MotionDecon/GMotionWeight.cu \
        ./FindCtf/GCtfCC1D.cu \
	./FindCtf/GCtfCC2D.cu \
	./FindCtf/GCalcSpectrum.cu \
	./FindCtf/GRmBackground2D.cu \
	./FindCtf/GCalcCTF1D.cu \
	./FindCtf/GCalcCTF2D.cu \
	./FindCtf/GRadialAvg.cu
CUCPPS = $(patsubst %.cu, %.cpp, $(CUSRCS))
#------------------------------------------
SRCS = ./Util/CCufft2D.cpp \
	./Util/CFileName.cpp \
	./Util/CFlipImage.cpp \
	./Util/CFourierCrop2D.cpp \
	./Util/CGroupFrames.cpp \
	./Util/CNextItem.cpp \
	./Util/CPad2D.cpp \
	./Util/CParseArgs.cpp \
	./Util/CSplineFit1D.cpp \
	./Util/CRemoveSpikes1D.cpp \
	./Util/CSaveTempMrc.cpp \
	./Util/CSimpleFuncs.cpp \
	./Util/CMultiGpuBase.cpp \
	./DataUtil/CMrcStack.cpp \
	./DataUtil/CAlnSums.cpp \
	./DataUtil/CReadFmIntFile.cpp \
	./DataUtil/CFmIntParam.cpp \
	./DataUtil/CFmGroupParam.cpp \
	./DataUtil/CDataPackage.cpp \
	./DataUtil/CStackFolder.cpp \
	./BadPixel/CCorrectMain.cpp \
	./BadPixel/CDetectMain.cpp \
	./BadPixel/CLocalCCMap.cpp \
	./BadPixel/CTemplate.cpp \
	./Align/CZbinStack.cpp \
	./Align/CAlignBase.cpp \
	./Align/CAlignedSum.cpp \
	./Align/CAlignMain.cpp \
	./Align/CAlignParam.cpp \
	./Align/CGenXcfStack.cpp \
	./Align/CFullAlign.cpp \
	./Align/CInterpolateShift.cpp \
	./Align/CAlignStack.cpp \
	./Align/CIterativeAlign.cpp \
	./Align/CExtractPatch.cpp \
	./Align/CMeasurePatches.cpp \
	./Align/CPatchAlign.cpp \
	./Align/CPatchShifts.cpp \
	./Align/CPeak2D.cpp \
	./Align/CSaveAlign.cpp \
	./Align/CLoadAlign.cpp \
	./Align/CSimpleSum.cpp \
	./Align/CStackShift.cpp \
	./Align/CTransformStack.cpp \
	./Align/CDetectFeatures.cpp \
	./Align/CPatchCenters.cpp \
	./Align/CEarlyMotion.cpp \
	./Correct/CCorrectFullShift.cpp \
	./Correct/CGenRealStack.cpp \
	./MotionDecon/CInFrameMotion.cpp \
	./MrcUtil/CAnalyzeMrc.cpp \
	./MrcUtil/CApplyRefs.cpp \
	./MrcUtil/CAsyncSingleSave.cpp \
	./MrcUtil/CLoadCryoEMStack.cpp \
	./MrcUtil/CLoadStack.cpp \
	./MrcUtil/CSaveSingleCryoEM.cpp \
	./MrcUtil/CSumFFTStack.cpp \
	./MrcUtil/CTiltAngles.cpp \
	./TiffUtil/CLoadTiffHeader.cpp \
	./TiffUtil/CLoadTiffImage.cpp \
	./TiffUtil/CLoadTiffStack.cpp \
	./EerUtil/CLoadEerHeader.cpp \
	./EerUtil/CLoadEerFrames.cpp \
	./EerUtil/CDecodeEerFrame.cpp \
	./EerUtil/CRenderMrcStack.cpp \
	./EerUtil/CLoadEerMain.cpp \
	./FindCtf/CCtfTheory.cpp \
	./FindCtf/CGenAvgSpectrum.cpp \
	./FindCtf/CFindCtfBase.cpp \
	./FindCtf/CFindCtf1D.cpp \
	./FindCtf/CFindCtf2D.cpp \
	./FindCtf/CFindDefocus1D.cpp \
	./FindCtf/CFindDefocus2D.cpp \
	./FindCtf/CFindCtfMain.cpp \
	./FindCtf/CFindCtfHelp.cpp \
	./CInput.cpp \
	./CGpuBuffer.cpp \
	./CStackBuffer.cpp \
	./CBufferPool.cpp \
	./CLoadRefs.cpp \
	./CCheckFreeGpus.cpp \
	./CProcessThread.cpp \
	./CSaveSerialCryoEM.cpp \
	./CGenStarFile.cpp \
	./CMain.cpp \
	./CMotionCor3.cpp \
	$(CUCPPS)
OBJS = $(patsubst %.cpp, %.o, $(SRCS))
#-------------------------------------
CC = g++ -std=c++11
CFLAG = -c -g -pthread -m64
NVCC = $(CUDAHOME)/bin/nvcc -std=c++11
CUFLAG = -Xptxas -dlcm=ca -O2 \
	-gencode arch=compute_75,code=sm_75 \
	-gencode arch=compute_70,code=sm_70 \
	-gencode arch=compute_52,code=sm_52 \
        -gencode arch=compute_53,code=sm_53 \
        -gencode arch=compute_60,code=sm_60 \
        -gencode arch=compute_61,code=sm_61 
#------------------------------------------
cuda: $(CUCPPS)

compile: $(OBJS)

exe: $(OBJS)
	@$(NVCC) -g -G -m64 $(OBJS) \
	$(PRJLIB)/libmrcfile.a $(PRJLIB)/libutil.a \
	-Xlinker -no-pie \
	-L$(CUDALIB) \
	-L$(CONDA)/lib \
	-L/usr/lib64 \
	-lcufft -lcudart -lcuda -lnvToolsExt -ltiff -lc -lm -lpthread \
	-o MotionCor3
	@echo MotionCor3 has been generated.

%.cpp: %.cu
	@echo "-----------------------------------------------"
	@$(NVCC) -cuda -cudart shared \
		$(CUFLAG) -I$(PRJINC) \
		-I$(CONDA)/include $< -o $@
	@echo $< has been compiled.

%.o: %.cpp
	@echo "------------------------------------------------"
	@$(CC) $(CFLAG) -I$(PRJINC) -I$(CUDAINC) \
		-I$(CONDA)/include \
		$< -o $@
	@echo $< has been compiled.

clean:
	@rm -f $(OBJS) $(CUCPPS) *.h~ makefile~ MotionCor3

