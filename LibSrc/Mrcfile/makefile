PROJ_DIR = $(shell pwd)/..
INCDIR	= $(PROJ_DIR)/Include
LIBDIR  = $(PROJ_DIR)/Lib

SRCS    = Header/CLoadExtHeader.cpp \
	Header/CLoadMainHeader.cpp \
	Header/CMainHeader.cpp \
	Header/CSaveExtHeader.cpp \
	Header/CSaveMainHeader.cpp \
	Image/CLoadImage.cpp \
	Image/CSaveImage.cpp \
	Image/C4BitImage.cpp \
	App/CLoadMrc.cpp \
	App/CSaveMrc.cpp \
	App/CReviseExtHeader.cpp \
	App/CMrcScale.cpp \
	App/CVerticalFlip.cpp

OBJS	= $(patsubst %.cpp, %.o, $(SRCS))
OUT	= libmrcfile.a

CC = g++ -std=c++11
CFLAGS	= -m64 -c -g -pthread -IInclude -I$(INCDIR)

all: $(OBJS)
	@ar rcs $(OUT) $(OBJS)
	@mv -f $(OUT) $(LIBDIR)
	@cp -f Include/*.h $(INCDIR)/Mrcfile
	@echo Static library $(OUT) has been created.

compile: $(OBJS)
	@echo Compilation has been done.

%.o: %.cpp
	@echo Compile $<
	@$(CC) $(CFLAGS) $< -o $@

clean:
	@rm -f $(OUT) $(OBJS) *.*~ makefile~
