TIFDIR  = $(PROJDIR)/Tiffile
INCDIR	= $(PROJDIR)/libtiff/include

SRCS    = $(TIFDIR)/CLoadTiffImage.cpp \
	$(TIFDIR)/CLoadTiffHeader.cpp \
	$(TIFDIR)/CTiffHeader.cpp

OBJS	= $(patsubst %.cpp, %.o, $(SRCS))
OUT	= libtiffile.a

CC	= g++
CFLAGS	= -m64 -c -g -pthread -I$(INCDIR) -I$(TIFDIR)/Include

all: $(OBJS)
	@ar rcs $(OUT) $(OBJS)
	@mv -f $(OUT) $(PROJDIR)/Lib
	@cp -f $(TIFDIR)/*.h $(PROJDIR)/Include/Tiffile
	@echo Static library $(OUT) has been created.

compile: $(OBJS)
	@echo Compilation has been done.

%.o: %.cpp
	@echo Compile $<
	@$(CC) $(CFLAGS) $< -o $@

clean:
	@rm -f $(OUT) $(OBJS) *.*~ makefile~
