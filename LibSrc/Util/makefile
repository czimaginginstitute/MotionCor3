PROJ_DIR = $(shell pwd)/..
SRCS = $(wildcard *.cpp)
HDRS = $(patsubst %.cpp, %.h, $(SRCS))
OBJS = $(patsubst %.cpp, %.o, $(SRCS))
#-------------------
OUT = libutil.a

CC = g++ -std=c++11
CFLAG = -c -g -pthread -m64

all: $(OBJS)
	@echo create library $(OUT) and move to $(PROJ_DIR)/Lib
	@echo copy headers to $(PROJ_DIR)/Include/Util	
	@ar rcs $(OUT) $(OBJS)
	@mv $(OUT) $(PROJ_DIR)/Lib
	@cp $(HDRS) $(PROJ_DIR)/Include/Util

%.o: %.cpp
	$(CC) $(CFLAG) $< -o $@

list:
	@echo $(OBJS)

clean:
	@echo delete all object files and temporary files	
	@rm -f $(OBJS)
	@rm -f $(PROJ_DIR)/Util/*.*~
	@rm -f $(PROJ_DIR)/Util/*~
