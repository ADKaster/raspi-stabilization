#
# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS= -g -Wall -O2 `pkg-config --cflags lcm`
LIBS= `pkg-config --libs lcm`
INCLUDES= -I../\
    -I../inc \
    -I../rocket

SRCS_TENDOF = tenDOF.cpp\
              tenDOF_main.cpp
OBJS_TENDOF = $(SRCS_TENDOF:.cpp=.o)

MAINS = tenDOF 

DEBUG = $(DEBUG)


.PHONY: all clean remake

all: $(MAINS)
	@echo Done compiling $(MAINS)

tenDOF: $(OBJS_TENDOF)
	g++ -o $@ $^ $(LIBS)

.cpp.o:
	g++ $(CFLAGS) $(INCLUDES) $(DEBUG) -c $< -o $@

clean:
	$(RM) *.o *~ $(MAINS)

remake: clean all
	@echo Finished remake

