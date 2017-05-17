#
# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS= -g -Wall -O2 `pkg-config --cflags lcm`
LIBS= `pkg-config --libs lcm` -lwiringPi
LDFLAGS = -L/usr/local/lib
INCLUDES= -I../\
    -I../inc \
    -I../rocket \
    -I/usr/local/include

SRCS_TENDOF = tenDOF.cpp\
              tenDOF_main.cpp
OBJS_TENDOF = $(SRCS_TENDOF:.cpp=.o)

SRCS_GYRO = I3G4250D.cpp \
			I3G4250D_main.cpp
OBJS_GYRO = $(SRCS_GYRO:.cpp=.o)

MAINS = tenDOF gyroscope

DEBUG = $(DEBUG)

.PHONY: all clean remake

all: $(MAINS)
	@echo Done compiling $(MAINS)

tenDOF: $(OBJS_TENDOF)
	g++ -o $@ $^ $(LIBS) $(LDFLAGS)

gyroscope: $(OBJS_GYRO)
	g++ -o $@ $^ $(LIBS) $(LDFLAGS)

.cpp.o:
	g++ $(CFLAGS) $(INCLUDES) $(DEBUG) -c $< -o $@

clean:
	$(RM) *.o *~ $(MAINS)

remake: clean all
	@echo Finished remake

