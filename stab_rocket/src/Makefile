#
CFLAGS= -g -Wall -O2
LIBS= -llcm
INCLUDES= -I../\
    -I../inc \
    -I../rocket

SRCS = tenDOF.cpp tenDOF_main.cpp
OBJS = $(SRCS:.cpp=.o)

MAINS = tenDOF 

DEBUG = $(DEBUG)


.PHONY: all clean remake

all: $(MAINS)
	@echo Done compiling $(MAINS)

$(MAINS): $(OBJS)
	g++ $(CFLAGS) $(INCLUDES) $(DEBUG) -o $(MAINS) $(OBJS) $(LIBS)

.cpp.o:
	g++ $(CFLAGS) $(INCLUDES) $(DEBUG) -c $< -o $@

clean:
	$(RM) *.o *~ $(MAINS)

remake: clean all
	@echo Finished remake
