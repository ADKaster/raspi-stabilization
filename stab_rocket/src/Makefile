#
CFLAGS= -g -Wall -O2
LIBS= -llcm
INCLUDES= -I../\
    -I../inc \
    -I../rocket

SRCS = tenDOF.cpp tenDOF_main.cpp
OBJS = $(SRCS:.cpp=.o)

MAINS = tenDOF

.PHONY: all clean

all: $(MAINS)
	@echo Done compiling $(MAINS)

$(MAINS): $(OBJS)
	g++ $(CFLAGS) $(INCLUDES) -o $(MAINS) $(OBJS) $(LIBS)

#tenDOF.o: tenDOF.cpp
#	g++ $(CFLAGS) $(INCLUDE) $(LIBS) tenDOF.cpp9

#tenDOF_main.o: tenDOF_main.cpp
#	g++ $(CFLAGS) $(INCLUDE) $(LIBS) tenDOF_main.cpp

.cpp.o:
	g++ $(CFLAGS) $(INCLUDES) -c $< -o $@

clean:
	$(RM) *.o *~ $(MAINS)
