CC=g++

CFLAGS=
LDFLAGS=

SOURCES = redtrack.cpp
OBJECTS = $(SOURCES:.cpp=.o)

EXECUTABLE = redtrack

CFLAGS += -c -Wall -std=c++11
CFLAGS += -I /home/blutack/Sources/mavconn/mavconn/src 
CFLAGS += -I /home/blutack/Sources/mavconn/mavlink 
CFLAGS += -I /home/blutack/Sources/mavconn/mavlink/pixhawk

LDFLAGS += `pkg-config --libs opencv`
LDFLAGS += -L /home/blutack/Sources/mavconn/mavconn/build/lib
LDFLAGS += -lmavconn_cam -lmavconn_shm -lmavconn_lcm -llcm

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS) 
	$(CC) $(OBJECTS) -o $@ $(LDFLAGS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@
clean:
	$(RM) *.o *~ $(EXECUTABLE)
