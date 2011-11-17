CC = g++
LD = g++
LDFLAGS =
#CFLAGS=-g -Wall `pkg-config --cflags libfreenect` -lopengl32 -lglut32
CFLAGS=-g -Wall
#LIBS = `pkg-config --libs libfreenect` 
LIBS = -L/usr/local/lib -I/usr/local/include/libfreenect
OBJECTS = main.o
PROG = mycppview

all:$(PROG)

$(PROG): $(OBJECTS)
	$(LD) $(LDFLAGS) $(LIBS) $(OBJECTS) -o $(PROG)

%.o: %.cpp
	$(CC) $(CFLAGS)  $(LIBS) -c $<

clean:
	rm -rf *.o $(PROG)
