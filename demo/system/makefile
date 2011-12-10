CC=gcc -Wall -std=c99
I=-I/usr/local/include/libfreenect
I+=-I../roomba
CC+=$I
L=-lfreenect_sync -lm -lpthread
SRC=main.c
SRC+=roombalib.c
SRC+=simpletest.c
SRC+=depth_regions.c

all: clean system

system: main.c
	${CC} -o system ${SRC} ${L}

clean:
	rm -f system