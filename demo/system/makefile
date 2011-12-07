CC=gcc -Wall -std=c99
I=-I/usr/local/include/libfreenect
CC+=$I
L=-lfreenect_sync
SRC=main.c
SRC+=../roomba/roombalib.c

all: clean system

system: main.c
	${CC} -o system ${SRC} ${L}

clean:
	rm -f system