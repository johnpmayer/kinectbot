CC=gcc -Wall -std=c99
I=-I/usr/local/include/libfreenect
CC+=$I
L=-lfreenect_sync
SRC=main.c
SRC+=../roomba/roombalib.c

all: system

system: main.c
	${CC} -o system ${SRC} ${L}