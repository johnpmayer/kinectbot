CC=gcc -Wall -std=c99
I=-I/usr/local/include/libfreenect
L=-lfreenect_sync -lm -lpthread
SRC=main.c

all: clean system

system: main.c
	${CC} ${I} -o system ${SRC} ${L}

clean:
	rm -f system

