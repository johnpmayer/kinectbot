CC=gcc -std=c99 -Wall
I=-I/usr/local/include/libfreenect
#L=-L/usr/local/lib
L=-lfreenect_sync
CC+=${I}
CC+=${L}

all: run-self-test

run-self-test: self-test.c
	${CC} -o run-self-test self-test.c ${L}

clean:
	rm -f run-self-test

