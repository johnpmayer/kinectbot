CC=gcc -std=c99 -Wall
I=-I/usr/local/include/libfreenect
#L=-L/usr/local/lib
L=-lfreenect_sync -lm -lncurses
CC+=${I}
CC+=${L}

all: clean run-color
	ls run-color # Compilation results:

#run-self-test: self-test.c bmp.c
#	${CC} -o run-self-test self-test.c ${L}

run-color: self-test.c bmp.c
	${CC} -DCOLOR -o run-color self-test.c ${L}

clean:
	rm -f run-color

