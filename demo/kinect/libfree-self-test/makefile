CC=gcc -lfreenect_sync -I/usr/local/include/libfreenect/

all: run-self-test

run-self-test: self-test.c
	${CC} -o run-self-test self-test.c

clean:
	rm -f run-self-test
