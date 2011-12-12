CC=gcc -std=c99 -Wall
I=-I/usr/local/include/libfreenect
CC+=$I

all: regions.o

regions.o: depth_regions.c
	${CC} -c depth_regions.c
