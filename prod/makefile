
NAME = roombacmd
VERSION = 1.0
DISTNAME = $(NAME)-$(VERSION)

CFLAGS = -Wall -I. -I/usr/local/include/libfreenect
L=-lpthread -lm -lfreenect_sync
CC = gcc -DSIMPLE_MAIN

BINARIES = main

all: $(BINARIES)

clean:
	rm -f $(BINARIES)

.c.o:
	$(CC) $(CFLAGS) -o $@ $@.c $(L)

main: main.c roombalib.c
	$(CC) $(CFLAGS) $^ -o $@ $(L)
