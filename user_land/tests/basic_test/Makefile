CC=gcc
CFLAGS=-Wall -pedantic

all: a_start o_stop

a_start: start.c
	$(CC) $(CFLAGS) -o a_start start.c

o_stop:  stop.c
	$(CC) $(CFLAGS) -o o_stop  stop.c

clean:
	rm -f *~
	rm -f o_stop a_start 