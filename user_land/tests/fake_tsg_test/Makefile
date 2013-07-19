CC=gcc
CFLAGS= -ggdb -Wall -pedantic

all: x_tsg

x_tsg: fake_tsg.c
	$(CC) $(CFLAGS) -o x_tsg fake_tsg.c

clean:
	rm -f *~
	rm -f \#*
	rm -f x_tsg