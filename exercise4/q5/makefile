



CC=gcc
CFLAGS=
 
all: test


clean:
	-rm -f *.o *.ppm *.pgm 
	-rm -f test

test: test.o
	$(CC) -o test test.o -lrt -lpthread

test.o: test.c
	$(CC) $(CFLAGS) -c test.c




