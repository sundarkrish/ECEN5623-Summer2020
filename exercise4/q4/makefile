



CC=gcc
CFLAGS=
 
all: capture_sh


clean:
	-rm -f *.o *.ppm 
	-rm -f capture_sh

capture_sh: capture_sh.o
	$(CC) -o capture_sh capture_sh.o -lrt -lpthread

capture_sh.o: capture_sh.c
	$(CC) $(CFLAGS) -c capture_sh.c




