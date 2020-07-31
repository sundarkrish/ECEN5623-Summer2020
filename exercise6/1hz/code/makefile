CC=gcc
CFLAGS=
 
all: 1hz 


clean:
	-rm -f *.o *.pgm *.ppm
	-rm -f 1hz 
	-sudo rm -f /dev/mqueue/my_queue

1hz: 1hz.o
	$(CC) -o 1hz 1hz.o -lrt -lpthread

1hz.o: 1hz.c
	$(CC) $(CFLAGS) -c 1hz.c




