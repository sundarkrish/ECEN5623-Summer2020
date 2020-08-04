CC=gcc
CFLAGS=
 
all: 1hz 10hz 5hz 


clean:
	-rm -f *.o *.pgm *.ppm
	-rm -f 1hz 10hz 5hz 
	-sudo rm -f /dev/mqueue/my_queue /dev/mqueue/my_queue1

1hz: 1hz.o
	$(CC) -o 1hz 1hz.o -lrt -lpthread

1hz.o: 1hz.c
	$(CC) $(CFLAGS) -c 1hz.c

10hz: 10hz.o
	$(CC) -o 10hz 10hz.o -lrt -lpthread

10hz.o: 10hz.c
	$(CC) $(CFLAGS) -c 10hz.c

5hz: 5hz.o
	$(CC) -o 5hz 5hz.o -lrt -lpthread

5hz.o: 5hz.c
	$(CC) $(CFLAGS) -c 5hz.c





