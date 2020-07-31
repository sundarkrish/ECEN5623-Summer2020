CC=gcc
CFLAGS=
 
all: 10hz 


clean:
	-rm -f *.o *.pgm *.ppm
	-rm -f 10hz 
	-sudo rm -f /dev/mqueue/my_queue



10hz: 10hz.o
	$(CC) -o 10hz 10hz.o -lrt -lpthread

10hz.o: 10hz.c
	$(CC) $(CFLAGS) -c 10hz.c





