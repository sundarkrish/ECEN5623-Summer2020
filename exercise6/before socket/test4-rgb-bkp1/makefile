CC=gcc
CFLAGS=
 
all: 1hz 10hz sequencer


clean:
	-rm -f *.o *.pgm *.ppm
	-rm -f 1hz 10hz sequencer
	-sudo rm -f /dev/mqueue/my_queue

1hz: 1hz.o
	$(CC) -o 1hz 1hz.o -lrt -lpthread

1hz.o: 1hz.c
	$(CC) $(CFLAGS) -c 1hz.c

10hz: 10hz.o
	$(CC) -o 10hz 10hz.o -lrt -lpthread

10hz.o: 10hz.c
	$(CC) $(CFLAGS) -c 10hz.c


sequencer: sequencer.o
	$(CC) -o sequencer sequencer.o -lrt -lpthread

sequencer.o: sequencer.c
	$(CC) $(CFLAGS) -c sequencer.c


