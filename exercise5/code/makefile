CC=gcc
CFLAGS=
 
all: 1hz sequencer


clean:
	-rm -f *.o *.pgm 
	-rm -f 1hz sequencer

1hz: 1hz.o
	$(CC) -o 1hz 1hz.o -lrt -lpthread

1hz.o: 1hz.c
	$(CC) $(CFLAGS) -c 1hz.c


sequencer: sequencer.o
	$(CC) -o sequencer sequencer.o -lrt -lpthread

sequencer.o: sequencer.c
	$(CC) $(CFLAGS) -c sequencer.c


