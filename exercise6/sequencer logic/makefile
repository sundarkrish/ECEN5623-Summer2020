CC=gcc
CFLAGS=
 
all: sequencer


clean:
	-rm -f *.o 
	-rm -f sequencer


sequencer: sequencer.o
	$(CC) -o sequencer sequencer.o -lrt -lpthread

sequencer.o: sequencer.c
	$(CC) $(CFLAGS) -c sequencer.c


