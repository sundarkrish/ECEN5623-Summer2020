#Code re-use: http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/simplethread/Makefile



CC=gcc
CFLAGS=

all:


clean:
	-rm -f *.o 
	-rm -f simple_thread incdec_thread question4

simple_thread: simple_thread.o
	$(CC) -o simple_thread simple_thread.o -lrt -lpthread

simple_thread.o: simple_thread.c
	$(CC) $(CFLAGS) -c simple_thread.c

incdec_thread: incdec_thread.o
	$(CC) -o incdec_thread incdec_thread.o -lrt -lpthread

incdec_thread.o: incdec_thread.c
	$(CC) $(CFLAGS) -c incdec_thread.c

question4: question4.o
	$(CC) -o question4 question4.o -lrt -lpthread

question4.o: question4.c
	$(CC) $(CFLAGS) -c question4.c


