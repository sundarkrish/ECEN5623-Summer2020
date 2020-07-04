


CC=gcc
CFLAGS=

all: heap_mq posix_mq


clean:
	-rm -f *.o 
	-rm -f heap_mq posix_mq

heap_mq: heap_mq.o
	$(CC) -o heap_mq heap_mq.o -lrt -lpthread

heap_mq.o: heap_mq.c
	$(CC) $(CFLAGS) -c heap_mq.c

posix_mq: posix_mq.o
	$(CC) -o posix_mq posix_mq.o -lrt -lpthread

posix_mq.o: posix_mq.c
	$(CC) $(CFLAGS) -c posix_mq.c

