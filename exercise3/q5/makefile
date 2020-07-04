



CC=gcc
CFLAGS=

all: nav_data_timeout


clean:
	-rm -f *.o 
	-rm -f nav_data_timeout

nav_data_timeout: nav_data_timeout.o
	$(CC) -o nav_data_timeout nav_data_timeout.o -lrt -lpthread

simple_thread.o: simple_thread.c
	$(CC) $(CFLAGS) -c nav_data_timeout.c




