

CC=gcc 
CFLAGS=

all: local global global_1 mutex nav_data


clean:
	-rm -f *.o 
	-rm -f local global global_1 mutex nav_data

local: local.o
	$(CC) -o local local.o -lrt -lpthread

local.o: local.c
	$(CC) $(CFLAGS) -c local.c

global: global.o
	$(CC) -o global global.o -lrt -lpthread

global.o: global.c
	$(CC) $(CFLAGS) -c global.c

global_1: global_1.o
	$(CC) -o global_1 global_1.o -lrt -lpthread

global_1.o: global_1.c
	$(CC) $(CFLAGS) -c global_1.c

mutex: mutex.o
	$(CC) -o mutex mutex.o -lrt -lpthread

mutex.o: mutex.c
	$(CC) $(CFLAGS) -c mutex.c

nav_data: nav_data.o
	$(CC) -o nav_data nav_data.o -lrt -lpthread

nav_data.o: nav_data.c
	$(CC) $(CFLAGS) -c nav_data.c

