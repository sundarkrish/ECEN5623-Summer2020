



CC=gcc
CFLAGS=
 
all: deadlock deadlock_ok deadlock_ok_1 pthread3 pthread3amp pthread3ok


clean:
	-rm -f *.o 
	-rm -f deadlock deadlock_ok deadlock_ok_1 pthread3 pthread3amp pthread3ok

deadlock: deadlock.o
	$(CC) -o deadlock deadlock.o -lrt -lpthread

deadlock.o: deadlock.c
	$(CC) $(CFLAGS) -c deadlock.c

deadlock_ok: deadlock_ok.o
	$(CC) -o deadlock_ok deadlock_ok.o -lrt -lpthread

deadlock_ok.o: deadlock_ok.c
	$(CC) $(CFLAGS) -c deadlock_ok.c

deadlock_ok_1: deadlock_ok_1.o
	$(CC) -o deadlock_ok_1 deadlock_ok_1.o -lrt -lpthread

deadlock_ok_1.o: deadlock_ok_1.c
	$(CC) $(CFLAGS) -c deadlock_ok_1.c

pthread3: pthread3.o
	$(CC) -o pthread3 pthread3.o -lrt -lpthread

pthread3.o: pthread3.c
	$(CC) $(CFLAGS) -c pthread3.c

pthread3amp: pthread3amp.o
	$(CC) -o pthread3amp pthread3amp.o -lrt -lpthread

pthread3amp.o: pthread3amp.c
	$(CC) $(CFLAGS) -c pthread3amp.c

pthread3ok: pthread3ok.o
	$(CC) -o pthread3ok pthread3ok.o -lrt -lpthread

pthread3ok.o: pthread3ok.c
	$(CC) $(CFLAGS) -c pthread3ok.c


