



CC=gcc
CFLAGS=
 
all: capture_ppm capture_ppm_sl


clean:
	-rm -f *.o *.ppm  
	-rm -f capture_ppm capture_ppm_sl

capture_ppm: capture_ppm.o
	$(CC) -o capture_ppm capture_ppm.o -lrt -lpthread

capture_ppm.o: capture_ppm.c
	$(CC) $(CFLAGS) -c capture_ppm.c

capture_ppm_sl: capture_ppm_sl.o
	$(CC) -o capture_ppm_sl capture_ppm_sl.o -lrt -lpthread

capture_ppm_sl.o: capture_ppm_sl.c
	$(CC) $(CFLAGS) -c capture_ppm_sl.c



