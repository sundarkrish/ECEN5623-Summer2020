#code reuse: http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/code/RT-Clock/Makefile

INCLUDE_DIRS = 
LIB_DIRS = 
CC=gcc

CDEFS= 
CFLAGS= -O3 -g $(INCLUDE_DIRS) $(CDEFS)
LIBS= -lpthread -lrt

PRODUCT=posix_clock

HFILES=
CFILES= posix_clock.c

SRCS= ${HFILES} ${CFILES}
OBJS= ${CFILES:.c=.o}

all:	${PRODUCT}

clean:
	-rm -f *.o *.NEW *~ *.d
	-rm -f ${PRODUCT} ${GARBAGE}

posix_clock:	posix_clock.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o $@ posix_clock.o $(LIBS)

depend:

.c.o:
	$(CC) -MD $(CFLAGS) -c $<
