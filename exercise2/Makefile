# -*- Makefile -*-

CC=gcc -O0
LIBS= -lm

all: feasibility_tests m_feasibility_tests


clean:
	-rm -f *.o 
	-rm -f feasibility_tests m_feasibility_tests

feasibility_tests: feasibility_tests.o
	$(CC) -o feasibility_tests feasibility_tests.o $(LIBS)

feasibility_tests.o: feasibility_tests.c
	$(CC) -c feasibility_tests.c

m_feasibility_tests: m_feasibility_tests.o
	$(CC) -o m_feasibility_tests m_feasibility_tests.o $(LIBS) 

m_feasibility_tests.o: m_feasibility_tests.c
	$(CC) -c m_feasibility_tests.c