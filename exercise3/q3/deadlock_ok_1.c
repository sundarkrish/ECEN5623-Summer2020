// Author: Sundar Krishnakumar
// Code reference : http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/code/example-sync-updated-2/
// NOTE : deadlock.c code modified with pthread_mutex_trylock() to  break the deadlock.
//        A random back off algorithm applied (to thread 2) -  question3 requirement
//        Thread 2 repeatedly tries until it gets both mutex B and A.

#include <pthread.h>
#include <stdio.h>
#include <sched.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define NUM_THREADS 2
#define THREAD_1 1
#define THREAD_2 2
#define MAX_SLEEP_CALL 4

typedef struct
{
    int threadIdx;

} threadParams_t;


pthread_t threads[NUM_THREADS];
threadParams_t threadParams[NUM_THREADS];

struct sched_param nrt_param;
struct timespec timeout;
struct timespec rem_time;

// On the Raspberry Pi, the MUTEX semaphores must be statically initialized
//
// This works on all Linux platforms, but dynamic initialization does not work
// on the R-Pi in particular as of June 2020.
//
pthread_mutex_t rsrcA = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t rsrcB = PTHREAD_MUTEX_INITIALIZER;

volatile int rsrcACnt=0, rsrcBCnt=0, noWait=0;


void *grabRsrcs(void *threadp)
{
   threadParams_t *threadParams = (threadParams_t *)threadp;
   int threadIdx = threadParams->threadIdx;
   int rv, complete_flag = 0, b_flag = 1;
   int sleep_cnt = 0;

   if(threadIdx == THREAD_1)
   {
     printf("THREAD 1 grabbing resources\n");
     pthread_mutex_lock(&rsrcA);
     rsrcACnt++;
     if(!noWait) sleep(1);
     printf("THREAD 1 got A, trying for B\n");
     pthread_mutex_lock(&rsrcB);
     rsrcBCnt++;
     printf("THREAD 1 got A and B\n");
     pthread_mutex_unlock(&rsrcB);
     pthread_mutex_unlock(&rsrcA);
     printf("THREAD 1 done\n");
   }
   else
   {
     printf("THREAD 2 grabbing resources\n");
     pthread_mutex_lock(&rsrcB);
     rsrcBCnt++;
     if(!noWait) sleep(1);

     printf("THREAD 2 got B, trying for A\n");
     //pthread_mutex_lock(&rsrcA);

     // Random back off algorithm
     while (complete_flag == 0)
     {
        if (b_flag != 1) // Does not enter the first.Will cause self-deadlock otherwise
        {
          pthread_mutex_lock(&rsrcB);
          rsrcBCnt++;
        } 
     
        rv = pthread_mutex_trylock(&rsrcA);
        if (rv == 0)
        {
          rsrcACnt++;
          printf("THREAD 2 got B and A\n");
          complete_flag = 1;

        }

        if (rv == EBUSY)
        {
          pthread_mutex_unlock(&rsrcB);
          rsrcBCnt--;
          sched_yield();
          b_flag = 0;
          printf("Thread 2 backing off.Will try again.\n");
          sleep_cnt = 0;
          timeout.tv_sec = 0;
          // tv_nsec maximum: 999999999.So using 900000000 as highest with offset of 100 for the random number.
          timeout.tv_nsec = ((rand() % 900000000) + 100 ); // random nanosecond delay
          do
          {
            nanosleep(&timeout, &rem_time);
            timeout.tv_sec = rem_time.tv_sec;
            timeout.tv_nsec = rem_time.tv_nsec;
            sleep_cnt++;

          } while(((rem_time.tv_sec > 0) || (rem_time.tv_nsec > 0)) && (sleep_cnt < MAX_SLEEP_CALL));
          // perror("nanosleep");
        }   

     } 

     
     pthread_mutex_unlock(&rsrcA);
     pthread_mutex_unlock(&rsrcB);
     printf("THREAD 2 done\n");
   }
   pthread_exit(NULL);
}


int main (int argc, char *argv[])
{
   int rc, safe=0;

   rsrcACnt=0, rsrcBCnt=0, noWait=0;


   if(argc == 2)
   {
     if(strncmp("safe", argv[1], 4) == 0)
       safe=1;
     else if(strncmp("race", argv[1], 4) == 0)
       noWait=1;
     else
       printf("Will set up unsafe deadlock scenario\n");
   }
   else
   {
     printf("Usage: deadlock [safe|race|unsafe]\n");
   }


   printf("Creating thread %d\n", THREAD_1);
   threadParams[THREAD_1].threadIdx=THREAD_1;
   rc = pthread_create(&threads[0], NULL, grabRsrcs, (void *)&threadParams[THREAD_1]);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 1 spawned\n");

   if(safe) // Make sure Thread 1 finishes with both resources first
   {
     if(pthread_join(threads[0], NULL) == 0)
       printf("Thread 1: %x done\n", (unsigned int)threads[0]);
     else
       perror("Thread 1");
   }

   printf("Creating thread %d\n", THREAD_2);
   threadParams[THREAD_2].threadIdx=THREAD_2;
   rc = pthread_create(&threads[1], NULL, grabRsrcs, (void *)&threadParams[THREAD_2]);
   if (rc) {printf("ERROR; pthread_create() rc is %d\n", rc); perror(NULL); exit(-1);}
   printf("Thread 2 spawned\n");

   printf("rsrcACnt=%d, rsrcBCnt=%d\n", rsrcACnt, rsrcBCnt);
   printf("will try to join CS threads unless they deadlock\n");

   if(!safe)
   {
     if(pthread_join(threads[0], NULL) == 0)
       printf("Thread 1: %x done\n", (unsigned int)threads[0]);
     else
       perror("Thread 1");
   }

   if(pthread_join(threads[1], NULL) == 0)
     printf("Thread 2: %x done\n", (unsigned int)threads[1]);
   else
     perror("Thread 2");

   printf("rsrcACnt: %d -- rsrcBCnt: %d\n", rsrcACnt, rsrcBCnt);

   if(pthread_mutex_destroy(&rsrcA) != 0)
     perror("mutex A destroy");

   if(pthread_mutex_destroy(&rsrcB) != 0)
     perror("mutex B destroy");

   printf("All done\n");

   exit(0);
}