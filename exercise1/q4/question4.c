// Author: Sundar Krishnakumar
// Brief: Question4 RTES Exercise 1
// Code reference & reuse: http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/code/sequencer/lab1.c


// Needed for using CPU affinity macros
#define _GNU_SOURCE


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/sysinfo.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <semaphore.h>
#include <syslog.h>
#define USEC_PER_MSEC (1000)
#define FIB_TEST_CYCLES (100)
#define FIB_LIMIT (10)
#define NUM_THREADS (3)     // service threads + sequencer 

unsigned int seqIterations = FIB_LIMIT;
sem_t semf10, semf20; // sema keys for fib10 and fib20
double start_time;
int abortTest = 0;

// NOTE: These are global variables used by 
// non reentrant functions fib10 fib20
unsigned int idx = 0, jdx = 1;
unsigned int fib = 0, fib0 = 0, fib1 = 1;


typedef struct
{
    int threadIdx;
    int MajorPeriods;
} threadParams_t;

// Gives epoch time in milliseconds
double getTimeMsec(void)
{
  struct timespec event_ts = {0, 0};

  clock_gettime(CLOCK_MONOTONIC, &event_ts);
  //clock_gettime(CLOCK_REALTIME, &event_ts);
  return ((event_ts.tv_sec)*1000.0) + ((event_ts.tv_nsec)/1000000.0);
}

// Fibonacci series MACRO function here
#define FIB_TEST(seqCnt, iterCnt)      \
   for(idx=0; idx < iterCnt; idx++)    \
   {                                   \
   	fib0=0; fib1=1; jdx=1;           \
      fib = fib0 + fib1;               \
      while(jdx < seqCnt)              \
      {                                \
         fib0 = fib1;                  \
         fib1 = fib;                   \
         fib = fib0 + fib1;            \
         jdx++;                        \
      }                                \
   } 


void *fib10(void *threadp)
{

	struct sched_param param;
	int policy;

   double event_time, run_time=0.0;
   int limit=0, release=0, cpucore, i;
   threadParams_t *threadParams = (threadParams_t *)threadp;
   unsigned int required_test_cycles;

   // Assume FIB_TEST short enough that preemption risk is minimal
   //
   FIB_TEST(seqIterations, FIB_TEST_CYCLES); //warm cache
   event_time=getTimeMsec();
   FIB_TEST(seqIterations, FIB_TEST_CYCLES);
   run_time=getTimeMsec() - event_time;

   required_test_cycles = (int)(10.0/run_time);
   syslog(LOG_INFO, "F10 runtime calibration %lf msec per %d test cycles, so %u required\n", run_time, FIB_TEST_CYCLES, required_test_cycles);

	pthread_getschedparam(pthread_self(), &policy, &param);
	syslog(LOG_INFO, "Priority: %d\n", param.sched_priority);
	if (policy == SCHED_FIFO) syslog(LOG_INFO, "policy : SCHED_FIFO\n");

   while(!abortTest)
   {
       sem_wait(&semf10); 

       if(abortTest)
           break; 
       else 
           release++;

       cpucore=sched_getcpu();
       syslog(LOG_INFO, "F10 start %d @ %lf on core %d\n", release, (event_time=getTimeMsec() - start_time), cpucore);

       do
       {
           FIB_TEST(seqIterations, FIB_TEST_CYCLES);
           limit++;
       }
       while(limit < required_test_cycles);

       syslog(LOG_INFO, "F10 complete %d @ %lf, %d loops\n", release, (event_time=getTimeMsec() - start_time), limit);
       limit=0;
   }

   pthread_exit((void *)0);
}

void *fib20(void *threadp)
{
	struct sched_param param;
	int policy;


   double event_time, run_time=0.0;
   int limit=0, release=0, cpucore, i;
   threadParams_t *threadParams = (threadParams_t *)threadp;
   int required_test_cycles;

   // Assume FIB_TEST short enough that preemption risk is minimal
   //
   FIB_TEST(seqIterations, FIB_TEST_CYCLES); //warm cache
   event_time=getTimeMsec();
   FIB_TEST(seqIterations, FIB_TEST_CYCLES);
   run_time=getTimeMsec() - event_time;

   required_test_cycles = (int)(20.0/run_time);
   syslog(LOG_INFO, "F20 runtime calibration %lf msec per %d test cycles, so %d required\n", run_time, FIB_TEST_CYCLES, required_test_cycles);

	pthread_getschedparam(pthread_self(), &policy, &param);
	syslog(LOG_INFO, "Priority: %d\n", param.sched_priority);
	if (policy == SCHED_FIFO) syslog(LOG_INFO, "policy : SCHED_FIFO\n");

   while(!abortTest)
   {
        sem_wait(&semf20);

        if(abortTest)
           break; 
        else 
           release++;

        cpucore=sched_getcpu();
        syslog(LOG_INFO, "F20 start %d @ %lf on core %d\n", release, (event_time=getTimeMsec() - start_time), cpucore);

        do
        {
            FIB_TEST(seqIterations, FIB_TEST_CYCLES);
            limit++;
        }
        while(limit < required_test_cycles);

        syslog(LOG_INFO, "F20 complete %d @ %lf, %d loops\n", release, (event_time=getTimeMsec() - start_time), limit);
        limit=0;
   }

   pthread_exit((void *)0);
}

// Prints scheduler policy of the main thread
void print_scheduler(void)
{
   int schedType;

   schedType = sched_getscheduler(getpid());

   switch(schedType)
   {
     case SCHED_FIFO:
           syslog(LOG_INFO, "Pthread Policy is SCHED_FIFO\n");
           break;
     case SCHED_OTHER:
           syslog(LOG_INFO, "Pthread Policy is SCHED_OTHER\n"); exit(-1);
       break;
     case SCHED_RR:
           syslog(LOG_INFO, "Pthread Policy is SCHED_RR\n"); exit(-1);
           break;
     default:
       syslog(LOG_INFO, "Pthread Policy is UNKNOWN\n"); exit(-1);
   }

}



void *Sequencer(void *threadp)
{
   
	int i;
	int MajorPeriodCnt=0;
	double event_time;
	threadParams_t *threadParams = (threadParams_t *)threadp;

	syslog(LOG_INFO, "Starting Sequencer: [S1, T1=20, C1=10], [S2, T2=50, C2=20], U=0.9, LCM=100\n");
	start_time=getTimeMsec();


	// Sequencing loop for LCM phasing of S1, S2
	do
	{

		// Basic sequence of releases after CI for 90% load
		//
		// S1: T1= 20, C1=10 msec 
		// S2: T2= 50, C2=20 msec
		//
		

		// Simulate the C.I. for S1 and S2 and timestamp in log
		syslog(LOG_INFO, "\n**** CI t=%lf\n", event_time=getTimeMsec() - start_time);
		sem_post(&semf10); sem_post(&semf20);
		
		usleep(20*USEC_PER_MSEC); sem_post(&semf10);
		syslog(LOG_INFO, "t=%lf\n", event_time=getTimeMsec() - start_time);
		
		usleep(20*USEC_PER_MSEC); sem_post(&semf10);
		syslog(LOG_INFO, "t=%lf\n", event_time=getTimeMsec() - start_time);

		usleep(10*USEC_PER_MSEC); sem_post(&semf20);
		syslog(LOG_INFO, "t=%lf\n", event_time=getTimeMsec() - start_time);

		usleep(10*USEC_PER_MSEC); sem_post(&semf10);
		syslog(LOG_INFO, "t=%lf\n", event_time=getTimeMsec() - start_time);

		usleep(20*USEC_PER_MSEC); sem_post(&semf10);
		syslog(LOG_INFO, "t=%lf\n", event_time=getTimeMsec() - start_time);

		usleep(20*USEC_PER_MSEC);

		MajorPeriodCnt++;
	} 
	while (MajorPeriodCnt < threadParams->MajorPeriods);

	abortTest=1;

	// Now release the sema keys for all services so that 
	// they exit.If they do not exit main thread will be
	// stuck at thread_join
	sem_post(&semf10); sem_post(&semf20);

	
}

void main(void)
{

   int i, rc, policy;
   cpu_set_t threadcpu;
   pthread_t threads[NUM_THREADS];
   threadParams_t threadParams[NUM_THREADS];
   pthread_attr_t rt_sched_attr[NUM_THREADS];
   int rt_max_prio, rt_min_prio;
   struct sched_param rt_sched_param[NUM_THREADS];
   struct sched_param main_param;
   pthread_attr_t main_attr;

	abortTest=0;

	openlog("Question4", LOG_CONS | LOG_NDELAY, LOG_LOCAL7);
	// syslog(LOG_NOTICE, "Program started by User %d", getuid ());

   syslog(LOG_INFO, "System has %d processors configured and %d available.\n", get_nprocs_conf(), get_nprocs());

	// Initialize the sema keys
	// If pshared has the value 0, then the semaphore is shared 
	// between the threads of a process.
	// Initial value of key = 0 i.e EMPTY
   if (sem_init(&semf10, 0, 0)) { syslog(LOG_INFO, "Failed to initialize semF10 semaphore\n"); exit(-1); }
   if (sem_init(&semf20, 0, 0)) { syslog(LOG_INFO, "Failed to initialize semF20 semaphore\n"); exit(-1); }

	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	// rc = sched_getparam(getpid(), &main_param);
	main_param.sched_priority = rt_max_prio;

	// Set the scheduling polic of the main thread to SCHED_FIFO
	// priority = 99 i.e. highest priority
	rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_param);
	if (rc != 0)
	{
		perror("schedule policy->main thread");
		exit(-1);
	}

	syslog(LOG_INFO, "schedule policy->main thread: ");
	print_scheduler();
	
	for(i = 0; i < NUM_THREADS; i++)
	{
		
		
		CPU_ZERO(&threadcpu);		
		CPU_SET(1, &threadcpu);

		if (pthread_attr_init(&rt_sched_attr[i])) { perror("while confifuring thread attribute"); exit(-1); }
		if (pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED)) { perror("while confifuring thread attribute"); exit(-1); }
		if (pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO)) { perror("while configuring thread attribute"); exit(-1); }
		if (pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &threadcpu)) { perror("while confifuring thread attribute"); exit(-1); }

		pthread_attr_getschedpolicy(&rt_sched_attr[i], &policy);

		if (policy == SCHED_FIFO) syslog(LOG_INFO, "Service thread index %d policy is SCHED_FIFO\n", i);
		// Set the priority of the thread here
		rt_sched_param[i].sched_priority = rt_max_prio - i;
		rc = pthread_attr_setschedparam(&rt_sched_attr[i], &rt_sched_param[i]);
		if (rc != 0)
		{
			perror("Setting the thread priority");
			exit(-1);
		}

		threadParams[i].threadIdx = i;
		
	}

	// Create service threads here
	// Service thread fib10
	rc = pthread_create(&threads[1], 
							  &rt_sched_attr[1],
							  fib10,
							  (void *)&threadParams[1]
							 );

	if (rc != 0)
	{
		perror("At pthread creation:");
		exit(-1);
	}

	// Service thread fib20
	rc = pthread_create(&threads[2], 
							  &rt_sched_attr[2],
							  fib20,
							  (void *)&threadParams[2]
							 );	

	if (rc != 0)
	{
		perror("At pthread creation:");
		exit(-1);
	}


	// Wait for threads fib10 anf fib20 to calibrate and await relese by sequencer
   usleep(300000);

	// 3 iterations of 100ms period
	// 100ms because LCM=100 for the schedule table
	
	threadParams[0].MajorPeriods = 3;

	// Sequencer thread  - highest priority
	rc = pthread_create(&threads[0], 
							  &rt_sched_attr[0],
							  Sequencer,
							  (void *)&threadParams[0]
							 );	

	if (rc != 0)
	{
		perror("At pthread creation:");
		exit(-1);
	}



	for(i=0;i<NUM_THREADS;i++)
	{
		pthread_join(threads[i], NULL);
		pthread_attr_destroy(&rt_sched_attr[i]);
	}

	sem_destroy(&semf10);
	sem_destroy(&semf20);

	closelog ();

	syslog(LOG_INFO, "*****COMPLETE*****\n");

}


