/*
 * Author: Sundar Krishnakumar
 * Brief: Demonstrates the use of thread indexed global variables for thread safe and reentrant code
 *        
 */


#define _GNU_SOURCE
#define MAX_THREADS 2
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/sysinfo.h>


// thread indexed global data
typedef struct
{
    int thread_idx;

} typedef_threadParams;

// thread handler function
void *thread_handler(void *arg)
{
    typedef_threadParams * thread_param = (typedef_threadParams *)arg;

    printf("cpu: %d -- thread: %d \n", sched_getcpu(), thread_param->thread_idx);

}

void main(void)
{
    int rc, i, m_sp, t_sp;
    cpu_set_t tcpu;
    typedef_threadParams thread_params[MAX_THREADS];
    pthread_t thread_id[MAX_THREADS];
    pthread_attr_t thread_attr[MAX_THREADS];
    int rt_max_pri;
    struct sched_param t_sched_param[MAX_THREADS];
    struct sched_param main_sched_param;

    rt_max_pri = sched_get_priority_max(SCHED_FIFO);

    main_sched_param.sched_priority = rt_max_pri;


    if ((rc = sched_setscheduler(getpid(), SCHED_FIFO, &main_sched_param)) != 0)
    {
        perror("Setting main thread schedule policy");
        exit(1);
    }
    else
    {
        m_sp = sched_getscheduler(getpid());
        if (m_sp == SCHED_FIFO)
        {
            printf("main thread-> schedule policy : SCHED_FIFO\n");
        }
        else
        {
            printf("main thread-> schedule policy : OTHER\n");
            exit(1);
        }

    }

    // Select cpu core 3 
    CPU_ZERO(&tcpu);
    CPU_SET(3, &tcpu);

    // Set schedule policy and prioriry for service threads here
    for(i = 0; i < MAX_THREADS; i++)
	{

		if (pthread_attr_init(&thread_attr[i])) { perror("while configuring thread attribute"); exit(1); }
		if (pthread_attr_setinheritsched(&thread_attr[i], PTHREAD_EXPLICIT_SCHED)) { perror("while configuring thread attribute"); exit(1); }
		if (pthread_attr_setschedpolicy(&thread_attr[i], SCHED_FIFO)) { perror("while configuring thread attribute"); exit(1); }
		if (pthread_attr_setaffinity_np(&thread_attr[i], sizeof(cpu_set_t), &tcpu)) { perror("while configuring thread attribute"); exit(1); }

		pthread_attr_getschedpolicy(&thread_attr[i], &t_sp);

		if (t_sp == SCHED_FIFO) printf("Service thread index %d policy is SCHED_FIFO\n", i);
		// Set the priority of the thread here
		t_sched_param[i].sched_priority = rt_max_pri - i;
		rc = pthread_attr_setschedparam(&thread_attr[i], &t_sched_param[i]);
		if (rc != 0)
		{
			perror("Setting the thread priority");
			exit(1);
		}

		thread_params[i].thread_idx = i;
		
	}

    // Spawn the threads here
    pthread_create(&thread_id[0], &thread_attr[0], thread_handler, (void *)&thread_params[0]);
    pthread_create(&thread_id[1], &thread_attr[1], thread_handler, (void *)&thread_params[1]);

    for (i = 0; i < MAX_THREADS; i++)
    {

        pthread_join(thread_id[i], NULL);
        pthread_attr_destroy(&thread_attr[i]);
    }

    printf("Exiting main thread\n");



}

