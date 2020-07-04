/*
 * Author: Sundar Krishnakumar
 * Brief: Demonstrates the heap_mq.c VxWorks code
 *        Pointer to a heap is shared between threads using POSIX message queue.
 *        Delete my_queue in /dev/mqueue/ folder before running if you don't unlink.
 */


#define _GNU_SOURCE
#define MAX_THREADS 2
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/sysinfo.h>
#include <mqueue.h>
#include <string.h>

#define MESSAGE_Q "/my_queue"
#define MAX_MSG_SIZE 50

mqd_t mqd; // message queue descriptor

// struct mq_attr {
//     long mq_flags;       /* Flags: 0 or O_NONBLOCK */
//     long mq_maxmsg;      /* Max. # of messages on queue */
//     long mq_msgsize;     /* Max. message size (bytes) */
//     long mq_curmsgs;     /* # of messages currently in queue */
// };
struct mq_attr attr = {0}; // message queue attribute data-structure.

// thread indexed global data
typedef struct
{
    int thread_idx;
    void *buffptr;

} typedef_threadParams;

// receiver thread - high priority than sender.Thread index 0
void *receiver(void *arg)
{
    // typedef_threadParams * thread_param = (typedef_threadParams *)arg;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(void *) + sizeof(int);
    // Created in /dev/mqueue/ folder
    mqd = mq_open(MESSAGE_Q, O_CREAT | O_RDWR, 0777, &attr);
    perror("Message queue creation");
    void *buffptr;
    char buffer[sizeof(void *) + sizeof(int)];
    int id;
    int msg_prio;
    int n_bytes = 0;
    // receive data here
    n_bytes = mq_receive(mqd, buffer, (size_t)(sizeof(void *) + sizeof(int)), &msg_prio); 
    perror("Message receive");
    memcpy(&buffptr, buffer, sizeof(void *));
    memcpy(&id, &buffer[sizeof(void *)], sizeof(int));
    printf("cpu: %d -- receiver thread -- location: 0x%X -- message: %s -- id: %d -- priority: %d -- received bytes: %d\n", 
        sched_getcpu(), buffptr, (char *)buffptr, id, msg_prio, n_bytes);

    free(buffptr); // free the heap memory allocated.The pointer sent from sender thread.
    sched_yield();
}

// sender thread.Thread index 1
void *sender(void *arg)
{
    int id = 999;
    char buffer[sizeof(void *) + sizeof(int)];
    typedef_threadParams * thread_param = (typedef_threadParams *)arg;
    void * buffptr = thread_param->buffptr;
    memcpy(buffer, &buffptr, sizeof(void *));
    memcpy(&buffer[sizeof(void *)], &id, sizeof(int));
    int msg_prio = 30;
    printf("cpu: %d -- sender thread -- location: 0x%X\n", sched_getcpu(), buffptr);
    // send the data here
    mq_send(mqd, buffer, (size_t)(sizeof(void *) + sizeof(int)), msg_prio);
    perror("message send");
    
    sched_yield();

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

    char * message = (char *)malloc(MAX_MSG_SIZE);
    strcpy(message, "Houston we have a problem");

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

    thread_params[1].buffptr = (void *)message;

    // Spawn the threads here
    pthread_create(&thread_id[0], &thread_attr[0], receiver, NULL);
    pthread_create(&thread_id[1], &thread_attr[1], sender, (void *)&thread_params[1]);

    for (i = 0; i < MAX_THREADS; i++)
    {

        pthread_join(thread_id[i], NULL);
        pthread_attr_destroy(&thread_attr[i]);
    }

    mq_unlink(MESSAGE_Q);
    printf("Exiting main thread\n");



}
