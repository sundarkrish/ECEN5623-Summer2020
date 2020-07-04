/*
 * Author: Sundar Krishnakumar
 * Brief: Demonstrates the posix_mq.c VxWorks code
 *        message is shared between threads using POSIX message queue.
 *        Delete my_queue in /dev/mqueue/ folder before running if you don't unlink
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

char message[] = "Houston we have a problem";

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
    typedef_threadParams * thread_param = (typedef_threadParams *)arg;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = MAX_MSG_SIZE;
    // Created in /dev/mqueue/ folder
    mqd = mq_open(MESSAGE_Q, O_CREAT | O_RDWR, 0777, &attr);
    perror("Message queue creation");
 
    char buffer[MAX_MSG_SIZE];
   
    int msg_prio;
    int n_bytes = 0;
    // receive data here
    n_bytes = mq_receive(mqd, buffer, (size_t)(MAX_MSG_SIZE), &msg_prio); 
    perror("Message receive");

    printf("cpu: %d -- receiver thread -- message: %s -- priority: %d -- received bytes: %d\n", 
        sched_getcpu(), (char *)buffer, msg_prio, n_bytes);
    sched_yield();
}

// sender thread.Thread index 1
void *sender(void *arg)
{

    int msg_prio = 30;
    // send the data here
    mq_send(mqd, message, sizeof(message), msg_prio);
    
    perror("message send");
    // printf("cpu: %d -- thread: %d --  \n", sched_getcpu(), thread_param->thread_idx);
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
    
    printf("size of message: %d\n", sizeof(message));

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

    thread_params[1].buffptr = (void *)malloc(sizeof(void *) + sizeof(int));
    thread_params[1].buffptr = message;

    // Spawn the threads here
    pthread_create(&thread_id[0], &thread_attr[0], receiver, NULL);
    pthread_create(&thread_id[1], &thread_attr[1], sender, NULL);

    for (i = 0; i < MAX_THREADS; i++)
    {

        pthread_join(thread_id[i], NULL);
        pthread_attr_destroy(&thread_attr[i]);
    }

    mq_unlink(MESSAGE_Q);
    printf("Exiting main thread\n");



}
