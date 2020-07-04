/*
 * Author: Sundar Krishnakumar
 * Brief: Demonstrates the use of mutex semaphore for wrapping critical sections for thread safe and reentrant code
 *        Using real time SCHED_RR policy and both threads same priority for simulating data race scenario.
 *
 *        
 */


#define _GNU_SOURCE
#define MAX_THREADS 2
#define COUNT 100
#define ITERATION_CNT 20000
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/sysinfo.h>

pthread_mutex_t *mtx;

double r_num;

// This function is a cycle burner.Which makes writer thread to take some noticeable time to
// populate the navdata
double myRand()
{

    int i;
    for(i = 0; i < ITERATION_CNT; i++)
    {
        r_num = rand();
        r_num += (double)(rand() / 100000.0);

    }

    return r_num;


}

// thread indexed global data
typedef struct
{
    int thread_idx;

} typedef_threadParams;


typedef struct
{
    double x, y, z;
    double roll, pitch, yaw;
    struct timespec ts;

} typdef_navdata;

// Navigation data - global unsafe
typdef_navdata navdata = {0};

// write thread handler function
void *thread_handler_1(void *arg)
{
    int cnt;
   

    for (cnt = 0; cnt < COUNT; cnt++)
    {
        
     
        // Thread 0 -> Update the values here
        pthread_mutex_lock(mtx);
        printf("Write attempt: %d\n", cnt);
        navdata.x = myRand();
        navdata.y = myRand();
        navdata.z = myRand();
        navdata.roll = myRand();
        navdata.pitch = myRand();
        navdata.yaw = myRand();
        clock_gettime(CLOCK_REALTIME, &navdata.ts);
        printf("Entered values\n");
        printf("Timestamp: %lf\n", (double)(navdata.ts.tv_sec + navdata.ts.tv_nsec / 1000000000.0));
        printf("X: %lf -- Y: %lf -- Z: %lf\n", navdata.x, navdata.y, navdata.z);
        printf("Roll: %lf -- Pitch: %lf -- Yaw: %lf\n", navdata.roll, navdata.pitch, navdata.yaw);
        printf("Entry complete\n\n");            
        pthread_mutex_unlock(mtx);
        sched_yield();        


    }

}

// read thread handler function
void *thread_handler_2(void *arg)
{
    int cnt;
  

    for (cnt = 0; cnt < COUNT; cnt++)
    {
           
            
        // Thread 1 -> Read the values here
        pthread_mutex_lock(mtx);
        printf("Read attempt: %d\n", cnt); 
        printf("Reading values\n");
        printf("Timestamp: %lf\n", (double)(navdata.ts.tv_sec + navdata.ts.tv_nsec / 1000000000.0));
        printf("X: %lf -- Y: %lf -- Z: %lf\n", navdata.x, navdata.y, navdata.z);
        printf("Roll: %lf -- Pitch: %lf -- Yaw: %lf\n", navdata.roll, navdata.pitch, navdata.yaw);
        printf("Reading complete\n\n");            
        pthread_mutex_unlock(mtx);
        sched_yield();

        

    }

}

void main(void)
{
    int rc, i, m_sp, t_sp;
    cpu_set_t tcpu;
    pthread_t thread_id[MAX_THREADS]; 
    int max_pri;
    pthread_attr_t thread_attr[MAX_THREADS];  
    struct sched_param t_sched_param[MAX_THREADS];

    max_pri = sched_get_priority_max(SCHED_FIFO);

    // Select cpu core 3 
    CPU_ZERO(&tcpu);
    CPU_SET(3, &tcpu);

    // Set schedule policy and prioriry for service threads here
    for(i = 0; i < MAX_THREADS; i++)
	{
		if (pthread_attr_init(&thread_attr[i])) { perror("while configuring thread attribute"); exit(1); }
        if (pthread_attr_setinheritsched(&thread_attr[i], PTHREAD_EXPLICIT_SCHED)) { perror("while configuring thread attribute"); exit(1); }
		if (pthread_attr_setschedpolicy(&thread_attr[i], SCHED_RR)) { perror("while configuring thread attribute"); exit(1); }
        if (pthread_attr_setaffinity_np(&thread_attr[i], sizeof(cpu_set_t), &tcpu)) { perror("while configuring thread attribute"); exit(1); }

        // both threads same priority
		t_sched_param[i].sched_priority = max_pri;
		rc = pthread_attr_setschedparam(&thread_attr[i], &t_sched_param[i]);
		if (rc != 0)
		{
			perror("Setting the thread priority");
			exit(1);
		}

		

    }

    mtx = malloc(sizeof(pthread_mutex_t));
    if ((rc = pthread_mutex_init(mtx, NULL)) != 0)
    {
        perror("Mutex init stage");
        exit(1);

    }

    // Spawn the threads here
    pthread_create(&thread_id[0], &thread_attr[0], thread_handler_1, NULL);
    perror("thread creation1");
    pthread_create(&thread_id[1], &thread_attr[1], thread_handler_2, NULL);
    perror("thread creation2");

    for (i = 0; i < MAX_THREADS; i++)
    {

        pthread_join(thread_id[i], NULL);
       
    }

    pthread_mutex_destroy(mtx);

    printf("Exiting main thread\n");



}

