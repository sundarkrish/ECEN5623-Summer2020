/*
 * Author: Sundar Krishnakumar
 * Brief: Demonstrates the use of mutex semaphore for wrapping critical sections for thread safe and reentrant code
 *
 */


#define _GNU_SOURCE
#define MAX_THREADS 2
#define COUNT 200000
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <sys/sysinfo.h>

pthread_mutex_t *mtx;

// unsafe global variable
int g;

// thread handler function
void *thread_handler(void *arg)
{
    int cnt;

    for (cnt = 0; cnt < COUNT; cnt++)
    {
        // Start of critical section
        pthread_mutex_lock(mtx);
        g++;
        printf("%d\n", g);
        pthread_mutex_unlock(mtx);
        // End of critical section

    }

}

void main(void)
{
    int rc, i, m_sp, t_sp;
    cpu_set_t tcpu;
    pthread_t thread_id[MAX_THREADS];   
  
  


    mtx = malloc(sizeof(pthread_mutex_t));
    if ((rc = pthread_mutex_init(mtx, NULL)) != 0)
    {
        perror("Mutex init stage");

    }

    // Spawn the threads here
    pthread_create(&thread_id[0], NULL, thread_handler, NULL);
    pthread_create(&thread_id[1], NULL, thread_handler, NULL);

    for (i = 0; i < MAX_THREADS; i++)
    {

        pthread_join(thread_id[i], NULL);
       
    }

    pthread_mutex_destroy(mtx);

    printf("Exiting main thread\n");



}

