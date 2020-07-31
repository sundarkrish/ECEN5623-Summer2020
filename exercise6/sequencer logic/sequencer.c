#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <semaphore.h>
#include <syslog.h>


#define FRAME_COUNT 100

#define _GNU_SOURCE

#define NSEC_PER_MSEC (1000000)
#define SEC_TO_MSEC (1000)
#define SEQ_FREQ 100
#define CAP_FREQ_MOD 10 // 10 for 10Hz


#define MAX_SLEEP_CALL 3

sem_t semC, semS;

double d_ftime(struct timespec *fstart, struct timespec *fstop)
{
  double dfstart = ((double)(fstart->tv_sec) + ((double)(fstart->tv_nsec) / 1000000000.0));
  double dfstop = ((double)(fstop->tv_sec) + ((double)(fstop->tv_nsec) / 1000000000.0)); 

  return (double)(dfstop - dfstart); 
}

void main(void)
{

	int frm_no = 0;
	long int seq_cnt = 0;
	int sleep_cnt;
	
	struct timespec runtime; 
	struct timespec start; 
	double time_elapsed; 
	double wcet_sequencer = 0.0;

	struct timespec begin; // for services
	struct timespec end; // for services

	struct timespec seq_timeout; // for nanosleep
	long int timeout_calc = (int)((1.0/SEQ_FREQ)*(SEC_TO_MSEC)*(NSEC_PER_MSEC));
	long int timeout;
	struct timespec rem_time; // for nanosleep
	syslog(LOG_INFO,"Sequencer timeout period : %ldns\n", timeout_calc);


	float avg_fps = 0;
	openlog("Question4", LOG_CONS | LOG_NDELAY, LOG_LOCAL7);

	if (sem_init(&semC, 0, 0)) {perror("While semakey init"); exit(1);}
	if (sem_init(&semS, 0, 0)) {perror("While semakey init"); exit(1);}

	// gun shot method
	syslog(LOG_INFO,"Press any key to start\n");
	getchar();
	clock_gettime(CLOCK_REALTIME, &start);
	clock_gettime(CLOCK_REALTIME, &runtime);
	time_elapsed = d_ftime(&start, &runtime);
	syslog(LOG_INFO,"------Sequencer start. Time elapsed: %4.2fs------\n", time_elapsed);

	// Critical instant.Starting at 0.00s
	while (frm_no < FRAME_COUNT)
	{	

		// if ((seq_cnt % CAP_FREQ_MOD) == 0)
		// {
		// 	sem_post(&semC);
		
		// }
			

		// if (((seq_cnt % CAP_FREQ_MOD) == 0) && (seq_cnt >= SEQ_FREQ))
		// {
		// 	sem_post(&semS);
			
		// }

		
		
		// sem_post(&semG);
		// sem_post(&semB);
// #ifdef LOG
// 		sem_post(&semL);
// #endif
		// seq_timeout.tv_sec = (timeout_calc /1000000000);
		// seq_timeout.tv_nsec = (timeout_calc % 1000000000);

		// seq_timeout.tv_sec = 0;
		// seq_timeout.tv_nsec = 9825000;
		
		clock_gettime(CLOCK_REALTIME, &seq_timeout);
		timeout = ((seq_timeout.tv_sec * 1000000000) + seq_timeout.tv_nsec) + timeout_calc;
		seq_timeout.tv_sec = (timeout /1000000000);
		seq_timeout.tv_nsec = (timeout % 1000000000);
		clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &seq_timeout, NULL);
		// usleep(10000); // 10ms

		// sleep_cnt = 0;
		// do
		// {
		// 	nanosleep(&seq_timeout, &rem_time);
		// 	seq_timeout.tv_sec = rem_time.tv_sec;
		// 	seq_timeout.tv_nsec = rem_time.tv_nsec;
		// 	sleep_cnt++;

		// } while (((rem_time.tv_sec > 0) || (rem_time.tv_nsec > 0)) && (sleep_cnt < MAX_SLEEP_CALL));
		clock_gettime(CLOCK_REALTIME, &runtime);
		time_elapsed = d_ftime(&start, &runtime);
		syslog(LOG_INFO,"------Sequencer wake up at %4.4fs------\n", time_elapsed);
		// syslog(LOG_INFO,"------Processed frame number: %d-------\n", frm_no);
		syslog(LOG_INFO,"------Sequence count: %ld-------\n", seq_cnt);
		frm_no++;
		seq_cnt++;



	}



	syslog(LOG_INFO,"WCET sequencer: %4.10f", wcet_sequencer);
}