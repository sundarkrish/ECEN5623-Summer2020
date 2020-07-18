/*
 * Author: Sundar Krishnakumar
 * Brief: Raw image capture and storage in .ppm format  + image sharpening
 *        YUV to RGB pixel format conversion
 *        Total frames : 30
          Real time image capture using SCHED_FIFO pthreads
 * Code reference : http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
                    http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/sharpen-psf/sharpen.c
                    
 */

#define _GNU_SOURCE  
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdlib.h>
#include <linux/videodev2.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
          
     

#define HRES  640
#define VRES 480
#define PIXELS ((HRES) * (VRES))
#define HRES_STR  "640"
#define VRES_STR  "480"


#define CAMERA "/dev/video0"

#define FRAME_COUNT 30
#define NUM_THREADS 3
#define _GNU_SOURCE
#define K (4.0)
#define NSEC_PER_MSEC (1000000)
#define SEC_TO_MSEC (1000)
#define FPS (5) // max possible fps is 5, frame loss at fps : 6
#define MAX_SLEEP_CALL 3

// all struct forward declarations here


typedef struct
{
	int threadIdx;

} typdef_threadParams;


// all global variables here

int frm_no; // incremented inside sequencer.
int stop_bit;


sem_t semC;
sem_t semS;
sem_t sem;

struct timespec runtime;
struct timespec start;
double time_elapsed;


double PSF[9] = {-K/8.0, -K/8.0, -K/8.0, -K/8.0, K+1.0, -K/8.0, -K/8.0, -K/8.0, -K/8.0};

// Used inside yuv2rgb and sharpen_image
// cannot be local 
char convR[PIXELS];
char convG[PIXELS];
char convB[PIXELS];

char R[PIXELS];
char G[PIXELS];
char B[PIXELS];

// Code reference: http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/RT-Clock/posix_clock.c
// Convert to double precision seconds
double d_ftime(struct timespec *fstart, struct timespec *fstop)
{
  double dfstart = ((double)(fstart->tv_sec) + ((double)(fstart->tv_nsec) / 1000000000.0));
  double dfstop = ((double)(fstop->tv_sec) + ((double)(fstop->tv_nsec) / 1000000000.0)); 

  return (double)(dfstop - dfstart); 
}

// Conversion of yuvu to rgb
// Code source : http://ecee.colorado.edu/~ecen5623/ecen/ex/Linux/computer-vision/simple-capture/capture.c
void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{  
	int r1, g1, b1;

	// replaces floating point coefficients
	int c = y-16, d = u - 128, e = v - 128;       

	// Conversion that avoids floating point
	r1 = (298 * c           + 409 * e + 128) >> 8;
	g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
	b1 = (298 * c + 516 * d           + 128) >> 8;

	// Computed values may need clipping.
	if (r1 > 255) r1 = 255;
	if (g1 > 255) g1 = 255;
	if (b1 > 255) b1 = 255;

	if (r1 < 0) r1 = 0;
	if (g1 < 0) g1 = 0;
	if (b1 < 0) b1 = 0;

	*r = r1 ;
	*g = g1 ;
	*b = b1 ;
}

// ppm header and ppm file name string should be terminated by null.
// strlen used to calc ppm header size
void save_image(char *PPM_fname, char *PPM_header, char *buffer, int buff_size)
{

	int fd1;	

	// Saving the PPM file
	if ((fd1 = open(PPM_fname, O_WRONLY | O_CREAT, 0777)) < 0)
	{

		perror("While creating PPM image file");
		exit(1);

	}


	// printf("%s\n", PPM_header);
	// Write the PPM header
	// IMPORTANT: Never use sizeof.It will print remaining garbage after 255\n in the pixel area.
	// This causes purple tint as pixel is corrupted by ppm header
	write(fd1, PPM_header, strlen(PPM_header)); 
	// printf("header size: %d\n", (int)strlen(PPM_header));
	// Write the colour space : YUYV format
	// char header[] = "P6 640 480 255\n";
	// write(fd1, header, sizeof(header));

	// Write the colour space : RGB format
	write(fd1, buffer, buff_size);
	close(fd1);

}

void process_image(char *buffptr, char *bigbuffer, int size, int frame)
{



	if (buffptr == NULL)
	{
		fprintf(stderr, "Buffptr is NULL\n");
		exit(1);
	}

	// k follows the index of separate rgb channels
	int y1_temp, y2_temp, u_temp, v_temp, i, j, k;

	int bb_size = (size * 6) / 4;

	for(i = 0, j = 0, k = 0; i < size; i = i + 4, j = j + 6, k = k + 2)
	{
		y1_temp = (int)buffptr[i]; 
		u_temp = (int)buffptr[i+1]; 
		y2_temp = (int)buffptr[i+2]; 
		v_temp = (int)buffptr[i+3];
		yuv2rgb(y1_temp, u_temp, v_temp, &bigbuffer[j], &bigbuffer[j+1], &bigbuffer[j+2]);
		yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[j+3], &bigbuffer[j+4], &bigbuffer[j+5]);

		// Split pixel space to RGB channels.
		// For use in sharpen_image service
		R[k] = bigbuffer[j];
		R[k+1] = bigbuffer[j+3];

		G[k] = bigbuffer[j+1];
		G[k+1] = bigbuffer[j+4];

		B[k] = bigbuffer[j+2];
		B[k+1] = bigbuffer[j+5];
	}

	
}

// Do image sharpening here
void *sharpen_image(void *threadp)
{
	int i, j;
	double temp;
	char sharpen_buffer[(PIXELS)*3]; // Stores convolved RGB pixels.

   while (1)
   {
		sem_wait(&semS);
		if (stop_bit == 1) break;
		sem_wait(&sem);
		printf("sharpening: %d\n", frm_no);
	   // Skip first and last row, no neighbors to convolve with
		for(i=1; i<((VRES)-1); i++)
		{

			// Skip first and last column, no neighbors to convolve with
			for(j=1; j<((HRES)-1); j++)
			{
				temp=0;
				temp += (PSF[0] * (double)R[((i-1)*HRES)+j-1]);
				temp += (PSF[1] * (double)R[((i-1)*HRES)+j]);
				temp += (PSF[2] * (double)R[((i-1)*HRES)+j+1]);
				temp += (PSF[3] * (double)R[((i)*HRES)+j-1]);
				temp += (PSF[4] * (double)R[((i)*HRES)+j]);
				temp += (PSF[5] * (double)R[((i)*HRES)+j+1]);
				temp += (PSF[6] * (double)R[((i+1)*HRES)+j-1]);
				temp += (PSF[7] * (double)R[((i+1)*HRES)+j]);
				temp += (PSF[8] * (double)R[((i+1)*HRES)+j+1]);
			if(temp<0.0) temp=0.0;
			if(temp>255.0) temp=255.0;
			convR[(i*HRES)+j]=(char)temp;

				temp=0;
				temp += (PSF[0] * (double)G[((i-1)*HRES)+j-1]);
				temp += (PSF[1] * (double)G[((i-1)*HRES)+j]);
				temp += (PSF[2] * (double)G[((i-1)*HRES)+j+1]);
				temp += (PSF[3] * (double)G[((i)*HRES)+j-1]);
				temp += (PSF[4] * (double)G[((i)*HRES)+j]);
				temp += (PSF[5] * (double)G[((i)*HRES)+j+1]);
				temp += (PSF[6] * (double)G[((i+1)*HRES)+j-1]);
				temp += (PSF[7] * (double)G[((i+1)*HRES)+j]);
				temp += (PSF[8] * (double)G[((i+1)*HRES)+j+1]);
			if(temp<0.0) temp=0.0;
			if(temp>255.0) temp=255.0;
			convG[(i*HRES)+j]=(char)temp;

				temp=0;
				temp += (PSF[0] * (double)B[((i-1)*HRES)+j-1]);
				temp += (PSF[1] * (double)B[((i-1)*HRES)+j]);
				temp += (PSF[2] * (double)B[((i-1)*HRES)+j+1]);
				temp += (PSF[3] * (double)B[((i)*HRES)+j-1]);
				temp += (PSF[4] * (double)B[((i)*HRES)+j]);
				temp += (PSF[5] * (double)B[((i)*HRES)+j+1]);
				temp += (PSF[6] * (double)B[((i+1)*HRES)+j-1]);
				temp += (PSF[7] * (double)B[((i+1)*HRES)+j]);
				temp += (PSF[8] * (double)B[((i+1)*HRES)+j+1]);
			if(temp<0.0) temp=0.0;
			if(temp>255.0) temp=255.0;
			convB[(i*HRES)+j]=(char)temp;
			}

		}	

	

		// Save the image in disk

		char PPM_fname[18];
		// memset(PPM_fname, 0, sizeof(PPM_fname));
		sprintf(PPM_fname, "sharp_%d.ppm", frm_no);
		// printf("%d\n", strlen(PPM_fname));

		char PPM_header[60];
		// char timestamp[25];
		// memset(timestamp, 0, sizeof(timestamp));
		memset(PPM_header, 0 , sizeof(PPM_header)); // IMPORTANT. Else will cause stack smash.strcat cannot find null char.		
		//M.S.nsec format
		// sprintf(timestamp, "#%d.%d.%d\n", (timestamp_struct.tv_sec/60), (timestamp_struct.tv_sec%60), (timestamp_struct.tv_nsec));
		// Constructing PPM header
		strcat(PPM_header, "P6\n");
		// strcat(PPM_header, timestamp);
		strcat(PPM_header, HRES_STR);
		strcat(PPM_header, " ");
		strcat(PPM_header, VRES_STR);
		strcat(PPM_header, "\n");
		strcat(PPM_header, "255\n");



		// IMPORTANT: Doing byte by byte write() takes a lot of time.So replaced with buffer and a single write().
		for (i = 0, j = 0; i < PIXELS; i++, j = j + 3)
		{
			sharpen_buffer[j] = convR[i];
			sharpen_buffer[j+1] = convG[i];
			sharpen_buffer[j+2] = convB[i];		

		}


		save_image(PPM_fname, PPM_header ,sharpen_buffer, sizeof(sharpen_buffer));
	
		printf("Sharpening complete : %d\n", frm_no);
		sem_post(&sem);
	}

	pthread_exit(NULL);


}

// image capture happens here
void *capture_image(void *threadp)
{

	int fd, i, j, buff_index;
	struct v4l2_format format;
	struct v4l2_requestbuffers buff_rq;
	struct v4l2_buffer buff_info;
	void *buffer = NULL; // holds the pointers to mmap.They point to raw image location
	int type; // holds value of buff_info.type  Here: V4L2_BUF_TYPE_VIDEO_CAPTURE

	char PPM_fname[10];
	// memset(PPM_fname, 0, sizeof(PPM_fname));
	
	// printf("%d\n", strlen(PPM_fname));

	char PPM_header[60];
	char timestamp[25];

	struct timespec timestamp_struct; 

	if ((fd = open(CAMERA, O_RDWR)) < 0)
	{
		perror("Opening camera file");
		exit(1);

	}

	// Set image format here
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	// Support available.Checked with v4l2-ctl -d /dev/video0 --list-formats-ext
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV; 
	// format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG; 
	format.fmt.pix.width = HRES;
	format.fmt.pix.height = VRES;

	if (ioctl(fd, VIDIOC_S_FMT, &format) < 0)
	{
		perror("VIDIOC_S_FMT");
		exit(1);
	}

	// Request buffers here
	buff_rq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buff_rq.memory = V4L2_MEMORY_MMAP;
	buff_rq.count = 1;

	if (ioctl(fd, VIDIOC_REQBUFS, &buff_rq) < 0)
	{
		perror("VIDIOC_REQBUFS");
		exit(1);
	}

   // printf("Number of buffers allocated: %d\n", buff_rq.count);



	// To get length and offset for doing mmap and retreiving pointer
	memset(&buff_info, 0, sizeof(buff_info)); // Protection against garbage values

	buff_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buff_info.memory = V4L2_MEMORY_MMAP;
	buff_info.index = 0;

	if (ioctl(fd, VIDIOC_QUERYBUF, &buff_info) < 0)
	{
		perror("VIDIOC_QUERYBUF");
		exit(1);
	}

	buffer = mmap(NULL, buff_info.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buff_info.m.offset);
	if ((buffer == (void *)-1) || (buffer == MAP_FAILED))
	{
		perror("While doing mmap");
		exit(1);

	}

    
	// Stores result of Conversion of YUVU422 to RGB.Created only once here.
	int bb_size = ((buff_info.length * 6) / 4);	
	char bigbuffer[bb_size];   

    
   type = buff_info.type;

   // Start streaming
   if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
   {
      perror("VIDIOC_STREAMON");
      exit(1);
    
	}

   // Start continuous capture
   // frm_no = 0;
   while (1)
   {		

		sem_wait(&semC);
		if (stop_bit == 1) break;
		sem_wait(&sem);
		printf("capturing: %d\n", frm_no);


		// memset(buffer, 0, sizeof(buff_info));
		memset(&buff_info, 0, sizeof(buff_info));


		buff_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buff_info.memory = V4L2_MEMORY_MMAP;
		buff_info.index = 0;

		// Put the  buffer to incoming queue 
		if (ioctl(fd, VIDIOC_QBUF, &buff_info) < 0)
		{
				perror("VIDIOC_QBUF");
				exit(1);
		}

		memset(&buff_info, 0, sizeof(buff_info));
		buff_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buff_info.memory = V4L2_MEMORY_MMAP;


		// Release the buffer from the outgoing queue of the driver.
		// IMPORTANT: Blocking call.Will block the calling thread.
		if (ioctl(fd, VIDIOC_DQBUF, &buff_info) < 0)
		{
			perror("VIDIOC_DQBUF");
			exit(1);
		}


			
		// IMPORTANT: image timestamp here			
		clock_gettime(CLOCK_REALTIME, &timestamp_struct);
		// CONVERT TO RGB
		// SAVE AS .ppm
		process_image(buffer, bigbuffer, buff_info.length, frm_no);

		sprintf(PPM_fname, "%d.ppm", frm_no);

		memset(timestamp, 0, sizeof(timestamp));
		memset(PPM_header, 0 , sizeof(PPM_header)); // IMPORTANT. Else will cause stack smash.strcat cannot find null char.
		
		//M.S.nsec format - Absolute time
		sprintf(timestamp, "#%ld.%d.%ld\n", (timestamp_struct.tv_sec/60), (int)(timestamp_struct.tv_sec%60), (timestamp_struct.tv_nsec));
		// Constructing PPM header
		strcat(PPM_header, "P6\n");
		strcat(PPM_header, timestamp);
		strcat(PPM_header, HRES_STR);
		strcat(PPM_header, " ");
		strcat(PPM_header, VRES_STR);
		strcat(PPM_header, "\n");
		strcat(PPM_header, "255\n");

		save_image(PPM_fname, PPM_header, bigbuffer, sizeof(bigbuffer));

      printf("capturing  complete : %d\n", frm_no);
		sem_post(&sem);
   }



   // Stop streaming
   if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0)
   {
      perror("VIDIOC_STREAMOFF");
      exit(1);
   }


    
   // printf("%d\n", buff_info.length);
   // printf("%d\n", buff_info.bytesused);

	//do munmap here
	munmap(buffer, buff_info.length);

   close(fd);
	pthread_exit(NULL);



}



void *sequencer(void *threadp)
{
	frm_no = 0;
	stop_bit = 0; // stop frame capture if stop_bit = 1 continue if 0
	int sleep_cnt;
	struct timespec seq_timeout; // for nanosleep
	long int timeout_calc = (int)((1.0/FPS)*(SEC_TO_MSEC)*(NSEC_PER_MSEC));
	struct timespec rem_time; // for nanosleep
	printf("Sequncer timeout period(deadline) : %ldns\n", timeout_calc);

	float avg_fps = 0;

	clock_gettime(CLOCK_REALTIME, &start);
	clock_gettime(CLOCK_REALTIME, &runtime);
	time_elapsed = d_ftime(&start, &runtime);
	printf("------Sequencer start. Time elapsed: %4.2fs------\n", time_elapsed);

	while (frm_no < FRAME_COUNT)
	{	

		sem_post(&semC);
		sem_post(&semS);
		seq_timeout.tv_sec = (timeout_calc /1000000000);
		seq_timeout.tv_nsec = (timeout_calc % 1000000000);
		// seq_timeout.tv_sec = 7;
		// seq_timeout.tv_nsec = 0;
		// printf("sequencer: %d\n", frm_no);

		sleep_cnt = 0;
		do
		{
			nanosleep(&seq_timeout, &rem_time);
			seq_timeout.tv_sec = rem_time.tv_sec;
			seq_timeout.tv_nsec = rem_time.tv_nsec;
			sleep_cnt++;

		} while (((rem_time.tv_sec > 0) || (rem_time.tv_nsec > 0)) && (sleep_cnt < MAX_SLEEP_CALL));

		printf("------Processed frame number: %d-------\n", frm_no);
		frm_no++;

		clock_gettime(CLOCK_REALTIME, &runtime);
		time_elapsed = d_ftime(&start, &runtime);
		printf("------Sequencer wake up at %4.2fs------\n", time_elapsed);
	}



	printf("Total frames processed: %d\n", frm_no);
	printf("Elapsed time : %lfs\n", time_elapsed);
	avg_fps = (frm_no / time_elapsed);

	printf("Average frame rate : %4.4f fps\n", avg_fps);
	printf("Configured frame rate : %d fps\n", FPS);


	stop_bit = 1;
	sem_post(&semC);
	sem_post(&semS);
	pthread_exit(NULL);

}



int main (int argc, char *argv[])
{
   int rc, policy;
   int i, result = 0;
   cpu_set_t tcpu;

   typdef_threadParams threadParams[NUM_THREADS];
	pthread_t threads[NUM_THREADS];

	pthread_attr_t rt_sched_attr[NUM_THREADS];
   int rt_max_prio, rt_min_prio;
   struct sched_param rt_sched_param[NUM_THREADS];
   struct sched_param main_param;
  

	rt_max_prio = sched_get_priority_max(SCHED_FIFO);
	// rc = sched_getparam(getpid(), &main_param);
	main_param.sched_priority = rt_max_prio;

	// Set the scheduling polic of the main thread to SCHED_FIFO
	// priority = 99 i.e. highest priority
	if (sched_setscheduler(getpid(), SCHED_FIFO, &main_param)) { perror("Main thread set scheduler"); exit(1);};
	
	policy = sched_getscheduler(getpid());
	if (policy == SCHED_FIFO) printf("Main thread policy is SCHED_FIFO\n");

	// Settting the priority for the tasks
	for(i = 0; i < NUM_THREADS; i++)
	{
		
		// Select cpu core 3 
		CPU_ZERO(&tcpu);
		CPU_SET(3, &tcpu);


		if (pthread_attr_init(&rt_sched_attr[i])) { perror("while configuring thread attribute"); exit(1); }
		if (pthread_attr_setinheritsched(&rt_sched_attr[i], PTHREAD_EXPLICIT_SCHED)) { perror("while configuring thread attribute"); exit(1); }
		if (pthread_attr_setschedpolicy(&rt_sched_attr[i], SCHED_FIFO)) { perror("while configuring thread attribute"); exit(1); }
		if (pthread_attr_setaffinity_np(&rt_sched_attr[i], sizeof(cpu_set_t), &tcpu)) { perror("while configuring thread attribute"); exit(1); }
		pthread_attr_getschedpolicy(&rt_sched_attr[i], &policy);

		if (policy == SCHED_FIFO) printf("Service thread index %d policy is SCHED_FIFO\n", i);

		// Set the priority of the thread here
		rt_sched_param[i].sched_priority = rt_max_prio - i;
		rc = pthread_attr_setschedparam(&rt_sched_attr[i], &rt_sched_param[i]);
		if (rc != 0)
		{
			perror("While Setting thread priority");
			exit(1);
		}

		threadParams[i].threadIdx = i;
		
	}

	// init the semaphore here
	// start value 0.Shared amonh threads.
	if (sem_init(&semC, 0, 0)) {perror("While semakey init"); exit(1);}
	if (sem_init(&semS, 0, 0)) {perror("While semakey init"); exit(1);}
	if (sem_init(&sem, 0, 1)) {perror("While semakey init"); exit(1);} // Command for 2 service threads below.Not used in sequencer.

   // spawn image capture and image sharpen service thread here
  
	pthread_create(&threads[1],   
						&rt_sched_attr[1],     
						capture_image, 
						&threadParams[1] 
					);

	pthread_create(&threads[2],   
						&rt_sched_attr[2],     
						sharpen_image, 
						&threadParams[2] 
					);

	sleep(5); // Let the above 2 threads start and try to take the semakey.
	// launch the sequencer thread here
	pthread_create(&threads[0],   
					&rt_sched_attr[0],     
					sequencer, 
					&threadParams[0] 
				);

   for(i=0; i < NUM_THREADS; i++)
   {
      pthread_join(threads[i], NULL);   

   }
       


	pthread_attr_destroy(&rt_sched_attr[0]);
	pthread_attr_destroy(&rt_sched_attr[1]);
	pthread_attr_destroy(&rt_sched_attr[2]);

	sem_destroy(&semC);
	sem_destroy(&semS);
	sem_destroy(&sem);

   printf("EXITING\n");
   exit(0);
}