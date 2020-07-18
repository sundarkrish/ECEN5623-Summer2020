/*
 * Author: Sundar Krishnakumar
 * Brief: Raw image capture and storage in .ppm format + b/w conversion + 
 *        grayscale conversion + image sharpening
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
          
     

#define HRES 160
#define VRES 120
#define PIXELS ((HRES) * (VRES))
#define HRES_STR  "160"
#define VRES_STR  "120"

#define LOG // Define to log metrics.Define before NUM_THREADS macro.

#define CAMERA "/dev/video0"

#define FRAME_COUNT 30

#ifdef LOG
#define NUM_THREADS 6
#endif

#ifndef LOG
#define NUM_THREADS 5
#endif

#define _GNU_SOURCE
#define K (4.0)
#define NSEC_PER_MSEC (1000000)
#define SEC_TO_MSEC (1000)
#define FPS (10) 
#define MAX_SLEEP_CALL 3
#define MAXVAL (255)
#define BIN_THRESH (0.5)



// all struct forward declarations here
typedef struct
{
	int threadIdx;

} typdef_threadParams;


// all global variables here

int frm_no; // incremented inside sequencer.Used in all service threads
int stop_bit; // toggled inside sequencer


sem_t semC;
sem_t semS;
sem_t sem; //  for nesting service thread's C.S because DQ_BUF ioctl call is blocking call.
sem_t semG;
sem_t semB;
sem_t semL;


double PSF[9] = {-K/8.0, -K/8.0, -K/8.0, -K/8.0, K+1.0, -K/8.0, -K/8.0, -K/8.0, -K/8.0};

// Used inside yuv2rgb and sharpen_image
// cannot be local 
char convR[PIXELS];
char convG[PIXELS];
char convB[PIXELS];

char R[PIXELS];
char G[PIXELS];
char B[PIXELS];

char gray[PIXELS]; // buffer for grayscale image

// Metrics variables

double et_capture;
double et_sharpen;
double et_gray;
double et_bw;
double et_process;


double wcet_capture = 0; // max et
double wcet_sharpen = 0 ; // max et
double wcet_gray = 0; // max et
double wcet_bw = 0; // max et
double wcet_process = 0; // max et
double deadline; // total execution > deadline => deadline_miss = YES 




struct timespec begin; // for services
struct timespec end; // for services
struct timespec process_begin;
struct timespec process_end;
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

// #ifdef LOG
// 	clock_gettime(CLOCK_REALTIME, &process_begin);
// #endif

	for(i = 0, j = 0, k = 0; i < size; i = i + 4, j = j + 6, k = k + 2)
	{
		gray[k] = y1_temp = (int)buffptr[i];  // luminance component
		u_temp = (int)buffptr[i+1]; 
		gray[k+1] = y2_temp = (int)buffptr[i+2]; // luminance component 
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

// #ifdef LOG
// 	clock_gettime(CLOCK_REALTIME, &process_end);
// 	et_process = d_ftime(&process_begin, &process_end);
// 	if (et_process > wcet_process) wcet_process = et_process;
// #endif

	
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
#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &begin);
#endif
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

#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &end);
		et_sharpen = d_ftime(&begin, &end);
		// et_sharpen += et_process;
		if (et_sharpen > wcet_sharpen) wcet_sharpen = et_sharpen;
#endif
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
#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &begin);
#endif
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
#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &end);
		et_capture = d_ftime(&begin, &end);
		if (et_capture > wcet_capture) wcet_capture = et_capture;
#endif
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

	// Do munmap here
	munmap(buffer, buff_info.length);

   close(fd);

	pthread_exit(NULL);

}


void *convert2gray(void *threadp)
{

	int i;

   while (1)
   {
		sem_wait(&semG);
		if (stop_bit == 1) break;
		sem_wait(&sem);
		printf("Converting to gray: %d\n", frm_no);

#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &begin);
#endif
		
		// Save the image in disk
		char PGM_fname[18];
		// memset(PPM_fname, 0, sizeof(PPM_fname));
		sprintf(PGM_fname, "gray_%d.pgm", frm_no);
		// printf("%d\n", strlen(PPM_fname));

		char PGM_header[60];
		// char timestamp[25];
		// memset(timestamp, 0, sizeof(timestamp));
		memset(PGM_header, 0 , sizeof(PGM_header)); // IMPORTANT. Else will cause stack smash.strcat cannot find null char.		
		//M.S.nsec format
		// sprintf(timestamp, "#%d.%d.%d\n", (timestamp_struct.tv_sec/60), (timestamp_struct.tv_sec%60), (timestamp_struct.tv_nsec));
		// Constructing PPM header
		strcat(PGM_header, "P5\n");
		// strcat(PGM_header, timestamp);
		strcat(PGM_header, HRES_STR);
		strcat(PGM_header, " ");
		strcat(PGM_header, VRES_STR);
		strcat(PGM_header, "\n");
		strcat(PGM_header, "255\n");




		save_image(PGM_fname, PGM_header ,gray, sizeof(gray));
#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &end);
		et_gray = d_ftime(&begin, &end);
		// et_gray += et_process;
		if (et_gray > wcet_gray) wcet_gray = et_gray;
#endif
		printf("Conversion to gray complete : %d\n", frm_no);
		sem_post(&sem);
	}

	pthread_exit(NULL);

}

// Algorithm based on reference : https://www.mathworks.com/help/images/ref/im2bw.html
void *convert2bin(void *threadp)
{

	int i;
	char bin[PIXELS];

   while (1)
   {
		sem_wait(&semB);
		if (stop_bit == 1) break;
		sem_wait(&sem);
		printf("Converting to binary: %d\n", frm_no);

#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &begin);
#endif

		for (i = 0;i < PIXELS; i++)
		{

			bin[i] = gray[i] > ((BIN_THRESH)*(MAXVAL)) ? 1 : 0;

		}
		
		// Save the image in disk
		char PGM_fname[18];
		// memset(PPM_fname, 0, sizeof(PPM_fname));
		sprintf(PGM_fname, "bin_%d.pgm", frm_no);
		// printf("%d\n", strlen(PPM_fname));

		char PGM_header[60];
		// char timestamp[25];
		// memset(timestamp, 0, sizeof(timestamp));
		memset(PGM_header, 0 , sizeof(PGM_header)); // IMPORTANT. Else will cause stack smash.strcat cannot find null char.		
		//M.S.nsec format
		// sprintf(timestamp, "#%d.%d.%d\n", (timestamp_struct.tv_sec/60), (timestamp_struct.tv_sec%60), (timestamp_struct.tv_nsec));
		// Constructing PPM header
		strcat(PGM_header, "P5\n");
		// strcat(PGM_header, timestamp);
		strcat(PGM_header, HRES_STR);
		strcat(PGM_header, " ");
		strcat(PGM_header, VRES_STR);
		strcat(PGM_header, "\n");
		strcat(PGM_header, "1\n");

		save_image(PGM_fname, PGM_header ,bin, sizeof(bin));
#ifdef LOG
		clock_gettime(CLOCK_REALTIME, &end);
		et_bw = d_ftime(&begin, &end);
		// et_bw += et_process;
		if (et_bw > wcet_bw) wcet_bw = et_bw;
#endif
		printf("Conversion to binary complete : %d\n", frm_no);
		sem_post(&sem);
	}

	pthread_exit(NULL);
	
}

// Code source: https://stackoverflow.com/questions/10936733/how-to-write-to-a-second-column-in-excel-in-c
//              https://www.tutorialspoint.com/c_standard_library/c_function_perror.htm
void *logger(void *threadp)
{
	FILE * fp2 = NULL;
	int deadline_miss;
	double jitter; // frame capture exec time  - deadline per frame
	double exec_jitter; // total execution time - deadline per frame

	fp2 = fopen("wcet.csv", "w");

	if(fp2 == NULL){
		perror("Opening csv file");
		exit(1);
	}
	
	fprintf(fp2, "%s,%d,%d", "RESOLUTION", HRES, VRES);
	fprintf(fp2, "\n");
   //  Dont print deadline here.Deadline calculated inside sequencer.Not avaialable at this point.
	fprintf(fp2, "%s,%s,%s,%s,%s,%s,%s\n", "et_capture", "et_sharpen", "et_gray", "et_binary", 
		"deadline_miss", "jitter in the frame", "execution jitter");

   while (1)
   {
		sem_wait(&semL);
		if (stop_bit == 1) break;
		sem_wait(&sem);

		deadline_miss = (et_capture + et_sharpen + et_gray + et_bw) > deadline;
		jitter = et_capture - deadline; // jitter in the frame relative to deadline
		exec_jitter = (et_capture + et_sharpen + et_gray + et_bw) - deadline;
		fprintf(fp2, "%lf,%lf,%lf,%lf,%d,%lf,%lf\n", et_capture, et_sharpen, et_gray, et_bw, 
			deadline_miss, jitter, exec_jitter);

		sem_post(&sem);

	}

	fprintf(fp2, "\n");
	fprintf(fp2, "%s,%s,%s,%s\n", "wcet_capture", "wcet_sharpen", "wcet_gray", "wcet_binary");
	fprintf(fp2, "%lf,%lf,%lf,%lf\n", wcet_capture, wcet_sharpen, wcet_gray, wcet_bw);
	fprintf(fp2, "\n");
	fprintf(fp2, "%s,%lf", "DEADLINE", deadline); // relative deadline
	

	fclose(fp2);

}

void *sequencer(void *threadp)
{
	frm_no = 0;
	stop_bit = 0; // stop frame capture if stop_bit = 1 continue if 0
	int sleep_cnt;

	struct timespec runtime; 
	struct timespec start; 
	double time_elapsed; 

	struct timespec seq_timeout; // for nanosleep
	long int timeout_calc = (int)((1.0/FPS)*(SEC_TO_MSEC)*(NSEC_PER_MSEC));
	struct timespec rem_time; // for nanosleep
	printf("Sequencer timeout period : %ldns\n", timeout_calc);

	deadline = (timeout_calc / 1000000000.0); // nanosec to seconds
	float avg_fps = 0;

	clock_gettime(CLOCK_REALTIME, &start);
	clock_gettime(CLOCK_REALTIME, &runtime);
	time_elapsed = d_ftime(&start, &runtime);
	printf("------Sequencer start. Time elapsed: %lfs------\n", time_elapsed);



	while (frm_no < FRAME_COUNT)
	{	

		sem_post(&semC);
		sem_post(&semS);
		sem_post(&semG);
		sem_post(&semB);
#ifdef LOG
		sem_post(&semL);
#endif
		seq_timeout.tv_sec = (timeout_calc /1000000000);
		seq_timeout.tv_nsec = (timeout_calc % 1000000000);
		
		// seq_timeout.tv_sec = 7;
		// seq_timeout.tv_nsec = 0;
		// printf("sequencer: %d\n", frm_no);

		sleep_cnt = 0;
		do
		{
			nanosleep(&seq_timeout, &rem_time);
			clock_gettime(CLOCK_REALTIME, &runtime);
			seq_timeout.tv_sec = rem_time.tv_sec;
			seq_timeout.tv_nsec = rem_time.tv_nsec;
			sleep_cnt++;

		} while (((rem_time.tv_sec > 0) || (rem_time.tv_nsec > 0)) && (sleep_cnt < MAX_SLEEP_CALL));
		
		
		time_elapsed = d_ftime(&start, &runtime);
		printf("------Processed frame number: %d-------\n", frm_no);
		frm_no++;
		printf("------Sequencer wake up at %lfs------\n", time_elapsed);
	}



	printf("Total frames processed: %d\n", frm_no);
	printf("Elapsed time : %lfs\n", time_elapsed);
	avg_fps = (frm_no / time_elapsed);

	printf("Average frame rate : %4.4f fps\n", avg_fps);
	printf("Configured frame rate : %d fps\n", FPS);
	printf("Configured resolution : %d x %d\n", HRES, VRES);



	stop_bit = 1;
	sem_post(&semC);
	sem_post(&semS);
	sem_post(&semG);
	sem_post(&semB);
#ifdef LOG	
	sem_post(&semL);
#endif	
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
	if (sem_init(&semG, 0, 0)) {perror("While semakey init"); exit(1);}
	if (sem_init(&semB, 0, 0)) {perror("While semakey init"); exit(1);}
#ifdef LOG	
	if (sem_init(&semL, 0, 0)) {perror("While semakey init"); exit(1);}
#endif	

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

	pthread_create(&threads[3],   
					&rt_sched_attr[3],     
					convert2gray, 
					&threadParams[3] 
				);

	pthread_create(&threads[4],   
					&rt_sched_attr[4],     
					convert2bin, 
					&threadParams[4] 
				);

#ifdef LOG

	pthread_create(&threads[5],   
				&rt_sched_attr[5],     
				logger, 
				&threadParams[5] 
			);

#endif

	sleep(7); // Let the above 2 threads start and try to take the semakey.
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
	pthread_attr_destroy(&rt_sched_attr[3]);
	pthread_attr_destroy(&rt_sched_attr[4]);
#ifdef LOG	
	pthread_attr_destroy(&rt_sched_attr[5]);
#endif

	sem_destroy(&semC);
	sem_destroy(&semS);
	sem_destroy(&sem);
	sem_destroy(&semG);
	sem_destroy(&semB);
#ifdef LOG	
	sem_destroy(&semL);
#endif	

   printf("EXITING\n");
   exit(0);
}