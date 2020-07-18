/*
 * Author: Sundar Krishnakumar
 * Brief: Raw image capture and storage in .ppm format
 *        YUV to RGB pixel format conversion
 *        Total frames : 30
 * Code reference : http://jwhsmith.net/2014/12/capturing-a-webcam-stream-using-v4l2/
 */


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

#define HRES  640
#define VRES 480
#define HRES_STR  "640"
#define VRES_STR  "480"


#define CAMERA "/dev/video0"
#define BUFF_CNT 6
#define FRAME_COUNT 30

// all struct forward declarations here



// all global variables here
struct v4l2_format format;
struct v4l2_requestbuffers buff_rq;
struct v4l2_buffer buff_info;
void *buffer[BUFF_CNT]; // holds the pointers to mmap.They point to raw image location
int type; // holds value of buff_info.type  Here: V4L2_BUF_TYPE_VIDEO_CAPTURE



struct timespec runtime;
struct timespec start;

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

int process_image(char *buffptr, char *bigbuffer, int size, int frame, int index)
{

    char PPM_fname[10];
    // memset(PPM_fname, 0, sizeof(PPM_fname));
    sprintf(PPM_fname, "%d.ppm", frame);
    // printf("%d\n", strlen(PPM_fname));

    if (buffptr == NULL)
    {
        fprintf(stderr, "Buffptr at index : %d is NULL\n", index);
        exit(1);
    }

    int y1_temp, y2_temp, u_temp, v_temp, fd1, i, j;

    int bb_size = (size * 6) / 4;
    
    clock_gettime(CLOCK_REALTIME, &start);
   

    for(i = 0, j = 0; i < size; i = i + 4, j = j + 6)
    {
        y1_temp = (int)buffptr[i]; 
        u_temp = (int)buffptr[i+1]; 
        y2_temp = (int)buffptr[i+2]; 
        v_temp = (int)buffptr[i+3];
        yuv2rgb(y1_temp, u_temp, v_temp, &bigbuffer[j], &bigbuffer[j+1], &bigbuffer[j+2]);
        yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[j+3], &bigbuffer[j+4], &bigbuffer[j+5]);
    }

    clock_gettime(CLOCK_REALTIME, &runtime);
    printf("start time : %lf\n", (start.tv_sec/1.0) + (start.tv_nsec / 1000000000.0));
    printf("end time : %lf\n", (runtime.tv_sec/1.0) + (runtime.tv_nsec / 1000000000.0));

    runtime.tv_sec -= start.tv_sec;
    runtime.tv_nsec -= start.tv_nsec;
    
    printf("elapsed time : %lf -- frame: %d\n", ((runtime.tv_sec/1.0) + (runtime.tv_nsec / 1000000000.0)), frame);

    // Saving the PPM file
    if ((fd1 = open(PPM_fname, O_WRONLY | O_CREAT, 0777)) < 0)
    {

        perror("While creating PPM image file");
        exit(1);

    }

    char PPM_header[25];
    memset(PPM_header, 0 , sizeof(PPM_header)); // IMPORTANT. Else will cause stack smash.strcat cannot find endof string
    // Constructing PPM header
    strcat(PPM_header, "P6\n");
    strcat(PPM_header, HRES_STR);
    strcat(PPM_header, " ");
    strcat(PPM_header, VRES_STR);
    strcat(PPM_header, "\n");
    strcat(PPM_header, "255\n");

    // printf("%s\n", PPM_header);
    // Write the PPM header
    // IMPORTANT: Never use sizeof.It will print remaining garbage after 255\n in the pixel area.
    // This causes purple tint as pixel is corrupted by ppm header
    write(fd1, PPM_header, strlen(PPM_header)); 
    // Write the colour space : YUYV format
    // char header[] = "P6 640 480 255\n";
    // write(fd1, header, sizeof(header));

    // Write the colour space : RGB format
    write(fd1, (char *)bigbuffer, bb_size);
    close(fd1);

    return 1;
}

void main(void)
{

    int fd, fd1, i, j, buff_index;
    int frm_no;

    for (i = 0; i < BUFF_CNT; i++)
    {

        buffer[i] = NULL;

    }

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
    buff_rq.count = BUFF_CNT;

    if (ioctl(fd, VIDIOC_REQBUFS, &buff_rq) < 0)
    {
        perror("VIDIOC_REQBUFS");
        exit(1);
    }

    printf("Number of buffers allocated: %d\n", buff_rq.count);

    for (i = 0;i < BUFF_CNT; i++)
    {

        // To get length and offset for doing mmap and retreiving pointer
        memset(&buff_info, 0, sizeof(buff_info)); // Protection against garbage values

        buff_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buff_info.memory = V4L2_MEMORY_MMAP;
        buff_info.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buff_info) < 0)
        {
            perror("VIDIOC_QUERYBUF");
            exit(1);
        }

        buffer[i] = mmap(NULL, buff_info.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buff_info.m.offset);
        if ((buffer == (void *)-1) || (buffer == MAP_FAILED))
        {
            perror("While doing mmap");
            exit(1);

        }

    }

    
    // Stores result of Conversion of YUVU422 to RGB.Created only once here.
    int bb_size = ((buff_info.length * 6) / 4);
    char bigbuffer[bb_size];

// Queue the 6 buffers in the incoming queue of camera driver
    for (i = 0; i < BUFF_CNT; i++)
    {

        // memset(buffer, 0, sizeof(buff_info));
        memset(&buff_info, 0, sizeof(buff_info));
        

        buff_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buff_info.memory = V4L2_MEMORY_MMAP;
        buff_info.index = i;


        // Release the buffer from the outgoing queue of the driver
        if (ioctl(fd, VIDIOC_QBUF, &buff_info) < 0)
        {
            perror("VIDIOC_QBUF");
            exit(1);
        }

    }

    
    type = buff_info.type;

    // Start streaming
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0)
    {
        perror("VIDIOC_STREAMON");
        exit(1);
    }


    // Start continuous capture
    frm_no = 0;
    while (frm_no < FRAME_COUNT)
    {

        memset(&buff_info, 0, sizeof(buff_info));
        buff_info.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buff_info.memory = V4L2_MEMORY_MMAP;


        // Release the buffer from the outgoing queue of the driver.(Blocking call)
        if (ioctl(fd, VIDIOC_DQBUF, &buff_info) < 0)
        {
            perror("VIDIOC_DQBUF");
            exit(1);
        }

        buff_index = buff_info.index;    
        // printf("Processing image at buffer : %d\n", buff_index);
        if ((buff_index >= 0) && (buff_index < BUFF_CNT))
        {
           
            
            // CONVERT TO RGB
            // SAVE AS .ppm
            process_image((char *)buffer[buff_index], bigbuffer, buff_info.length, frm_no, buff_index);

            // Put the currently used camera buffer to incoming queue again
            if (ioctl(fd, VIDIOC_QBUF, &buff_info) < 0)
            {
                perror("VIDIOC_QBUF");
                exit(1);
            }

        }
        else
        {

            fprintf( stderr, "Improper buffer index: %d\n", buff_index);
            exit(1);
        }

        frm_no++;

        
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
	for (buff_index = 0; buff_index < BUFF_CNT; buff_index++)
	{

		munmap(buffer[buff_index], buff_info.length);

	}

    close(fd);



}
