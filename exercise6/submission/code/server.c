// Code reference: https://www.educative.io/edpresso/how-to-implement-tcp-sockets-in-c


#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <signal.h>

#define HRES 640
#define VRES 480
#define PIXELS ((HRES) * (VRES))

// #define GRAY
#define RGB
#define FRAME_CNT 1801

int socket_desc, client_sock;

void handler(int signum)
{
	close(client_sock);
	close(socket_desc);
	exit(0);

}

void main(void)
{
	// int socket_desc, client_sock;
	signal(SIGINT, handler);
	struct sockaddr_in server_addr, client_addr; // Internet address type
	char frm_buffer[(PIXELS*3) + 550]; // 550 is for ppm/pgm header
	int read_size, write_size, cnt = 0;
	char fname[30];
	socklen_t addrlen;
	char ack = 'G';

	int fd, recsize = 0, size = 0, tw_size;

	// Create socket:
	socket_desc = socket(AF_INET, SOCK_STREAM, 0);
	if (socket_desc == -1)
	{
		perror("While creating the socket");
		exit(1);
	}

	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(2000); // hton converts to network byte order
	// inet_addr converts dot format to network byte order
	server_addr.sin_addr.s_addr = inet_addr("10.0.0.8"); 

 	if(bind(socket_desc, (struct sockaddr*)&server_addr, sizeof(struct sockaddr_in))<0)
	{
		perror("While binding to the port");
		exit(1);
	}

	// 1 - queue size of pending connections
	if(listen(socket_desc, 1) < 0)
	{
		perror("While listening");
		exit(1);
	}

	client_sock = accept(socket_desc, (struct sockaddr*)&client_addr, &addrlen);
	
	if (client_sock < 0)
	{
		perror("While accepting client");
		exit(1);
	}

	// Use client_sock here on
	printf("Can start receiving from client\n");
	cnt = 0;
	while(1)
	{
		recsize = 0;
		size = 0;
		if (cnt >= FRAME_CNT) break;


				
#ifdef GRAY
		sprintf(fname, "gray_%d.pgm", cnt);
		fd = open(fname, O_RDWR | O_CREAT, 0777);
#endif

#ifdef RGB
		sprintf(fname, "rgb_%d.ppm", cnt);
		fd = open(fname, O_RDWR | O_CREAT, 0777);
#endif
		recv(client_sock, &size, sizeof(int), 0);
		send(client_sock, &ack, sizeof(char), 0);
		printf("size: %d\n", size);
		while (recsize < size)
		{
			do{
				read_size = recv(client_sock, frm_buffer, sizeof(frm_buffer), 0);
				write_size = write(fd, frm_buffer, read_size);
			} while (read_size < 0);
			recsize += read_size;
		}

		send(client_sock, &ack, sizeof(char), 0);

		if (size == 0) break;

		tw_size = lseek(fd, 0 , SEEK_CUR);
		printf("eof: %d\n", tw_size);
		// memset(fname, 0, sizeof(fname));

		// printf("Read bytes : %d\n", read_size);
		

		// 1if (write_size == read_size)
		if (size != tw_size) break;
		if (size == tw_size) printf("%s -- saved\n", fname);
		

		cnt++;
		close(fd);

	}

	close(fd);	
	close(client_sock);
	close(socket_desc);

}
