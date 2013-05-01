#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <stdlib.h>

#include "../common/common.h"
#include "../../rfm12b_config.h"
#include "../../rfm12b_ioctl.h"

#define PACKET_LEN		10
#define SEND_DELAY		1000

static volatile int running;

void sig_handler(int signum)
{
	signal(signum, SIG_IGN);
	running = 0;
}

int main(int argc, char** argv)
{
	int fd, len, i, ppos;
	char* devname, buf[128];
	unsigned long pkt_cnt;
	time_t tt;
	unsigned char* bytes = (unsigned char*)malloc(PACKET_LEN);
	
	devname = RF12_TESTS_DEV;
	
	fd = open(RF12_TESTS_DEV, O_RDWR);
	if (fd < 0) {
		printf("\nfailed to open %s: %s.\n\n", devname, strerror(errno));
		return fd;
	} else
		printf(
			"\nsuccessfully opened %s as fd %i, entering read loop...\n\n",
			devname, fd
		);

	fflush(stdout);
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);
	
	pkt_cnt = 0;
	running = 1;
	ppos = 0;
	do {
		for (i=0; i<PACKET_LEN; i++)
			bytes[i] = (i + ppos) % 255;
		ppos = (ppos + 1) % 255;
		
		len = write(fd, bytes, PACKET_LEN);
		
		time (&tt);
		
		printf("%s", ctime(&tt));
		printf("\t%i bytes written\n\t\t", len);
		
		for (i=0; i<PACKET_LEN; i++) {
			printf("%d ", bytes[i]);
		}
		printf("\n");
		
		fflush(stdout);	
		
		pkt_cnt++;
		
		usleep(SEND_DELAY * 1000);
	} while (running);
	
	print_stats(fd);
	
	close(fd);
	
	free(bytes);
	
	printf("\n\n%lu packet(s) sent.\n\n", pkt_cnt);
	
	return len;
}
