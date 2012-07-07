#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

#define RF12_DEV		"/dev/rfm12.2.1"
#define RF12_BUF_LEN	128

#define MAX_PRINT_BYTES	10

static volatile int running;

void sig_handler(int signum)
{
	running = 0;
}

int main(int argc, char** argv)
{
	int fd, len, i;
	char* devname, buf[128];
	unsigned long pkt_cnt;
	time_t tt;
	
	devname = RF12_DEV;
	
	fd = open(RF12_DEV, O_RDWR);
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
	
	pkt_cnt = 0;
	running = 1;
	do {
		len = read(fd, buf, RF12_BUF_LEN);
		
		time (&tt);
		
		if (len > 0) {
			printf("%s", ctime(&tt));
			printf("\t%i bytes read\n\t\t", len);
			
			for (i=0; i<(len < MAX_PRINT_BYTES ? len : MAX_PRINT_BYTES); i++) {
				printf("%d ", buf[i]);
			}
			printf("\n");
			
			fflush(stdout);	
	
			pkt_cnt++;
		} else if (len < 0) {
			break;
		}
	} while (running);
	
	close(fd);
	
	printf("\n\n%lu packet(s) received.\n\n", pkt_cnt);
	
	return len;
}
