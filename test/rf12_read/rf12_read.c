#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

#include "../../rfm12b_ioctl.h"

#define RF12_DEV		"/dev/rfm12.2.1"
#define RF12_BUF_LEN	128

#define MAX_PRINT_BYTES	10

static volatile int running;

void sig_handler(int signum)
{
	signal(signum, SIG_IGN);
	running = 0;
}

void print_stats(int fd)
{
	rfm12b_stats s;
	
	if (0 == ioctl(fd, RFM12B_IOCTL_GET_STATS, &s)) {
		printf("ioctl() succeeded.\n");
	
		printf(
			"\tbytes_recvd: %lu\n"
			"\tpkts_recvd: %lu\n"
			"\tbytes_sent: %lu\n"
			"\tpkts_sent: %lu\n"
			"\tnum_overflows: %lu\n"
			"\tnum_timeouts: %lu\n"
			"\tnum_crc16_fail: %lu\n"
			"\tlow_battery: %u\n",
			s.bytes_recvd, s.pkts_recvd, s.bytes_sent, s.pkts_sent,
			s.num_overflows, s.num_timeouts, s.num_crc16_fail,
			s.low_battery
		);
	
#ifdef RFM12B_IOCTL_DEBUG
		printf(
			"\tin_buf: %p\n"
			"\tin_buf_pos: %p\n"
			"\tout_buf: %p\n"
			"\tout_buf_pos: %p\n"
			"\tin_cur_len_pos: %p\n"
			"\tout_cur_len_pos: %p\n"
			"\tin_cur_end: %p\n"
			"\tout_cur_end: %p\n"
			"\tin_cur_num_bytes: %i\n"
			"\tout_cur_num_bytes: %i\n",
			s.in_buf, s.in_buf_pos, s.out_buf, s.out_buf_pos,
			s.in_cur_len_pos, s.out_cur_len_pos,
			s.in_cur_end, s.out_cur_end,
			s.in_cur_num_bytes, s.out_cur_num_bytes
		);
#endif
	} else
		printf("ioctl() failed.\n");
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
	signal(SIGTERM, sig_handler);
	
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
	
	print_stats(fd);
	
	close(fd);
	
	printf("\n\n%lu packet(s) received.\n\n", pkt_cnt);
	
	return len;
}
