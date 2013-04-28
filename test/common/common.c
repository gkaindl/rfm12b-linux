#include <fcntl.h>
#include <stdio.h>

#include "common.h"
#include "../../rfm12b_ioctl.h"

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
			"\tnum_send_underruns: %lu\n"
			"\tnum_send_timeouts: %lu\n"
			"\tlow_battery: %u\n",
			s.bytes_recvd, s.pkts_recvd, s.bytes_sent, s.pkts_sent,
			s.num_recv_overflows, s.num_recv_timeouts, s.num_recv_crc16_fail,
			s.num_send_underruns, s.num_send_timeouts,
			s.low_battery
		);
	} else
		printf("ioctl() failed.\n");
}
