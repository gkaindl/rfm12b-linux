#if !defined(__RFM12B_IOCTL_H__)
#define __RFM12B_IOCTL_H__

#include <linux/ioctl.h>

typedef struct
{
	unsigned long bytes_recvd, bytes_sent;
	unsigned long pkts_recvd, pkts_sent;
	unsigned long num_recv_overflows, num_recv_timeouts, num_recv_crc16_fail;
	unsigned long num_send_underruns, num_send_timeouts;
	unsigned char low_battery;
} rfm12b_stats;

#define RFM12B_IOCTL_GET_STATS		_IOR('r', 1, rfm12b_stats*)

#endif // __RFM12B_IOCTL_H__