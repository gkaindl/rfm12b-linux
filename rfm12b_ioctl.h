#if !defined(__RFM12B_IOCTL_H__)
#define __RFM12B_IOCTL_H__

#include <linux/ioctl.h>

// TODO: only during development!
#define RFM12B_IOCTL_DEBUG

typedef struct
{
	unsigned long bytes_recvd, bytes_sent;
	unsigned long pkts_recvd, pkts_sent;
	unsigned long num_overflows, num_timeouts, num_crc16_fail;
	unsigned char low_battery;
#ifdef RFM12B_IOCTL_DEBUG
	unsigned char* in_buf, *in_buf_pos;
	unsigned char* out_buf, *out_buf_pos;
	unsigned char* in_cur_len_pos, *out_cur_len_pos;
	unsigned char* in_cur_end, *out_cur_end;
	int in_cur_num_bytes, out_cur_num_bytes;
#endif
} rfm12b_stats;

#define RFM12B_IOCTL_GET_STATS		_IOR('r', 1, rfm12b_stats*)

#endif