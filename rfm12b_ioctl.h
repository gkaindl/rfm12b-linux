#if !defined(__RFM12B_IOCTL_H__)
#define __RFM12B_IOCTL_H__

#include <linux/ioctl.h>

#include "rfm12b_config.h"

typedef struct
{
	unsigned long bytes_recvd, bytes_sent;
	unsigned long pkts_recvd, pkts_sent;
	unsigned long num_recv_overflows, num_recv_timeouts, num_recv_crc16_fail;
	unsigned long num_send_underruns, num_send_timeouts;
	unsigned char low_battery;
} rfm12b_stats;

#define RFM12B_IOCTL_GET_STATS		_IOR(RFM12B_SPI_MAJOR, 0, rfm12b_stats*)
#define RFM12B_GET_GROUP_ID			_IOR(RFM12B_SPI_MAJOR, 1, int)
#define RFM12B_GET_BAND_ID			_IOR(RFM12B_SPI_MAJOR, 2, int)
#define RFM12B_GET_BIT_RATE			_IOR(RFM12B_SPI_MAJOR, 3, int)
#define RFM12B_GET_JEE_ID			_IOR(RFM12B_SPI_MAJOR, 4, int)
#define RFM12B_SET_GROUP_ID			_IOW(RFM12B_SPI_MAJOR, 5, int)
#define RFM12B_SET_BAND_ID			_IOW(RFM12B_SPI_MAJOR, 6, int)
#define RFM12B_SET_BIT_RATE			_IOW(RFM12B_SPI_MAJOR, 7, int)
#define RFM12B_SET_JEE_ID			_IOR(RFM12B_SPI_MAJOR, 8, int)

#endif // __RFM12B_IOCTL_H__