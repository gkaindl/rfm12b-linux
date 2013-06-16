#if !defined(__RFM12B_IOCTL_H__)
#define __RFM12B_IOCTL_H__
#include <linux/ioctl.h>
#include "rfm12b_config.h"

/*
   Allows you to obtain a structure with statistics about the currently open
   rfm12b board. See the definition of struct rfm12b_stats for details.
   Tracked stats include things like how many bytes and packets have been
   sent and received, how many crc16 checks for received packets failed,
   if the "low battery" status is currently set (probably not all too useful),
   and other driver-internal statistics like rx/tx overflows and underruns.
*/
#define RFM12B_IOCTL_GET_STATS      _IOR(RFM12B_SPI_MAJOR, 0, rfm12b_stats*)

/*
   Get the currently set group identifier for the given board. Only boards
   within the same group can talk to each other.
*/
#define RFM12B_IOCTL_GET_GROUP_ID   _IOR(RFM12B_SPI_MAJOR, 1, int)

/*
   Get the currently set band for the board. The band should match whatever
   your board has been tuned to (check the bottom silkscreen of the rfm12b
   module).
   
   The settings are as follows
      1 ... 433mhz
      2 ... 868mhz
*/
#define RFM12B_IOCTL_GET_BAND_ID    _IOR(RFM12B_SPI_MAJOR, 2, int)

/*
   Get the currently set bitrate for the board as a register value. Boards
   must be set to same bitrate in order to communicate with each other. Lower
   bitrates offer greater range and are more error-tolerant.
   
   You can use the macro RFM12B_BIT_RATE_FROM_BYTE to translate this value
   to a bits-per-second value. 
*/
#define RFM12B_IOCTL_GET_BIT_RATE   _IOR(RFM12B_SPI_MAJOR, 3, int)

#define RFM12B_BIT_RATE_FROM_BYTE(b) (10000000/29/((b&0x7f)+1)/(1+(b&0x80)*7))

/*
   Get the currently set Jee ID for the board. If 0, the driver is not
   in Jee-compatible mode (check the doc for RFM12B_DEFAULT_JEE_ID in
   rfm12b_config.h for more info).
*/
#define RFM12B_IOCTL_GET_JEE_ID     _IOR(RFM12B_SPI_MAJOR, 4, int)

/*
   Get whether automatic sending of ACK packets in Jee-compatible mode
   is active (also see the docs for RFM12B_DEFAULT_JEE_AUTOACK in
   rfm12b_config.h)
*/
#define RFM12B_IOCTL_GET_JEEMODE_AUTOACK  _IOR(RFM12B_SPI_MAJOR, 5, int)

/*
   Set the group identifier for the given board. Only boards within the
   same group can talk to each other.
*/
#define RFM12B_IOCTL_SET_GROUP_ID   _IOW(RFM12B_SPI_MAJOR, 6, int)

/*
   Set the band for the board. The band should match whatever your board
   has been tuned to (check the bottom of the silkscreen of the rfm12b
   module).
   
   The settings are as follows
      1 ... 433mhz
      2 ... 868mhz
*/
#define RFM12B_IOCTL_SET_BAND_ID    _IOW(RFM12B_SPI_MAJOR, 7, int)

/*
   Set the bitrate for the board. Boards must be set to the same bitrate
   to communicate with each other.
   
   Please refer to the datasheet on how to calculate the byte value.
*/
#define RFM12B_IOCTL_SET_BIT_RATE   _IOW(RFM12B_SPI_MAJOR, 8, int)

/*
   Set the Jee ID for the board. If this value is non-zero, then the
   driver is in the Jee-compatible mode. Please check the docs for
   RFM12B_DEFAULT_JEE_ID in rfm12b_config.h for more info on Jee-
   compatible mode.
*/
#define RFM12B_IOCTL_SET_JEE_ID     _IOW(RFM12B_SPI_MAJOR, 9, int)

/*
   Enable or disable (1 or 0) the automatic sending for ACK packets
   in Jee-compatible mode. Check the docs for RFM12B_DEFAULT_JEE_AUTOACK
   in rfm12b_config.h for more information.
*/
#define RFM12B_IOCTL_SET_JEEMODE_AUTOACK  _IOW(RFM12B_SPI_MAJOR, 10, int)

/*
   Structure for driver statistics (see RFM12B_IOCTL_GET_STATS).
   
   All statistics are tracked since the when the device was opened
   by the current user-space process.
*/
typedef struct
{
   unsigned long
      bytes_recvd,         // received bytes count
      bytes_sent,          // sent bytes count
      pkts_recvd,          // received packets count
      pkts_sent,           // sent packets count
      num_recv_overflows,  // FIFO overflow during receiving count
      num_recv_timeouts,   // timeout count during receiving
      num_recv_crc16_fail, // failed crc16 check count during receiving
      num_send_underruns,  // send buffer underrun count
      num_send_timeouts,   // timeout count during sending
      low_battery;         // yes/no if rfm12b Vcc is currently below threshold
} rfm12b_stats;

#endif // __RFM12B_IOCTL_H__