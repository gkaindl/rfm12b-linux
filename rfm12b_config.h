#if !defined(__RFM12B_CONFIG_H__)
#define __RFM12B_CONFIG_H__

/*
  The name of the driver within the kernel (e.g. shows up in logs, etc...)
*/
#define RFM12B_DRV_NAME		"rfm12"

/*
  The textual part of the device name in /dev. E.g., with the default
  setting, the device name for an RFM12b module on SPI bus 2, CS 1 would
  be /dev/rfm12.2.1
*/
#define RFM12_NAME        	"rfm12"

/*
  The RFM12b has a FFOV (rx register overflow) and an RGUR (tx register
  underflow) field in the status word. Those should be set if there was
  an overflow of data while receiving (e.g. you're not reading data fast
  enough) or if you don't supply bits to be sent fast enough, respectively.
  
  However, I'm not sure if I understand those fields correctly, as disregarding
  them seems to improve reliability a lot – I need to investigate this further.
  Thus, you can enable/disable the behavior to regard a send/recv as failed
  when these bits are set in the status register.
  
  Note that even if the "drop packet" behavior is disabled, you are still
  protected against corrupted data by the internal CRC16 checksum.
  
  Tip: Leave this at defaults, unless you're actively investigating this.
*/
#define DROP_PACKET_ON_FFOV		1
#define RETRY_SEND_ON_RGUR		0

#define RFM12B_SPI_MAX_HZ	2500000
#define RFM12B_SPI_MODE		0
#define RFM12B_SPI_BITS		8

#define RFM12_SPI_MAJOR    	154
#define RFM12_N_SPI_MINORS  32

#include "platform/platform.h"
#include "platform/plat_am33xx.h"
#include "platform/plat_spi.h"

#endif // __RFM12B_CONFIG_H__