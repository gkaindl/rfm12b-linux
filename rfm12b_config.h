#if !defined(__RFM12B_CONFIG_H__)
#define __RFM12B_CONFIG_H__

/*
  The plaform you are building for. This is important, change it to
  whatever board you are using. You can further edit hardware-specific
  configuration in the following files.
  
  BOARD             NUMBER          HARDWARE-SPECIFICS
  
  Raspberri Pi      1               platform/plat_raspberrypi.h
  Beaglebone        2               platform/plat_beaglebone.h
*/
#define RFM12B_BOARD        1

/*
  The name of the driver within the kernel (e.g. shows up in logs, etc...)
*/
#define RFM12B_DRV_NAME     "rfm12b"

/*
  The textual part of the device name in /dev. E.g., with the default
  setting, the device name for an RFM12b module on SPI bus 2, CS 1 would
  be /dev/rfm12.2.1
*/
#define RFM12B_DEV_NAME     "rfm12b"

/*
  The default group ID to use for each RFM12B board. You can change the
  group ID for a particular board via ioctl() when you open the device.
  You can also pass group_id=X when loading the module.
  
  Each RFM12B needs to have the same group ID to talk to each other.
  Change this to whatever you want, (needs to be between 0 and 255).
*/
#define RFM12B_DEFAULT_GROUP_ID   211

/*
  The default frequency band to use for each RFM12B board. You can change
  the band for a particular board via ioctl() when you open the device.
  You can also pass band_id=X when loading the module.
  
  You should only use the band for which your board was built. This should
  be written at the bottom of the RFM12B board.
  
  The settings are as follows
     1 ... 433mhz
     2 ... 868mhz
*/
#define RFM12B_DEFAULT_BAND_ID   2

/*
  The default bit rate to use for each RFM12B board. You can change the
  bit rate for a particular board via ioctl() when you open the device.
  You can also pass bit_rate=X when loading the module.
  
  The bit rate determines how fast the board sends and receives data.
  All boards need to be configured to the same rate to communicate.
  
  Please refer to the datasheet for how to calculate this byte. The
  default setting is ~ 49.2kpbs, which is also what jeelib uses.
  <https://github.com/jcw/jeelib>
*/
#define RFM12B_DEFAULT_BIT_RATE  0x06

/*
  SPI settings that the driver will use. You shouldn't need to change
  anything, though you can experiment with higher SPI frequencies -
  however, note that the RFM12b's datasheet mentions that 2.5MHz is
  the highest freq. for receiving, so experiment at your own risk!
*/

#define RFM12B_SPI_MAX_HZ	2500000
#define RFM12B_SPI_MODE		0
#define RFM12B_SPI_BITS		8

/*
  The major and number of minors for registering the rfm12 SPI driver.
  You shouldn't need to change this, unless you get an error trying to
  load the module about the SPI major already being taken. If so, change
  to a different integer.
*/

#define RFM12B_SPI_MAJOR        'r'
#define RFM12B_NUM_SPI_MINORS   32

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
  
  Tip: Leave these at defaults, unless you're actively investigating this.
*/
#define RFM12B_DROP_PACKET_ON_FFOV		1
#define RFM12B_RETRY_SEND_ON_RGUR		  0

/****************************** DON'T EDIT BELOW **************************/

#if RFM12B_BOARD<=0 || RFM12B_BOARD>2
#error Please specify your board. (RFM12B_BOARD in rfm12b_config.h).
#else
#define MODULE_BOARD_CONFIGURED 1
#endif

#if defined(MODULE_BOARD_CONFIGURED)
#if BUILD_MODULE
#include "platform/platform.h"

#if RFM12B_BOARD==1
#include "platform/plat_raspberrypi.h"

#elif RFM12B_BOARD==2
#include "platform/plat_beaglebone.h"
#endif

#include "platform/plat_spi.h"
#endif // BUILD_MODULE

#if RFM12B_BOARD==1
#define RF12_TESTS_DEV      "/dev/" RFM12B_DEV_NAME ".0.1"
#elif RFM12B_BOARD==2
#define RF12_TESTS_DEV      "/dev/" RFM12B_DEV_NAME ".2.1"
#endif

#endif // MODULE_BOARD_CONFIGURED

#endif // __RFM12B_CONFIG_H__