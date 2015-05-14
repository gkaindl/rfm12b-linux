/* rfm12b-linux: linux kernel driver for the rfm12(b) RF module by HopeRF
*  Copyright (C) 2013 Georg Kaindl
*  
*  This file is part of rfm12b-linux.
*  
*  rfm12b-linux is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, either version 2 of the License, or
*  (at your option) any later version.
*  
*  rfm12b-linux is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*  
*  You should have received a copy of the GNU General Public License
*  along with rfm12b-linux.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/spi/spi.h>

#define BUILD_MODULE   1

#include "rfm12b_config.h"

#if defined(MODULE_BOARD_CONFIGURED)

#include "rfm12b_ioctl.h"
#include "rfm12b_jeenode.h"

#define RFM69     1

static u8 group_id      = RFM12B_DEFAULT_GROUP_ID;
static u8 band_id         = RFM12B_DEFAULT_BAND_ID;
static u8 bit_rate      = RFM12B_DEFAULT_BIT_RATE;
static u8 jee_id         = RFM12B_DEFAULT_JEE_ID;
static u8 jee_autoack   = RFM12B_DEFAULT_JEE_AUTOACK;

module_param(group_id, byte, 0000);
MODULE_PARM_DESC(group_id,
   "group ID for rfm12b modules. can be changed per board via ioctl().");
module_param(band_id, byte, 0000);
MODULE_PARM_DESC(band_id,
   "band ID for rfm12b modules. can be changed per board via ioctl(). "
   "1 .. 433mhz; 2 .. 868mhz; 3 .. 915mhz");
module_param(bit_rate, byte, 0000);
MODULE_PARM_DESC(bit_rate,
   "bit rate setting byte for rfm12b modules (see datasheet). "
   "can be changed per board via ioctl().");
module_param(jee_id, byte, 0000);
MODULE_PARM_DESC(jee_id,
   "jeenode id for rfm12b modules, or 0 to disable the jee protocol. this "
   "can be changed per board via ioctl().");
module_param(jee_autoack, byte, 0000);
MODULE_PARM_DESC(jee_autoack,
   "if the jee-protocol is enabled by setting a jee-id, enable or disable "
   "the automated sending of ACKs when a packet requests them. Can also be "
   "changed per board via ioctl().");

#define RF_READ_STATUS     0x0000
#define RF_IDLE_MODE       0x820D
#define RF_SLEEP_MODE      0x8205
#define RF_TXREG_WRITE     0xB800
#define RF_RECEIVER_ON     0x82DD
#define RF_XMITTER_ON      0x823D
#define RF_RX_FIFO_READ    0xB000

#define RF_STATUS_BIT_LBAT          (0x0400)
#define RF_STATUS_BIT_FFEM          (0x0200)
#define RF_STATUS_BIT_FFOV_RGUR     (0x2000)
#define RF_STATUS_BIT_RSSI          (0x0100)
#define RF_STATUS_BIT_FFIT_RGIT     (0x8000)

#define RFM69_MASK_REGWRITE         (0x80)
   
#define RFM69_REG_IRQFLAGS2         (0x28)
#define RFM69_REG_FIFO              (0x00)
#define RFM69_REG_OPMODE            (0x01)
#define RFM69_REG_RSSIVALUE         (0x24)
#define RFM69_REG_DIOMAPPING1       (0x25)
   
#define RFM69_MODE_STANDBY          (0x04)
#define RFM69_MODE_RECEIVER         (0x10)
#define RFM69_MODE_XMITTER          (0x0C)
   
#define RFM69_DIOMAPPING1_DIO0_00   (0x00)
#define RFM69_DIOMAPPING1_DIO0_10   (0x80)
   
#define RFM69_IRQ2_FIFOFULL         (0x80)
#define RFM69_IRQ2_FIFONOTEMPTY     (0x40)
#define RFM69_IRQ2_FIFOOVERRUN      (0x10)

#define RFM69_RSSIVAL_TO_DBM(val)   (-(((int)(val))>>1))
#define RFM69_RSSIVAL_SEND_MIN      (-90)

#define RF_MAX_DATA_LEN    66
#define RF_EXTRA_LEN       4 // 4 : 1 byte hdr, 1 byte len, 2 bytes crc16 (see JeeLib)
#define RF_MAX_LEN         (RF_MAX_DATA_LEN+RF_EXTRA_LEN)

#define OPEN_WAIT_MILLIS   (50)

#define READ_FIFO_WAIT           (0)
#define WRITE_TX_WAIT            (0)

#define RXTX_WATCHDOG_JIFFIES    (HZ/4)
#define TRYSEND_RETRY_JIFFIES    (HZ/16)

#define DATA_BUF_SIZE            (512)
#define NUM_MAX_CONCURRENT_MSG   (3)

#define INTERPRETS_JEENODE_PROTOCOL(rfm12)   (0 != (rfm12)->jee_id)
#define CAN_SEND_BYTES_OF_LENGTH(LEN)      \
   (DATA_BUF_SIZE - (rfm12->out_cur_end - rfm12->out_buf) >= (LEN) + RF_EXTRA_LEN)

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static DECLARE_BITMAP(minors, RFM12B_NUM_SPI_MINORS);

typedef enum _rfm12_state_t {
   RFM12_STATE_NO_CHANGE      = 0,
   RFM12_STATE_CONFIG         = 1,
   RFM12_STATE_SLEEP            = 2,
   RFM12_STATE_IDLE            = 3,
   RFM12_STATE_LISTEN         = 4,
   RFM12_STATE_RECV            = 5,
   RFM12_STATE_RECV_FINISH      = 6,
   RFM12_STATE_SEND_PRE1      = 7,
   RFM12_STATE_SEND_PRE2      = 8,
   RFM12_STATE_SEND_PRE3      = 9,
   RFM12_STATE_SEND_SYN1      = 10,
   RFM12_STATE_SEND_SYN2      = 11,
   RFM12_STATE_SEND            = 12,
   RFM12_STATE_SEND_TAIL1      = 13,
   RFM12_STATE_SEND_TAIL2      = 14,
   RFM12_STATE_SEND_TAIL3      = 15,
   RFM12_STATE_SEND_FINISHED  = 16
} rfm12_state_t;

typedef enum _rfm12_module_type_t {
	RFM12_TYPE_RF12	= 0,
	RFM12_TYPE_RF69
} rfm12_module_type_t;

struct rfm12_spi_message {
   struct spi_message   spi_msg;
   struct spi_transfer  spi_transfers[4];
   rfm12_state_t        spi_finish_state;
   u8                   spi_tx[8], spi_rx[8];
   void*                context;
   u8                   pos;
};

struct rfm12_data {
   u16                   irq;
   void*                 irq_identifier;

   dev_t                devt;
   spinlock_t           lock;
   struct spi_device*   spi;
   struct list_head     device_entry;

   rfm12_module_type_t  module_type;

   u8                   open, should_release, trysend;
   rfm12_state_t        state;
   u8                   group_id, band_id, bit_rate, jee_id, jee_autoack;
   unsigned long        bytes_recvd, pkts_recvd;
   unsigned long        bytes_sent, pkts_sent;
   unsigned long        num_recv_overflows, num_recv_timeouts, num_recv_crc16_fail;
   unsigned long        num_send_underruns, num_send_timeouts;
   u8*                  in_buf, *in_buf_pos;
   u8*                  out_buf, *out_buf_pos;
   u8*                  in_cur_len_pos;
   u8*                  in_cur_end, *out_cur_end;
   u16                  crc16, last_status;
   int                  in_cur_num_bytes, out_cur_num_bytes;
   struct rfm12_spi_message spi_msgs[NUM_MAX_CONCURRENT_MSG];
   u8                   free_spi_msgs;
   struct timer_list    rxtx_watchdog;
   u8                   rxtx_watchdog_running;
   struct timer_list    retry_sending_timer;
   u8                   retry_sending_running;
   wait_queue_head_t    wait_read;
   wait_queue_head_t    wait_write;
   
   u8                   rf69_req_mode;
   void (*rf69_mode_callback)(void*); 
   void (*rf69_flush_fifo_callback)(struct rfm12_data*);
};

// forward declarations
static struct rfm12_spi_message*
rfm_claim_spi_message(struct rfm12_data* rfm12);
static void
rfm_unclaim_spi_message(struct rfm12_spi_message* spi_msg);
struct spi_transfer
rfm_make_spi_transfer(uint16_t cmd, u8* tx_buf, u8* rx_buf);
struct spi_transfer
rfm_control_spi_transfer(struct rfm12_spi_message* msg,
   u8 pos, uint16_t cmd);
static void
rfm_spi_completion_common(struct rfm12_spi_message* msg);
static void
__rfm_generic_spi_completion_handler(void *arg);
static void
rfm_generic_spi_completion_handler(void *arg);
static int
rfm_send_generic_async_cmd(struct rfm12_data* rfm12, uint16_t* cmds,
                         int num_cmds, uint16_t delay_usecs,
                         void (*callback)(void*),
                         rfm12_state_t finish_state);
static rfm12_module_type_t
rfm_detect_module_type(struct rfm12_data* rfm12);
static const char*
rfm_string_for_module_type(rfm12_module_type_t module_type);
static int
rfm_start_receiving_common(struct rfm12_data* rfm12);
static int
rfm_reset(struct rfm12_data* rfm12);
static void
rfm_begin_sending_or_receiving(struct rfm12_data* rfm12);
static int
rfm_consume_received_byte(struct rfm12_data* rfm12, u8 recvd_byte);
static int
rfm_finish_receiving(struct rfm12_data* rfm12, int skip_packet);
static void
rfm_update_rxtx_watchdog(struct rfm12_data* rfm12, u8 cancelTimer);
static void
rfm_rxtx_watchdog_expired(unsigned long ptr);
static void
rfm_apply_crc16(struct rfm12_data* rfm12, unsigned char* ptr, unsigned len);
static u16
rfm_crc16_update(u16 crc, u8 b);
static void
rfm_finish_send_or_recv_callback(void *arg);
static void
rfm_finish_trysend(struct rfm12_data* rfm12);
static void
rfm_start_trysend_retry_timer(struct rfm12_data* rfm12);
static void
rfm_trysend_retry_timer_expired(unsigned long ptr);
static int
rfm_finish_sending(struct rfm12_data* rfm12, int success);
static void
rfm_stop_listening_and_send_callback(void *arg);
static void
rfm_release_when_safe(struct rfm12_data* rfm12);
static void
rfm_disable_hardware_on_release_handler(void *arg);

static int
rfmXX_setup(struct rfm12_data* rfm12);
static int
rfmXX_disable_hardware_on_release(struct rfm12_data* rfm12);
static int
rfmXX_try_sending(struct rfm12_data* rfm12);
static int
rfmXX_start_receiving(struct rfm12_data* rfm12);
static int
rfmXX_finish_send_recv_common(struct rfm12_data* rfm12);
static int
rfmXX_stop_listening_and_send(struct rfm12_data* rfm12);
static void
rfmXX_handle_interrupt(struct rfm12_data* rfm12);

static int
rfm12_setup(struct rfm12_data* rfm12);
static int
rfm12_disable_hardware_on_release(struct rfm12_data* rfm12);
static int
rfm12_start_receiving(struct rfm12_data* rfm12);
static int
rfm12_request_fifo_byte(struct rfm12_data* rfm12);
static int
rfm12_finish_send_recv_common(struct rfm12_data* rfm12);
static int
rfm12_try_sending(struct rfm12_data* rfm12);
static void
rfm12_trysend_completion_handler(void *arg);
static int
rfm12_stop_listening_and_send(struct rfm12_data* rfm12);
static void
rfm12_handle_interrupt(struct rfm12_data* rfm12);

static int
rfm69_flush_fifo_blocking(struct rfm12_data* rfm12);
static int
rfm69_set_mode(struct rfm12_data* rfm12, u8 mode, void (*spi_callback)(void*));
static void
rfm69_set_mode_handler(void* arg);
static int
rfm69_setup(struct rfm12_data* rfm12);
static int
rfm69_disable_hardware_on_release(struct rfm12_data* rfm12);
static int
rfm69_start_receiving(struct rfm12_data* rfm12);
static void
rfm69_start_receiving_with_empty_fifo(struct rfm12_data* rfm12);
static int
rfm69_flush_fifo_with_finish_handler(struct rfm12_data* rfm12,
   void (*a_handler)(struct rfm12_data*));
static void
rfm69_flush_fifo_async_completion_handler(void* arg);
static void
rfm69_flush_fifo_async_loop_handler(void* arg);
static int
rfm69_try_sending(struct rfm12_data* rfm12);
static void
rfm69_trysend_completion_handler(void *arg);
static int
rfm69_stop_listening_and_send(struct rfm12_data* rfm12);
static void
rfm69_start_sending_with_empty_fifo(struct rfm12_data* rfm12);
static void
rfm69_start_sending_with_empty_fifo_handler(void* arg);
static void
rfm69_set_mode_transmitter_callback(void* arg);
static int
rfm69_send_fifo_loop(struct rfm12_data* rfm12);
static void
rfm69_send_fifo_loop_callback(void* arg);
static int
rfm69_poll_receive_fifo(struct rfm12_data* rfm12);
static void
rfm69_poll_receive_fifo_completion_handler(void* arg);
static int
rfm69_request_fifo_byte(struct rfm12_data* rfm12);
static void
rfm69_request_fifo_byte_completion_handler(void* arg);
static int
rfm69_finish_send_recv_common(struct rfm12_data* rfm12);
static void
rfm69_handle_interrupt(struct rfm12_data* rfm12);

static ssize_t
rfm_filop_read(struct file* filp, char __user *buf, size_t count,
   loff_t* f_pos);
static ssize_t
rfm_filop_write(struct file *filp, const char __user *buf,
   size_t count, loff_t *f_pos);
static long
rfm_filop_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static int
rfm_filop_open(struct inode *inode, struct file *filp);
static int
rfm_filop_release(struct inode *inode, struct file *filp);

// 3.8 kernels and later make these obsolete due to changes in HOTPLUG
// we keep them for compatibility with earlier kernel versions...
#ifndef __devinit
#define __devinit
#endif
#ifndef __devexit
#define __devexit
#endif
#ifndef __devexit_p
#define __devexit_p(x)   x
#endif

static struct rfm12_spi_message*
rfm_claim_spi_message(struct rfm12_data* rfm12)
{
   u8 i;
   struct rfm12_spi_message* rv = NULL;

   for (i=0; i < NUM_MAX_CONCURRENT_MSG; i++) {
     if (0 == (rfm12->free_spi_msgs & (1 << i))) {
       rfm12->free_spi_msgs |= (1 << i);
       rv = &rfm12->spi_msgs[i];
       rv->pos = i;
       rv->context = rfm12;

       break;
     }
   }

   return rv;
}

static void
rfm_unclaim_spi_message(struct rfm12_spi_message* spi_msg)
{
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   rfm12->free_spi_msgs &= ~(1 << spi_msg->pos);   
}

struct spi_transfer
rfm_make_spi_transfer(uint16_t cmd, u8* tx_buf, u8* rx_buf)
{
   struct spi_transfer tr = {
     .tx_buf           = tx_buf,
     .rx_buf           = rx_buf,
     .len              = 2,
     .cs_change        = 0,
     .bits_per_word    = 0,
     .delay_usecs      = 0,
     .speed_hz         = 0
   };

   tx_buf[0] = (cmd >> 8) & 0xff;
   tx_buf[1] = cmd & 0xff;

   return tr;
}

struct spi_transfer
rfm_control_spi_transfer(struct rfm12_spi_message* msg,
   u8 pos, uint16_t cmd)
{   
   return rfm_make_spi_transfer(cmd,
            msg->spi_tx + 2*pos,
            msg->spi_rx + 2*pos);
}

static void
rfm_spi_completion_common(struct rfm12_spi_message* msg)
{
   rfm_unclaim_spi_message(msg);
}

static void
__rfm_generic_spi_completion_handler(void *arg)
{
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   if (RFM12_STATE_NO_CHANGE != spi_msg->spi_finish_state)
     rfm12->state = spi_msg->spi_finish_state;

   rfm_spi_completion_common(spi_msg);
}

static void
rfm_generic_spi_completion_handler(void *arg)
{
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   __rfm_generic_spi_completion_handler(arg);

   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static int
rfm_send_generic_async_cmd(struct rfm12_data* rfm12, uint16_t* cmds,
                         int num_cmds, uint16_t delay_usecs,
                         void (*callback)(void*),
                         rfm12_state_t finish_state)
{
   int err, i;
   struct rfm12_spi_message* spi_msg;   

   spi_msg = rfm_claim_spi_message(rfm12);

   if (NULL == spi_msg)
     return -EBUSY;

   spi_msg->spi_finish_state = finish_state;

   spi_message_init(&spi_msg->spi_msg);

   spi_msg->spi_msg.complete = 
     (NULL == callback) ? rfm_generic_spi_completion_handler : callback;
   spi_msg->spi_msg.context = (void*)spi_msg;

   spi_msg->spi_transfers[0] =
     rfm_control_spi_transfer(spi_msg, 0, cmds[0]);

   i=1;
   if (num_cmds > 1) {
        for (i=1; i<num_cmds; i++) {
           spi_msg->spi_transfers[i-1].cs_change = 1;
           spi_msg->spi_transfers[i-1].delay_usecs = delay_usecs;
           spi_message_add_tail(&spi_msg->spi_transfers[i-1], &spi_msg->spi_msg);
   
           spi_msg->spi_transfers[i] =
               rfm_control_spi_transfer(spi_msg, i, cmds[i]);
        }
   } 
   
   spi_message_add_tail(&spi_msg->spi_transfers[i-1], &spi_msg->spi_msg);

   err = spi_async(rfm12->spi, &spi_msg->spi_msg);
   if (err)
     __rfm_generic_spi_completion_handler((void*)spi_msg);

   return err;
}

static rfm12_module_type_t
rfm_detect_module_type(struct rfm12_data* rfm12)
{
   rfm12_module_type_t module_type = RFM12_TYPE_RF12;
	int err;
	u8* sync_ptr;
	u8 tx_buf[4];
	u8 rx_buf[2];
	u8 init_synchro[] = { 0xAA, 0x55, 0};
      
	err = 0;
	sync_ptr = init_synchro;
	
	while (0 == err && 0 != *sync_ptr) {
		int num_tries = 10;
		
		while (0 == err && --num_tries > 0) {
			struct spi_message msg;
			struct spi_transfer tr, tr2;
      
         rx_buf[0] = 0;
         rx_buf[1] = 0;
      
			tr = rfm_make_spi_transfer(
					0x2F00 | (((u16)RFM69_MASK_REGWRITE) << 8) | *sync_ptr,
					tx_buf,
					NULL
			);
               
         tr.cs_change = 1;
					
			tr2 = rfm_make_spi_transfer(
					0x2F00,
					tx_buf+2,
					rx_buf
			);
               			
         spi_message_init(&msg);
			spi_message_add_tail(&tr, &msg);
			spi_message_add_tail(&tr2, &msg);
			
			err = spi_sync(rfm12->spi, &msg);
			
			if (0 != err || rx_buf[1] == *sync_ptr) {
				break;
			}
		}
		
		if (0 != err) {
			goto pError;
		}
		
		if (0 < num_tries) {
		   module_type = RFM12_TYPE_RF69;
      } else {
         module_type = RFM12_TYPE_RF12;
      }
		      
		sync_ptr++;
	}

pError:   
   return module_type;
}

static const char*
rfm_string_for_module_type(rfm12_module_type_t module_type)
{
   const char* name = "RFM12(B)";
   
   switch (module_type) {
      case RFM12_TYPE_RF69:
         name = "RFM69W";
         break;
      default:
         break;
   }
   
   return name;
}

static int
rfm_start_receiving_common(struct rfm12_data* rfm12)
{
   rfm12->state = RFM12_STATE_LISTEN;   
   rfm12->in_cur_num_bytes = 0;
   
   return 0;
}

static int
rfm_reset(struct rfm12_data* rfm12)
{   
   rfmXX_setup(rfm12);
   rfm_begin_sending_or_receiving(rfm12);
      
   return 0;
}

static int
rfm_consume_received_byte(struct rfm12_data* rfm12, u8 recvd_byte)
{
   uint16_t packet_finished = 0;

   if (RFM12_STATE_LISTEN == rfm12->state) {
      rfm12->state = RFM12_STATE_RECV;
   }

   if (NULL != rfm12->in_buf) {
      if (rfm12->in_buf_pos < (rfm12->in_buf + DATA_BUF_SIZE)) {
         *rfm12->in_buf_pos++ = recvd_byte;

         if (0 == rfm12->in_cur_num_bytes)
            rfm12->crc16 = rfm_crc16_update(~0, rfm12->group_id);

         rfm12->crc16 = rfm_crc16_update(rfm12->crc16, recvd_byte);
      }

      if (1 == rfm12->in_cur_num_bytes) {
         rfm12->in_cur_len_pos = rfm12->in_buf_pos-1;
         
         if (*rfm12->in_cur_len_pos > RF_MAX_DATA_LEN)
         // if the data len is larger than RF_MAX_DATA_LEN, we
         // ignore this packet early.
         *rfm12->in_cur_len_pos = 0;
      }

      if (1 < rfm12->in_cur_num_bytes) {         
         // +2 ... those are the CRC bytes, +2 for header & length
         if (rfm12->in_cur_num_bytes+1 >= (*rfm12->in_cur_len_pos + RF_EXTRA_LEN) ||
         rfm12->in_cur_num_bytes+1 >= (RF_MAX_LEN)) {
            (void)rfm_finish_receiving(rfm12, 0);
            packet_finished = 1;
         }
      }

      if (!packet_finished) {
         rfm12->in_cur_num_bytes++;  
         rfm_update_rxtx_watchdog(rfm12, 0);
      } 
   } else {
      (void)rfm_finish_receiving(rfm12, 1);
      packet_finished = 1;
   }
      
   return packet_finished;
}

static int
rfm_finish_receiving(struct rfm12_data* rfm12, int skip_packet)
{
   int err = 0, num_bytes;

   if (RFM12_STATE_RECV == rfm12->state) {
     rfm12->state = RFM12_STATE_RECV_FINISH;

     num_bytes = rfm12->in_cur_num_bytes;
     rfm12->in_cur_num_bytes = 0;
     rfm_update_rxtx_watchdog(rfm12, 1);

     if (0 == skip_packet && 0 == rfm12->crc16) {
       rfm12->pkts_recvd++;
       rfm12->bytes_recvd += num_bytes - 3;
       
       // if we are in jeenode-compatible mode and this packet
       // needs an ACK, we send one automatically to the source node.
       if (INTERPRETS_JEENODE_PROTOCOL(rfm12) && rfm12->jee_autoack && CAN_SEND_BYTES_OF_LENGTH(0)) {
          u8 hdr = rfm12->in_cur_len_pos[-1];

          if ((hdr & RFM12B_JEE_HDR_ACK_BIT) && !(hdr & RFM12B_JEE_HDR_CTL_BIT)) {
             u8 snd_hdr = 0;
             
             if (hdr & RFM12B_JEE_HDR_DST_BIT) {
                // if this was only sent to us, send ACK as broadcast
                snd_hdr |= RFM12B_JEE_HDR_CTL_BIT | RFM12B_JEE_ID_FROM_HDR(rfm12->jee_id);
             } else {
                // if this was a broadcast, send ACK directly to source node.
                snd_hdr |= RFM12B_JEE_HDR_CTL_BIT | RFM12B_JEE_HDR_DST_BIT |
                            RFM12B_JEE_ID_FROM_HDR(hdr);
             }
             
             *rfm12->out_cur_end++ = snd_hdr;             
             *rfm12->out_cur_end++ = (u8)0;
             
             rfm_apply_crc16(rfm12, rfm12->out_cur_end-2, 2);
             
             rfm12->out_cur_end += 2;   // crc16 = 2 bytes
          }
       }
       
       rfm12->in_cur_end = rfm12->in_buf_pos;
       
       wake_up_interruptible(&rfm12->wait_read);
     } else {
       if (0 == skip_packet && 0 != rfm12->crc16)
          rfm12->num_recv_crc16_fail++;
        
       rfm12->in_buf_pos = rfm12->in_cur_end;
     }
     
     err = rfmXX_finish_send_recv_common(rfm12);
   }

   return err;
}

static void
rfm_begin_sending_or_receiving(struct rfm12_data* rfm12)
{      
   if (RFM12_STATE_IDLE == rfm12->state) {      
      if (!rfmXX_try_sending(rfm12))
         rfmXX_start_receiving(rfm12);
   }
}

static void
rfm_rxtx_watchdog_expired(unsigned long ptr)
{
   unsigned long flags;
   struct rfm12_data* rfm12 = (struct rfm12_data*)ptr;

   spin_lock_irqsave(&rfm12->lock, flags);

   if (RFM12_STATE_RECV >= rfm12->state &&
         RFM12_STATE_LISTEN <= rfm12->state) {
     rfm12->num_recv_timeouts++;
     (void)rfm_finish_receiving(rfm12, 1);
   } else if (RFM12_STATE_SEND_PRE1 <= rfm12->state &&
        RFM12_STATE_SEND_TAIL3 >= rfm12->state) {
     rfm12->num_send_timeouts++;
     (void)rfm_finish_sending(rfm12, 0);        
   }

   rfm12->rxtx_watchdog_running = 0;
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static void
rfm_update_rxtx_watchdog(struct rfm12_data* rfm12, u8 cancelTimer)
{
   if (rfm12->rxtx_watchdog_running) {
     if (cancelTimer) {
       // not del_timer_sync, because we might be called within our
       // own expiration handler
       del_timer(&rfm12->rxtx_watchdog);
       rfm12->rxtx_watchdog_running = 0;
     } else
       mod_timer(&rfm12->rxtx_watchdog,
         jiffies + RXTX_WATCHDOG_JIFFIES);
   } else if (!cancelTimer) {
     init_timer(&rfm12->rxtx_watchdog);
     rfm12->rxtx_watchdog.expires = jiffies + RXTX_WATCHDOG_JIFFIES;
     rfm12->rxtx_watchdog.data = (unsigned long)rfm12;
     rfm12->rxtx_watchdog.function = rfm_rxtx_watchdog_expired;
     add_timer(&rfm12->rxtx_watchdog);
     rfm12->rxtx_watchdog_running = 1;
   }
}

static void
rfm_apply_crc16(struct rfm12_data* rfm12, unsigned char* ptr, unsigned len)
{
   u16 i, crc16 = ~0;
   if (0 != rfm12->group_id)
      crc16 = rfm_crc16_update(~0, rfm12->group_id);
   for (i=0; i<len; i++)
      crc16 = rfm_crc16_update(crc16, ptr[i]);
   
   // crc16
   ptr[len] = crc16 & 0xFF;
   ptr[len+1] = (crc16 >> 8) & 0xFF;
}

static u16
rfm_crc16_update(u16 crc, u8 b)
{
   int i=0;
   
   crc ^= b;
   for (i = 0; i < 8; ++i) {
      if (crc & 1)
         crc = (crc >> 1) ^ 0xa001;
      else
         crc = (crc >> 1);
   }
   
   return crc;
}

static void
rfm_finish_send_or_recv_callback(void *arg)
{
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;
   u8 should_release = 0;
   
   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);
  
   if ((should_release = rfm12->should_release)) {
     spin_unlock_irqrestore(&rfm12->lock, flags);
     rfm_release_when_safe(rfm12);
     spin_lock_irqsave(&rfm12->lock, flags);
   } else {
        rfm_begin_sending_or_receiving(rfm12);
   }
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   if (!should_release) {
      platform_irq_handled(rfm12->irq_identifier);
   }
}

static void
rfm_finish_trysend(struct rfm12_data* rfm12)
{
   rfm12->retry_sending_running = 0;
   rfm12->trysend = 0;
}

static void
rfm_start_trysend_retry_timer(struct rfm12_data* rfm12)
{
   init_timer(&rfm12->retry_sending_timer);
   rfm12->retry_sending_timer.expires = jiffies + TRYSEND_RETRY_JIFFIES;
   rfm12->retry_sending_timer.data = (unsigned long)rfm12;
   rfm12->retry_sending_timer.function = rfm_trysend_retry_timer_expired;
   add_timer(&rfm12->retry_sending_timer);
   rfm12->retry_sending_running = 1;
}

static void
rfm_trysend_retry_timer_expired(unsigned long ptr)
{
   unsigned long flags;
   struct rfm12_data* rfm12 = (struct rfm12_data*)ptr;

   spin_lock_irqsave(&rfm12->lock, flags);

   if (rfm12->retry_sending_running) {
      rfm_finish_trysend(rfm12);
      rfmXX_try_sending(rfm12);
   } else {
      rfm_finish_trysend(rfm12);
   }
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static int
rfm_finish_sending(struct rfm12_data* rfm12, int success)
{
   int err = 0, len = 0;
   
   rfm_update_rxtx_watchdog(rfm12, 1);
         
   if (success) {
      len = rfm12->out_buf[1] + RF_EXTRA_LEN;
      
      memmove(rfm12->out_buf,
               rfm12->out_buf + len,
               DATA_BUF_SIZE - len
      );
      
      rfm12->out_cur_end -= len;
      rfm12->out_buf_pos = rfm12->out_buf;
      
      rfm12->pkts_sent++;
      rfm12->bytes_sent += len - RF_EXTRA_LEN;
      
      wake_up_interruptible(&rfm12->wait_write);
   }
   
   err = rfmXX_finish_send_recv_common(rfm12);
   
   return err;
}

static void
rfm_stop_listening_and_send_callback(void *arg)
{
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);
   
   rfm_begin_sending_or_receiving(rfm12);
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static void
rfm_release_when_safe(struct rfm12_data* rfm12)
{   
   rfm12->should_release = 0;

   platform_irq_cleanup(rfm12->irq_identifier);
      
   rfm_update_rxtx_watchdog(rfm12, 1);
      
   (void)rfmXX_disable_hardware_on_release(rfm12);   
}

static void
rfm_disable_hardware_on_release_handler(void *arg)
{
   unsigned long flags;
   int dofree = 0;
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   spin_lock_irqsave(&rfm12->lock, flags);
   
   rfm_spi_completion_common(spi_msg);
   
   kfree(rfm12->in_buf);
   rfm12->in_buf = rfm12->in_buf_pos = NULL;
   rfm12->out_buf = rfm12->out_buf_pos = NULL;
   
   dofree = (rfm12->spi == NULL);
      
   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   if (dofree) {
      kfree(rfm12);
   }
}

/* RFMXX Wrappers ----------------------------------------------*/

static int
rfmXX_setup(struct rfm12_data* rfm12)
{
   int err = 0;
   
   switch (rfm12->module_type) {
      case RFM12_TYPE_RF12:
         err = rfm12_setup(rfm12);
         break;
      case RFM12_TYPE_RF69:
         err = rfm69_setup(rfm12);
         break;
      default:
         err = -EINVAL;
         break;
   }
   
   if (0 == err) {
      printk(KERN_INFO RFM12B_DRV_NAME
          ": transceiver <0x%x> (%s) settings now: "
          "group %d, band %d, bit rate 0x%.2x (%d bps), "
          "jee id: %d, jee autoack: %d.\n",
          (unsigned)rfm12->irq_identifier,
          rfm_string_for_module_type(rfm12->module_type),
          rfm12->group_id, rfm12->band_id,
          rfm12->bit_rate, RFM12B_BIT_RATE_FROM_BYTE(rfm12->bit_rate),
          rfm12->jee_id, rfm12->jee_autoack);
   }
   
   return err;
}

static int
rfmXX_try_sending(struct rfm12_data* rfm12)
{
   int rv = 0;
   
   switch (rfm12->module_type) {
      case RFM12_TYPE_RF12:
         rv = rfm12_try_sending(rfm12);
         break;
      case RFM12_TYPE_RF69:
         rv = rfm69_try_sending(rfm12);
         break;
      default:
         break;
   }
   
   return rv;
}

static int
rfmXX_start_receiving(struct rfm12_data* rfm12)
{
   int err = 0;
   
   switch (rfm12->module_type) {
      case RFM12_TYPE_RF12:
         err = rfm12_start_receiving(rfm12);
         break;
      case RFM12_TYPE_RF69:
         err = rfm69_start_receiving(rfm12);
         break;
      default:
         break;
   }
   
   return err;
}

static int
rfmXX_finish_send_recv_common(struct rfm12_data* rfm12)
{
   int err = 0;
   
   switch (rfm12->module_type) {
      case RFM12_TYPE_RF12:
         err = rfm12_finish_send_recv_common(rfm12);
         break;
      case RFM12_TYPE_RF69:
         err = rfm69_finish_send_recv_common(rfm12);
         break;
      default:
         break;
   }
   
   return err;
}

static int
rfmXX_stop_listening_and_send(struct rfm12_data* rfm12)
{
   int err = 0;
   
   switch (rfm12->module_type) {
      case RFM12_TYPE_RF12:
         err = rfm12_stop_listening_and_send(rfm12);
         break;
      case RFM12_TYPE_RF69:
         err = rfm69_stop_listening_and_send(rfm12);
         break;
      default:
         break;
   }
   
   return err;
}

static int
rfmXX_disable_hardware_on_release(struct rfm12_data* rfm12)
{
   int err = 0;
   
   switch (rfm12->module_type) {
      case RFM12_TYPE_RF12:
         err = rfm12_disable_hardware_on_release(rfm12);
         break;
      case RFM12_TYPE_RF69:
         err = rfm69_disable_hardware_on_release(rfm12);
         break;
      default:
         break;
   }
   
   return err;
}

static void
rfmXX_handle_interrupt(struct rfm12_data* rfm12)
{   
   switch (rfm12->module_type) {
      case RFM12_TYPE_RF12:
         rfm12_handle_interrupt(rfm12);
         break;
      case RFM12_TYPE_RF69:
         rfm69_handle_interrupt(rfm12);
         break;
      default:
         break;
   }
}

/* RFM12  ------------------------------------------------------*/

static int
rfm12_start_receiving(struct rfm12_data* rfm12)
{
   int err;
   uint16_t cmd[2];

   // read the fifo before listening to make sure it's empty
   cmd[0] = RF_RX_FIFO_READ;
   cmd[1] = RF_RECEIVER_ON;
   
   err = rfm_send_generic_async_cmd(rfm12, cmd, 2, 0,
     NULL, RFM12_STATE_NO_CHANGE);

   if (0 == err) {
      (void)rfm_start_receiving_common(rfm12);
   }

   return err;
}

static int
rfm12_setup(struct rfm12_data* rfm12)
{
   struct spi_transfer tr, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9,
     tr10, tr11, tr12, tr13;
   struct spi_message msg;
   u8 tx_buf[26];
   int err;

   rfm12->state = RFM12_STATE_CONFIG;

   spi_message_init(&msg);

   tr = rfm_make_spi_transfer(RF_READ_STATUS, tx_buf+0, NULL);
   tr.cs_change = 1;
   spi_message_add_tail(&tr, &msg);

   tr2 = rfm_make_spi_transfer(RF_SLEEP_MODE, tx_buf+2, NULL);
   tr2.cs_change = 1;
   spi_message_add_tail(&tr2, &msg);

   tr3 = rfm_make_spi_transfer(RF_TXREG_WRITE, tx_buf+4, NULL);
   spi_message_add_tail(&tr3, &msg);

   err = spi_sync(rfm12->spi, &msg);

   if (err)
     goto pError;

   msleep(OPEN_WAIT_MILLIS);

   // ok, we're now ready to be configured.
   spi_message_init(&msg);

   tr = rfm_make_spi_transfer(0x80C7 |
         ((rfm12->band_id & 0xff) << 4), tx_buf+0, NULL);
   tr.cs_change = 1;
   spi_message_add_tail(&tr, &msg);

   tr2 = rfm_make_spi_transfer(0xA640, tx_buf+2, NULL);
   tr2.cs_change = 1;
   spi_message_add_tail(&tr2, &msg);

   tr3 = rfm_make_spi_transfer(0xC600 | rfm12->bit_rate, tx_buf+4, NULL);
   tr3.cs_change = 1;
   spi_message_add_tail(&tr3, &msg);

   tr4 = rfm_make_spi_transfer(0x94A2, tx_buf+6, NULL);
   tr4.cs_change = 1;
   spi_message_add_tail(&tr4, &msg);

   tr5 = rfm_make_spi_transfer(0xC2AC, tx_buf+8, NULL);
   tr5.cs_change = 1;
   spi_message_add_tail(&tr5, &msg);

   if (0 != rfm12->group_id) {
     tr6 = rfm_make_spi_transfer(0xCA83, tx_buf+10, NULL);
     tr6.cs_change = 1;
     spi_message_add_tail(&tr6, &msg);

     tr7 = rfm_make_spi_transfer(0xCE00 |
        rfm12->group_id, tx_buf+12, NULL);
     tr7.cs_change = 1;
     spi_message_add_tail(&tr7, &msg);
   } else {
     tr6 = rfm_make_spi_transfer(0xCA8B, tx_buf+10, NULL);
     tr6.cs_change = 1;
     spi_message_add_tail(&tr6, &msg);

     tr7 = rfm_make_spi_transfer(0xCE2D, tx_buf+12, NULL);
     tr7.cs_change = 1;
     spi_message_add_tail(&tr7, &msg);
   }

   tr8 = rfm_make_spi_transfer(0xC483, tx_buf+14, NULL);
   tr8.cs_change = 1;
   spi_message_add_tail(&tr8, &msg);

   tr9 = rfm_make_spi_transfer(0x9850, tx_buf+16, NULL);
   tr9.cs_change = 1;
   spi_message_add_tail(&tr9, &msg);

   tr10 = rfm_make_spi_transfer(0xCC77, tx_buf+18, NULL);
   tr10.cs_change = 1;
   spi_message_add_tail(&tr10, &msg);

   tr11 = rfm_make_spi_transfer(0xE000, tx_buf+20, NULL);
   tr11.cs_change = 1;
   spi_message_add_tail(&tr11, &msg);

   tr12 = rfm_make_spi_transfer(0xC800, tx_buf+22, NULL);
   tr12.cs_change = 1;
   spi_message_add_tail(&tr12, &msg);

   // set low battery threshold to 2.9V
   tr13 = rfm_make_spi_transfer(0xC047, tx_buf+24, NULL);
   spi_message_add_tail(&tr13, &msg);

   err = spi_sync(rfm12->spi, &msg);

   if (0 == err) {
     spi_message_init(&msg);

     tr = rfm_make_spi_transfer(RF_READ_STATUS, tx_buf+0, NULL);
     spi_message_add_tail(&tr, &msg);

     err = spi_sync(rfm12->spi, &msg);
   }

pError:
   rfm12->state = RFM12_STATE_IDLE;

   return err;
}

// 0 ... nothing to send, can go to listen state
// 1 ... something needs sending, don't go to listen state
static int
rfm12_try_sending(struct rfm12_data* rfm12)
{   
   int retval = 0;

   if (!rfm12->trysend
      && NULL != rfm12->out_buf
      && rfm12->out_cur_end != rfm12->out_buf) {
     
      uint16_t cmd = RF_READ_STATUS;
      
      (void)rfm_send_generic_async_cmd(rfm12, &cmd, 1,
         0, rfm12_trysend_completion_handler, RFM12_STATE_NO_CHANGE);
      
      retval = rfm12->trysend = 1;
   }
         
   return retval;
}

static int
rfm12_disable_hardware_on_release(struct rfm12_data* rfm12)
{
   u16 cmd = RF_SLEEP_MODE;
   
   return rfm_send_generic_async_cmd(rfm12, &cmd, 1,
      0, rfm_disable_hardware_on_release_handler, RFM12_STATE_NO_CHANGE);
}

static int
rfm12_finish_send_recv_common(struct rfm12_data* rfm12)
{
   uint16_t cmds[3];
   
   cmds[0] = RF_READ_STATUS;
   cmds[1] = RF_IDLE_MODE;
   cmds[2] = RF_TXREG_WRITE | 0xAA;
   
   rfm12->state = RFM12_STATE_IDLE;
      
   return rfm_send_generic_async_cmd(rfm12, cmds, 3,
      0, rfm_finish_send_or_recv_callback, RFM12_STATE_NO_CHANGE);
}

static void
rfm12_recv_spi_completion_handler(void *arg)
{
   u8* recv_data;
   unsigned long flags;
   uint16_t status, packet_finished = 0, valid_interrupt = 0;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);

   recv_data = spi_msg->spi_rx;

   status = (recv_data[0] << 8) | recv_data[1];
   
   valid_interrupt =
      ((status & RF_STATUS_BIT_FFIT_RGIT) && !(status & RF_STATUS_BIT_FFEM)) ||
      (status & RF_STATUS_BIT_FFOV_RGUR);
   
   rfm12->last_status = status;

   if (valid_interrupt) {
      packet_finished = rfm_consume_received_byte(rfm12, recv_data[3]);

      if ((status & RF_STATUS_BIT_FFOV_RGUR) && !packet_finished) {
        rfm12->num_recv_overflows++;

        if (RFM12B_DROP_PACKET_ON_FFOV)
           (void)rfm_finish_receiving(rfm12, 1);
      }
   }

   spin_unlock_irqrestore(&rfm12->lock, flags);

   if (!rfm12->should_release &&
        (!valid_interrupt ||
        (!packet_finished && (RFM12B_DROP_PACKET_ON_FFOV &&
         !(status & RF_STATUS_BIT_FFOV_RGUR))))) {
         platform_irq_handled(rfm12->irq_identifier);
   }
}

static void
rfm12_send_spi_completion_handler(void *arg)
{
   unsigned long flags;
   uint16_t status, valid_interrupt = 0, packet_finished = 0;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);

   status = (spi_msg->spi_rx[0] << 8) | spi_msg->spi_rx[1];
   rfm12->last_status = status;

   valid_interrupt =
         ((status & RF_STATUS_BIT_FFIT_RGIT) || (status & RF_STATUS_BIT_FFOV_RGUR));

   if (valid_interrupt && NULL != rfm12->out_buf) {
      if (RFM12B_RETRY_SEND_ON_RGUR && (status & RF_STATUS_BIT_FFOV_RGUR)) {
         packet_finished = 1;
         rfm12->num_send_underruns++;
         (void)rfm_finish_sending(rfm12, 0);
      } else {
         if (!RFM12B_RETRY_SEND_ON_RGUR && (status & RF_STATUS_BIT_FFOV_RGUR))
               rfm12->num_send_underruns++;
         
         switch(rfm12->state) {
            case RFM12_STATE_SEND_PRE1:
                 rfm12->out_buf_pos = rfm12->out_buf;
                 rfm12->out_cur_num_bytes = rfm12->out_buf_pos[1] + RF_EXTRA_LEN;
            case RFM12_STATE_SEND_PRE2:
            case RFM12_STATE_SEND_PRE3:
            case RFM12_STATE_SEND_SYN1:
            case RFM12_STATE_SEND_SYN2:
            case RFM12_STATE_SEND_TAIL1:
            case RFM12_STATE_SEND_TAIL2:
                 rfm12->state++;
                 break;
            case RFM12_STATE_SEND:
               rfm12->out_cur_num_bytes--;
               
               if (0 == rfm12->out_cur_num_bytes) {
                  rfm12->state = RFM12_STATE_SEND_TAIL1;
               }
               
               break;
            case RFM12_STATE_SEND_TAIL3:
                 packet_finished = 1;
                 (void)rfm_finish_sending(rfm12, 1);
                 break;
            default:
               // should never happen
               packet_finished = 1;
               (void)rfm_finish_sending(rfm12, 0);
               break;
         }
         
         if (!packet_finished)
               rfm_update_rxtx_watchdog(rfm12, 0);
         }
   }

   spin_unlock_irqrestore(&rfm12->lock, flags);

   if (!rfm12->should_release && (!valid_interrupt || !packet_finished))
         platform_irq_handled(rfm12->irq_identifier);
}

static int
rfm12_request_fifo_byte(struct rfm12_data* rfm12)
{
   u16 cmds[2];

   cmds[0] = RF_READ_STATUS;
   cmds[1] = RF_RX_FIFO_READ;

   return rfm_send_generic_async_cmd(rfm12, cmds, 2,
      READ_FIFO_WAIT, rfm12_recv_spi_completion_handler,
      RFM12_STATE_NO_CHANGE);
}

static int
rfm_write_tx_byte(struct rfm12_data* rfm12, u8 tx_byte)
{
   uint16_t cmds[2];
   
   cmds[0] = RF_READ_STATUS;
   cmds[1] = RF_TXREG_WRITE | tx_byte;
   
   return rfm_send_generic_async_cmd(rfm12, cmds, 2,
      WRITE_TX_WAIT, rfm12_send_spi_completion_handler,
      RFM12_STATE_NO_CHANGE);
}

static void
rfm12_trysend_completion_handler(void *arg)
{
   unsigned long flags;
   uint16_t status = 0;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_finish_trysend(rfm12);

   rfm_spi_completion_common(spi_msg);

   status = (spi_msg->spi_rx[0] << 8) | spi_msg->spi_rx[1];

   rfm12->last_status = status;
      
   if ((RFM12_STATE_IDLE == rfm12->state || RFM12_STATE_LISTEN == rfm12->state) &&
       0 == (status & RF_STATUS_BIT_RSSI)) {
      uint16_t cmd[4];
            
      cmd[0] = RF_IDLE_MODE;
      cmd[1] = RF_READ_STATUS;
      cmd[2] = RF_RX_FIFO_READ;
      cmd[3] = RF_XMITTER_ON;
      
      rfm12->state = RFM12_STATE_SEND_PRE1;
      
      rfm_update_rxtx_watchdog(rfm12, 0);
            
      rfm_send_generic_async_cmd(rfm12, cmd, 4,
            0, NULL, RFM12_STATE_NO_CHANGE);
   } else if (RFM12_STATE_SEND_PRE1 > rfm12->state) {
      // try again a bit later...
      rfm_start_trysend_retry_timer(rfm12);
   }
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static int
rfm12_stop_listening_and_send(struct rfm12_data* rfm12)
{
   uint16_t cmd[2];
   int err = 0;

   if (RFM12_STATE_LISTEN == rfm12->state) {
      cmd[0] = RF_IDLE_MODE;
      cmd[1] = RF_READ_STATUS;
      
      rfm12->state = RFM12_STATE_IDLE;
      
      err = rfm_send_generic_async_cmd(rfm12, cmd, 2,
         0, rfm_stop_listening_and_send_callback, RFM12_STATE_NO_CHANGE);
   }
   
   return err;
}

// this should only be called from an interrupt context
static void
rfm12_handle_interrupt(struct rfm12_data* rfm12)
{   
   spin_lock(&rfm12->lock);
   
   if (RFM12_TYPE_RF69 == rfm12->module_type) {      
      switch (rfm12->state) {
         case RFM12_STATE_LISTEN:
         case RFM12_STATE_RECV:
            rfm12->state = RFM12_STATE_RECV;
            rfm_update_rxtx_watchdog(rfm12, 0);
            (void)rfm69_poll_receive_fifo(rfm12);
            break;
         
         case RFM12_STATE_SEND_PRE1:
         case RFM12_STATE_SEND_PRE2:
         case RFM12_STATE_SEND_PRE3:
         case RFM12_STATE_SEND_SYN1:
         case RFM12_STATE_SEND_SYN2:
         case RFM12_STATE_SEND:
         case RFM12_STATE_SEND_TAIL1:
         case RFM12_STATE_SEND_TAIL2:
         case RFM12_STATE_SEND_TAIL3: {
            rfm12->num_send_underruns++;
            rfm_finish_sending(rfm12, 0);
            break;
         }
         
         case RFM12_STATE_SEND_FINISHED: {
            rfm_finish_sending(rfm12, 1);
            break;
         }
         
         default:
            // bogus/uninteresting interrupt
            platform_irq_handled(rfm12->irq_identifier);
            break;
      }
      
      spin_unlock(&rfm12->lock);
      
      return;
   }
   
   switch (rfm12->state) {
     case RFM12_STATE_LISTEN:
     case RFM12_STATE_RECV:
       (void)rfm12_request_fifo_byte(rfm12);
       break;
     case RFM12_STATE_SEND_PRE1:
     case RFM12_STATE_SEND_PRE2:
     case RFM12_STATE_SEND_PRE3:
     case RFM12_STATE_SEND_TAIL1:
     case RFM12_STATE_SEND_TAIL2:
        (void)rfm_write_tx_byte(rfm12, 0xAA);
        break;
     case RFM12_STATE_SEND_TAIL3: {
      uint16_t cmd = RF_IDLE_MODE;
        (void)rfm_send_generic_async_cmd(rfm12, &cmd, 1, 0,
           NULL, RFM12_STATE_NO_CHANGE);
        (void)rfm_write_tx_byte(rfm12, 0xAA);
        break;
     }
     case RFM12_STATE_SEND_SYN1:
         (void)rfm_write_tx_byte(rfm12, 0x2D);
         break;
     case RFM12_STATE_SEND_SYN2:
         (void)rfm_write_tx_byte(rfm12, rfm12->group_id);
         break;
     case RFM12_STATE_SEND:
        (void)rfm_write_tx_byte(rfm12, *rfm12->out_buf_pos++);
        break;
     default: {
       uint16_t cmd = RF_READ_STATUS; 
       (void)rfm_send_generic_async_cmd(rfm12, &cmd, 1,
          0, NULL, RFM12_STATE_NO_CHANGE);
       break;
     }
   }
   
   spin_unlock(&rfm12->lock);
}

/* RFM69 ------------------------------------------------------*/

static int
rfm69_flush_fifo_blocking(struct rfm12_data* rfm12)
{
   int err = 0;
   u8 it = 100;
   u8 tx_buf[4];
   u8 rx_buf[2];
   
   do {
      struct spi_message msg;
   	struct spi_transfer tr, tr2;

   	tr = rfm_make_spi_transfer(
   			((u16)RFM69_REG_FIFO) << 8,
   			tx_buf,
   			NULL
   	);
		
      tr.cs_change = 1;
      
   	tr2 = rfm_make_spi_transfer(
   			((u16)RFM69_REG_IRQFLAGS2) << 8,
   			tx_buf+2,
   			rx_buf
   	);
		
      spi_message_init(&msg);
   	spi_message_add_tail(&tr, &msg);
   	spi_message_add_tail(&tr2, &msg);
	
   	err = spi_sync(rfm12->spi, &msg);      
   } while (it-- > 0
         && 0 == err
         && 0 != (rx_buf[1] & RFM69_IRQ2_FIFONOTEMPTY));
      
   return err;
}

static int
rfm69_set_mode(struct rfm12_data* rfm12, u8 mode,
   void (*spi_callback)(void*))
{
   u16 cmd = ((u16)RFM69_REG_OPMODE) << 8;
   
   int err = rfm_send_generic_async_cmd(rfm12, &cmd, 1, 0,
     rfm69_set_mode_handler, RFM12_STATE_NO_CHANGE);
   
   rfm12->rf69_req_mode = mode;
   rfm12->rf69_mode_callback = spi_callback;
      
   return err;
}

static void
rfm69_set_mode_handler(void* arg)
{
   u16 cmd;
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);
      
   cmd = (((u16)(RFM69_REG_OPMODE | RFM69_MASK_REGWRITE)) << 8) | 
      (spi_msg->spi_rx[1] & 0xE3) | rfm12->rf69_req_mode;
   
   (void)rfm_send_generic_async_cmd(rfm12, &cmd, 1, 0,
     rfm12->rf69_mode_callback, RFM12_STATE_NO_CHANGE);
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static int
rfm69_setup(struct rfm12_data* rfm12)
{
	// based on https://github.com/jcw/jeelib/blob/master/RF69.cpp
	int err;
	u32 rf69_freq;
	u8* cmd_ptr;
	u8 tx_buf[4];
	u32 bitrate_actual = 0;
   u8 config_init[] = {
		0x01, 0x04, // OpMode = standby
		0x02, 0x00, // DataModul = packet mode, fsk
		0x03, 0x00, // BitRateMsb, data rate = 49,261 khz
		0x04, 0x00, // BitRateLsb, divider = 32 MHz / 650
		0x05, 0x05, // FdevMsb = 90 KHz
		0x06, 0xC3, // FdevLsb = 90 KHz
		0x0B, 0x20, // AfcCtrl, afclowbetaon
      0x18, 0x88, // LNA gain selected by AGC loop, 50 ohm load
		0x19, 0x42, // RxBw ...
		0x1E, 0x2C, // FeiStart, AfcAutoclearOn, AfcAutoOn
		0x25, 0x80, // DioMapping1 = SyncAddress (Rx)
		0x2E, 0x88, // SyncConfig = sync on, sync size = 2
		0x2F, 0x2D, // SyncValue1 = 0x2D
		0x37, 0x00, // PacketConfig1 = fixed, no crc, filt off
		0x38, 0x00, // PayloadLength = 0, unlimited
		0x3C, 0x8F, // FifoTresh, not empty, level 15
		0x3D, 0x10, // PacketConfig2, interpkt = 1, autorxrestart off
		0x6F, 0x20, // TestDagc ...
		0x30, 0x00, // synchro2,
		0x07, 0x00, // freq1
		0x08, 0x00, // freq2
		0x09, 0x00, // freq3
		0
   };
   
	if (0 == rfm12->group_id) {
		printk(KERN_WARNING RFM12B_DRV_NAME
			": group id 0 doesn't work well for RFM69!\n");
	}
	
	cmd_ptr = config_init;
	
	bitrate_actual = RFM12B_BIT_RATE_FROM_BYTE(rfm12->bit_rate);
	bitrate_actual = (32000000UL + bitrate_actual / 2) / bitrate_actual;
	
	cmd_ptr[5] = (bitrate_actual >> 8) & 0xff;
	cmd_ptr[7] = bitrate_actual & 0xff;
	
	cmd_ptr[37] = rfm12->group_id;
	
	switch (rfm12->band_id) {
		case 1: rf69_freq = 43; break;
		case 3: rf69_freq = 90; break;
		default: rf69_freq = 86; break;
	}
	
	rf69_freq = rf69_freq * 10000000L + rfm12->band_id * 2500L * 1600UL;
	rf69_freq = ((rf69_freq << 2) / (32000000L >> 11)) << 6;
	
	cmd_ptr[39] = (rf69_freq >> 16) & 0xff;
	cmd_ptr[41] = (rf69_freq >> 8)  & 0xff;
	cmd_ptr[43] = (rf69_freq)       & 0xff;
   
   err = 0;
   
	while (0 == err && 0 != *cmd_ptr) {
		struct spi_message msg;
		struct spi_transfer tr =
			rfm_make_spi_transfer(
				(((u16)(cmd_ptr[0] | RFM69_MASK_REGWRITE)) << 8) | cmd_ptr[1],
				tx_buf,
				NULL
			);
            
      spi_message_init(&msg);
		spi_message_add_tail(&tr, &msg);
		
		err = spi_sync(rfm12->spi, &msg);
		
		cmd_ptr += 2;
	}
 
   rfm12->state = RFM12_STATE_IDLE;
   
   if (0 == err) {
      msleep(OPEN_WAIT_MILLIS);
      
      rfm69_flush_fifo_blocking(rfm12);
   }

   return err;
}

static int
rfm69_start_receiving(struct rfm12_data* rfm12)
{
   return rfm69_flush_fifo_with_finish_handler(rfm12,
      rfm69_start_receiving_with_empty_fifo);
}

static void
rfm69_start_receiving_with_empty_fifo(struct rfm12_data* rfm12)
{
   (void)rfm69_set_mode(rfm12, RFM69_MODE_RECEIVER, NULL);
   (void)rfm_start_receiving_common(rfm12);
}

static int
rfm69_flush_fifo_with_finish_handler(struct rfm12_data* rfm12,
   void (*a_handler)(struct rfm12_data*))
{
   u16 cmd = ((u16)RFM69_REG_IRQFLAGS2) << 8;
 
   rfm12->rf69_flush_fifo_callback = a_handler;
      
   return rfm_send_generic_async_cmd(rfm12, &cmd, 1,
      0,
      rfm69_flush_fifo_async_completion_handler,
      RFM12_STATE_NO_CHANGE);
}

static void
rfm69_flush_fifo_async_completion_handler(void* arg)
{
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   rfm_spi_completion_common(spi_msg);
   
   if (spi_msg->spi_rx[1] & (RFM69_IRQ2_FIFONOTEMPTY | RFM69_IRQ2_FIFOOVERRUN)) {
      u16 cmd = ((u16)RFM69_REG_FIFO) << 8;
      
      (void)rfm_send_generic_async_cmd(rfm12, &cmd, 1,
         0,
         rfm69_flush_fifo_async_loop_handler,
         RFM12_STATE_NO_CHANGE);
   } else {
      if (NULL != rfm12->rf69_flush_fifo_callback) {
         rfm12->rf69_flush_fifo_callback(rfm12);
      }
   }
}

static void
rfm69_flush_fifo_async_loop_handler(void* arg)
{
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   rfm_spi_completion_common(spi_msg);

   rfm69_flush_fifo_with_finish_handler(rfm12,
      rfm12->rf69_flush_fifo_callback);
}

static int
rfm69_try_sending(struct rfm12_data* rfm12)
{
   int retval = 0;

   if (!rfm12->trysend
      && NULL != rfm12->out_buf
      && rfm12->out_cur_end != rfm12->out_buf) {
     
      uint16_t cmd = ((u16)RFM69_REG_RSSIVALUE) << 8;
      
      (void)rfm_send_generic_async_cmd(rfm12, &cmd, 1,
         0, rfm69_trysend_completion_handler, RFM12_STATE_NO_CHANGE);
      
      retval = rfm12->trysend = 1;
   }
         
   return retval;
}

static void
rfm69_trysend_completion_handler(void *arg)
{
   unsigned long flags;
   int rssi = 0;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);
   
   rfm_finish_trysend(rfm12);

   rssi = RFM69_RSSIVAL_TO_DBM(spi_msg->spi_rx[1]);
      
   if ((RFM12_STATE_IDLE == rfm12->state || RFM12_STATE_LISTEN == rfm12->state) &&
       rssi < RFM69_RSSIVAL_SEND_MIN) {
      rfm12->state = RFM12_STATE_SEND_PRE1;
      
      rfm69_set_mode(rfm12, RFM69_MODE_STANDBY, NULL);
            
      rfm69_flush_fifo_with_finish_handler(rfm12,
         rfm69_start_sending_with_empty_fifo);
   } else if (RFM12_STATE_SEND_PRE1 > rfm12->state) {
      // try again a bit later...
      rfm_start_trysend_retry_timer(rfm12);
   }
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static void
rfm69_start_sending_with_empty_fifo(struct rfm12_data* rfm12)
{
   u16 cmd = (((u16)(RFM69_REG_DIOMAPPING1 | RFM69_MASK_REGWRITE)) << 8)
      | RFM69_DIOMAPPING1_DIO0_00;
   
   (void)rfm_send_generic_async_cmd(rfm12, &cmd, 1,
      0, rfm69_start_sending_with_empty_fifo_handler, RFM12_STATE_NO_CHANGE);
}

static void
rfm69_start_sending_with_empty_fifo_handler(void* arg)
{
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   rfm_spi_completion_common(spi_msg);

   rfm69_set_mode(rfm12, RFM69_MODE_XMITTER,
      rfm69_set_mode_transmitter_callback);
}

static void
rfm69_set_mode_transmitter_callback(void* arg)
{
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   rfm_spi_completion_common(spi_msg);
   
   rfm12->out_buf_pos = rfm12->out_buf;
   rfm12->out_cur_num_bytes = rfm12->out_buf_pos[1] + RF_EXTRA_LEN;
   rfm12->state = RFM12_STATE_SEND;
   
   rfm_update_rxtx_watchdog(rfm12, 0);
      
   (void)rfm69_send_fifo_loop(rfm12);
}

static int
rfm69_send_fifo_loop(struct rfm12_data* rfm12)
{
   unsigned long flags;
   u16 cmds[2];
   u8 out_byte;
   int packet_finished = 0, err = 0;
   
   spin_lock_irqsave(&rfm12->lock, flags);
   
   if (rfm12->state < RFM12_STATE_SEND) {
      spin_unlock_irqrestore(&rfm12->lock, flags);
      return 0;
   }
   
   switch (rfm12->state) {
      case RFM12_STATE_SEND: {
         if (0 < rfm12->out_cur_num_bytes) {
            out_byte = *rfm12->out_buf_pos++;
            rfm12->out_cur_num_bytes--;
         } else {
            out_byte = 0xAA;
            rfm12->state = RFM12_STATE_SEND_FINISHED;
         }
         
         break;
      }
      
      case RFM12_STATE_SEND_FINISHED: {
         packet_finished = 1;
         break;
      }
      
      default: {
         out_byte = 0xAA;
         rfm12->state++;
         
         break;
      }
   }
   
   if (!packet_finished) {
      rfm_update_rxtx_watchdog(rfm12, 0);
   
      cmds[0] = (((u16)(RFM69_REG_FIFO | RFM69_MASK_REGWRITE)) << 8)
         | out_byte;
      cmds[1] = ((u16)RFM69_REG_IRQFLAGS2) << 8;
   
      err = rfm_send_generic_async_cmd(rfm12, cmds, 2,
         0, rfm69_send_fifo_loop_callback, RFM12_STATE_NO_CHANGE);         
   } else {
      // nothing to do but wait for the packetsent interrupt
      err = 0;
   }
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   return err;
}

static void
rfm69_send_fifo_loop_callback(void* arg)
{
   unsigned long flags;
   u16 cmds[2];
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   spin_lock_irqsave(&rfm12->lock, flags);
   
   rfm_spi_completion_common(spi_msg);
   
   if (spi_msg->spi_rx[3] & RFM69_IRQ2_FIFOFULL) {
      cmds[0] = ((u16)RFM69_REG_IRQFLAGS2) << 8;
      cmds[1] = cmds[0];
      
      (void)rfm_send_generic_async_cmd(rfm12, cmds, 2,
         0, rfm69_send_fifo_loop_callback,
         RFM12_STATE_NO_CHANGE);
      
      spin_unlock_irqrestore(&rfm12->lock, flags);
   } else {
      spin_unlock_irqrestore(&rfm12->lock, flags);
      
      (void)rfm69_send_fifo_loop(rfm12);
   }
}

static int
rfm69_stop_listening_and_send(struct rfm12_data* rfm12)
{
   int err = 0;

   if (RFM12_STATE_LISTEN == rfm12->state) {
      rfm12->state = RFM12_STATE_IDLE;
      
      err = rfm69_set_mode(
         rfm12,
         RFM69_MODE_STANDBY,
         rfm_stop_listening_and_send_callback
      );
   }
   
   return err;
}

static int
rfm69_poll_receive_fifo(struct rfm12_data* rfm12)
{
   u16 cmd = ((u16)(RFM69_REG_IRQFLAGS2)) << 8;
   
   return rfm_send_generic_async_cmd(rfm12, &cmd, 1,
      0, rfm69_poll_receive_fifo_completion_handler,
      RFM12_STATE_NO_CHANGE);
}

static void
rfm69_poll_receive_fifo_completion_handler(void* arg)
{
   unsigned long flags;
   uint16_t status, packet_finished = 0;
   
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);

   status = spi_msg->spi_rx[1];
   
   rfm12->last_status = status;

   if (status & (RFM69_IRQ2_FIFONOTEMPTY | RFM69_IRQ2_FIFOOVERRUN)) {
      if (!(status & RFM69_IRQ2_FIFOOVERRUN)) {
         (void)rfm69_request_fifo_byte(rfm12);
      } else {
         rfm12->num_recv_overflows++;

         if (RFM12B_DROP_PACKET_ON_FFOV) {
            packet_finished = 1;
            
            (void)rfm_finish_receiving(rfm12, 1);
         }
      }
   } else {
      rfm69_poll_receive_fifo(rfm12);
   }

   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static int
rfm69_request_fifo_byte(struct rfm12_data* rfm12)
{
   u16 cmd = ((u16)(RFM69_REG_FIFO)) << 8;
   
   return rfm_send_generic_async_cmd(rfm12, &cmd, 1,
      0, rfm69_request_fifo_byte_completion_handler,
      RFM12_STATE_NO_CHANGE);
}

static void
rfm69_request_fifo_byte_completion_handler(void* arg)
{
   unsigned long flags;
   uint16_t packet_finished = 0;
   
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm_spi_completion_common(spi_msg);
   
   packet_finished = rfm_consume_received_byte(rfm12, spi_msg->spi_rx[1]);

   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   if (packet_finished && !rfm12->should_release) {
      platform_irq_handled(rfm12->irq_identifier);
   } else {
      (void)rfm69_poll_receive_fifo(rfm12);
   }
}

static int
rfm69_finish_send_recv_common(struct rfm12_data* rfm12)
{
   u16 cmd;
   
   rfm69_set_mode(rfm12, RFM69_MODE_STANDBY, NULL);
   
   cmd = (((u16)(RFM69_REG_DIOMAPPING1 | RFM69_MASK_REGWRITE)) << 8)
      | RFM69_DIOMAPPING1_DIO0_10;
   
   rfm12->state = RFM12_STATE_IDLE;
   
   return rfm_send_generic_async_cmd(rfm12, &cmd, 1,
      0, rfm_finish_send_or_recv_callback, RFM12_STATE_NO_CHANGE);
}

static int
rfm69_disable_hardware_on_release(struct rfm12_data* rfm12)
{
   return rfm69_set_mode(rfm12, RFM69_MODE_STANDBY,
      rfm_disable_hardware_on_release_handler);
}

// this should only be called from an interrupt context
static void
rfm69_handle_interrupt(struct rfm12_data* rfm12)
{   
   spin_lock(&rfm12->lock);
   
   switch (rfm12->state) {
      case RFM12_STATE_LISTEN:
      case RFM12_STATE_RECV:
         rfm12->state = RFM12_STATE_RECV;
         rfm_update_rxtx_watchdog(rfm12, 0);
         (void)rfm69_poll_receive_fifo(rfm12);
         break;
      
      case RFM12_STATE_SEND_PRE1:
      case RFM12_STATE_SEND_PRE2:
      case RFM12_STATE_SEND_PRE3:
      case RFM12_STATE_SEND_SYN1:
      case RFM12_STATE_SEND_SYN2:
      case RFM12_STATE_SEND:
      case RFM12_STATE_SEND_TAIL1:
      case RFM12_STATE_SEND_TAIL2:
      case RFM12_STATE_SEND_TAIL3: {
         rfm12->num_send_underruns++;
         rfm_finish_sending(rfm12, 0);
         break;
      }
      
      case RFM12_STATE_SEND_FINISHED: {
         rfm_finish_sending(rfm12, 1);
         break;
      }
      
      default:
         // bogus/uninteresting interrupt
         platform_irq_handled(rfm12->irq_identifier);
         break;
   }
   
   spin_unlock(&rfm12->lock);
}

/* File Operations ------------------------------------------- */

static ssize_t
rfm_filop_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos)
{
   struct rfm12_data* rfm12 = (struct rfm12_data*)filp->private_data;
   int length = 0, bytes_to_copy = 0, mmovelen = 0, offset = 0;
   unsigned long flags;

   if (rfm12->in_cur_end == rfm12->in_buf)
     wait_event_interruptible(rfm12->wait_read,
         (rfm12->in_cur_end > rfm12->in_buf));

   if (rfm12->in_cur_end <= rfm12->in_buf)
     return 0;

   spin_lock_irqsave(&rfm12->lock, flags);

   length = *(rfm12->in_buf + 1) + 2;
   
   // if we are not in jee-compatible mode, we dont pass JeeLib hdr/len
   // bytes to userspace.
   if (INTERPRETS_JEENODE_PROTOCOL(rfm12)) {
      offset = 0;
     bytes_to_copy = (count > length) ? length : count;
   } else {
     offset = 2;
     bytes_to_copy = (count > length-2) ? length-2 : count;
   }

   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   bytes_to_copy -= copy_to_user(buf, rfm12->in_buf+offset, bytes_to_copy);

   spin_lock_irqsave(&rfm12->lock, flags);

   mmovelen = length + 2;
   if (mmovelen > 0)
     memmove(rfm12->in_buf,
       rfm12->in_buf + mmovelen,
       mmovelen);

   rfm12->in_cur_end -= length + 2;
   rfm12->in_buf_pos -= length + 2;

   spin_unlock_irqrestore(&rfm12->lock, flags);

   return bytes_to_copy;
}

static ssize_t
rfm_filop_write(struct file *filp, const char __user *buf,
   size_t count, loff_t *f_pos)
{
   struct rfm12_data* rfm12 = (struct rfm12_data*)filp->private_data;
   int bytes_to_copy = 0, copied = 0, offset = 0;
   unsigned long flags;
   
   if (INTERPRETS_JEENODE_PROTOCOL(rfm12)) {
      offset = 0;
   } else {
      offset = 2;
   }
   
   if (count < offset)
      return -EINVAL;

   bytes_to_copy =
      (RF_MAX_DATA_LEN+2-offset < count) ? RF_MAX_DATA_LEN+2-offset : count;
   
   if (!CAN_SEND_BYTES_OF_LENGTH(bytes_to_copy))
      wait_event_interruptible(rfm12->wait_write,
         CAN_SEND_BYTES_OF_LENGTH(bytes_to_copy));

   copied = bytes_to_copy - copy_from_user(
      rfm12->out_cur_end+offset,
      buf,
      bytes_to_copy
   );

   spin_lock_irqsave(&rfm12->lock, flags);

   if (INTERPRETS_JEENODE_PROTOCOL(rfm12)) {
      // header is taken verbatim from userspace, but we'll check (and fix) len
      rfm12->out_cur_end++;      // skip hdr, it's taken from userspace.
      *rfm12->out_cur_end++ = copied-2;   // don't trust userspace's len field.
   } else {
      // header: we keep this at 0, meaning "broadcast from node 0" in jeenode speak
      //   that way. any flow control etc... has to be implemented by the client app,
      //   but we're still compatible with the jeenode format.
      *rfm12->out_cur_end++ = 0;         // hdr      
      *rfm12->out_cur_end++ = (u8)copied;   // len
   }
      
   rfm_apply_crc16(rfm12, rfm12->out_cur_end-2, copied+offset);

   rfm12->out_cur_end += copied + offset;

    if (RFM12_STATE_IDLE == rfm12->state) {
       rfm_begin_sending_or_receiving(rfm12);
    } else if (RFM12_STATE_LISTEN == rfm12->state) {
       rfmXX_stop_listening_and_send(rfm12);
    }

   spin_unlock_irqrestore(&rfm12->lock, flags);

   return copied;
}

static unsigned int
rfm_filop_poll(struct file* filp, poll_table* wait)
{
   unsigned int mask = 0;
   struct rfm12_data* rfm12 = (struct rfm12_data*)filp->private_data;
   
   poll_wait(filp, &rfm12->wait_read,  wait);
   poll_wait(filp, &rfm12->wait_write, wait);
   
   if (rfm12->in_cur_end > rfm12->in_buf)
      mask |= POLLIN | POLLRDNORM;
   if (DATA_BUF_SIZE - (rfm12->out_cur_end - rfm12->out_buf) >= 1 + RF_EXTRA_LEN)
      mask |= POLLOUT | POLLWRNORM;
   
   return mask;
}

static long
rfm_filop_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
   struct rfm12_data* rfm12 = (struct rfm12_data*)filp->private_data;
   
   switch (cmd) {
      case RFM12B_IOCTL_GET_STATS: {
         rfm12b_stats s;
                  
         s.bytes_recvd = rfm12->bytes_recvd;
         s.pkts_recvd = rfm12->pkts_recvd;
         s.bytes_sent = rfm12->bytes_sent;
         s.pkts_sent = rfm12->pkts_sent;
         s.num_recv_overflows = rfm12->num_recv_overflows;
         s.num_recv_timeouts = rfm12->num_recv_timeouts;
         s.num_recv_crc16_fail = rfm12->num_recv_crc16_fail;
         s.num_send_underruns = rfm12->num_send_underruns;
         s.num_send_timeouts = rfm12->num_send_timeouts;
         s.low_battery = (0 != (rfm12->last_status & RF_STATUS_BIT_LBAT));

         if (0 != copy_to_user((rfm12b_stats*)arg, &s, sizeof(rfm12b_stats)))
               return -EACCES;
         
         break;
      }
      
      case RFM12B_IOCTL_GET_GROUP_ID: {
         int gid = rfm12->group_id;
            
         if (0 != copy_to_user((int*)arg, &gid, sizeof(gid)))
            return -EACCES;
         
         break;
      }
      
      case RFM12B_IOCTL_GET_BAND_ID: {
         int bid = rfm12->band_id;
               
         if (0 != copy_to_user((int*)arg, &bid, sizeof(bid)))
            return -EACCES;
         
         break;
      }
      
      case RFM12B_IOCTL_GET_BIT_RATE: {
         int br = rfm12->bit_rate;
               
         if (0 != copy_to_user((int*)arg, &br, sizeof(br)))
            return -EACCES;
         
         break;
      }
      
      case RFM12B_IOCTL_GET_JEE_ID: {
         int ji = rfm12->jee_id;
               
         if (0 != copy_to_user((int*)arg, &ji, sizeof(ji)))
            return -EACCES;
            
         break;
      }
      
      case RFM12B_IOCTL_GET_JEEMODE_AUTOACK: {
         int ack = rfm12->jee_autoack;
         
         if (0 != copy_to_user((int*)arg, &ack, sizeof(ack)))
            return -EACCES;
         
         break;
      }
      
      case RFM12B_IOCTL_SET_GROUP_ID: {
            int gid;
                                             
            if (0 != copy_from_user(&gid, (int*)arg, sizeof(gid)))
                  return -EACCES;
                           
            rfm12->group_id = gid;
            rfm_reset(rfm12);
            
            break;
      }
      
      case RFM12B_IOCTL_SET_BAND_ID: {
            int bid;
                        
            if (0 != copy_from_user(&bid, (int*)arg, sizeof(bid)))
                  return -EACCES;
                              
            rfm12->band_id = bid;
            rfm_reset(rfm12);
            
            break;
      }
       
      case RFM12B_IOCTL_SET_BIT_RATE: {
            int br;
                        
            if (0 != copy_from_user(&br, (int*)arg, sizeof(br)))
                  return -EACCES;
            
            rfm12->bit_rate = br;
            rfm_reset(rfm12);
            
            break;
      }
      
      case RFM12B_IOCTL_SET_JEE_ID: {
            int ji;
                        
            if (0 != copy_from_user(&ji, (int*)arg, sizeof(ji)))
                  return -EACCES;
            
            rfm12->jee_id = ji;
            rfm_reset(rfm12);
            
            break;
      }
      
      case RFM12B_IOCTL_SET_JEEMODE_AUTOACK: {
         int ack;
         
         if (0 != copy_from_user(&ack, (int*)arg, sizeof(ack)))
           return -EACCES;
        
         rfm12->jee_autoack = (ack ? 1 : 0);
         rfm_reset(rfm12);
         
         break;
      }
      
      default:
            return -EINVAL;
   }
   
   return 0;
}

static int
rfm_filop_open(struct inode *inode, struct file *filp)
{
   struct rfm12_data* rfm12;
   unsigned long flags;
   u32 irq_trigger = 0;
   int err = -ENXIO, has_irq = 0;

   mutex_lock(&device_list_lock);

   list_for_each_entry(rfm12, &device_list, device_entry) {
      if (rfm12->devt == inode->i_rdev) {
         err = 0;
         break;
      }
   }
   if (0 == err) {      
      if (0 != rfm12->open || 0 != rfm12->should_release) {
          err = -EBUSY;
         goto pError;
      }
      
      rfm12->group_id = group_id;
      rfm12->band_id = band_id;
      rfm12->bit_rate = bit_rate;
      rfm12->jee_id = jee_id;
      rfm12->jee_autoack = jee_autoack;
      
      if (0 == err)
          err = rfmXX_setup(rfm12);
      
      if (err) {
          goto pError;
      }
            
      spin_lock_irqsave(&rfm12->lock, flags);
   
      if (!rfm12->in_buf) {
          rfm12->in_buf = kmalloc(2*DATA_BUF_SIZE, GFP_KERNEL);
          if (!rfm12->in_buf) {
            dev_dbg(&rfm12->spi->dev, "open/ENOMEM\n");
               err = -ENOMEM;
         }
      
          rfm12->out_buf = rfm12->in_buf + DATA_BUF_SIZE;
      
          rfm12->out_buf_pos = rfm12->out_buf;
          rfm12->in_buf_pos = rfm12->in_buf;
      }
      
      if (0 == err) {
         rfm12->open++;
         filp->private_data = rfm12;
         nonseekable_open(inode, filp);
      }
      
      if (1 == rfm12->open) {
         rfm12->bytes_recvd = 0;
          rfm12->pkts_recvd = 0;
          rfm12->bytes_sent = 0;
          rfm12->pkts_sent = 0;
          rfm12->num_recv_overflows = 0;
          rfm12->num_recv_timeouts = 0;
          rfm12->num_recv_crc16_fail = 0;
          rfm12->num_send_underruns = 0;
          rfm12->num_send_timeouts = 0;
          rfm12->in_cur_num_bytes = 0;
          rfm12->rxtx_watchdog_running = 0;
          rfm12->retry_sending_running = 0;
          rfm12->crc16 = 0;
          rfm12->last_status = 0;
          rfm12->should_release = 0;
          rfm12->trysend = 0;
          rfm12->in_cur_len_pos = rfm12->in_buf;
          rfm12->in_cur_end = rfm12->in_buf;
          rfm12->out_cur_end = rfm12->out_buf;
      }
      
      spin_unlock_irqrestore(&rfm12->lock, flags);
      
      if (0 == err)
         rfm_begin_sending_or_receiving(rfm12);
      
      switch (rfm12->module_type) {
         case RFM12_TYPE_RF69:
            irq_trigger = IRQF_TRIGGER_RISING;
            break;
         case RFM12_TYPE_RF12:
         default:
            irq_trigger = IRQF_TRIGGER_FALLING;
            break;
      }
      
      err = platform_irq_init(
         rfm12->irq_identifier, irq_trigger, (void*)rfm12);
      has_irq = (0 == err);  
   } else
      pr_debug("rfm12: nothing for minor %d\n", iminor(inode));

pError:

   if (err && has_irq)
     platform_irq_cleanup(rfm12->irq_identifier);

   mutex_unlock(&device_list_lock);
   return err;
}

static int
rfm_filop_release(struct inode *inode, struct file *filp)
{
   unsigned long flags;
   struct rfm12_data* rfm12;
   int err = 0;

   mutex_lock(&device_list_lock);

   rfm12 = filp->private_data;
   filp->private_data = NULL;
   
   spin_lock_irqsave(&rfm12->lock, flags);

   rfm12->open--;
   if (0 == rfm12->open && 0 == rfm12->should_release) {     
     rfm12->should_release = 1;
               
     if (RFM12_STATE_LISTEN == rfm12->state ||
          RFM12_STATE_IDLE == rfm12->state) {
        spin_unlock_irqrestore(&rfm12->lock, flags);
        rfm_release_when_safe(rfm12);
        spin_lock_irqsave(&rfm12->lock, flags);
     }
   }
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
   mutex_unlock(&device_list_lock);

   return err;
}

static const struct file_operations rfm12_fops = {
   .owner            = THIS_MODULE,
   .llseek           = no_llseek,
   .write            = rfm_filop_write,
   .read             = rfm_filop_read,
   .unlocked_ioctl   = rfm_filop_ioctl,
   .poll             = rfm_filop_poll,
   .open             = rfm_filop_open,
   .release          = rfm_filop_release,
};

/* Device Probing ------------------------------------------ */

static struct class* rfm12_class;

static int __devinit
rfm_dev_probe(struct spi_device *spi)
{
   struct rfm12_data* rfm12;
   int err;
   unsigned long minor;

   rfm12 = kzalloc(sizeof(*rfm12), GFP_KERNEL);
   if (!rfm12)
      return -ENOMEM;
      
   rfm12->irq_identifier =
      platform_irq_identifier_for_spi_device(spi->master->bus_num,
                                      spi->chip_select);
   
   if (0 > rfm12->irq_identifier) {
      printk(KERN_ERR RFM12B_DRV_NAME
        ": no IRQ identifier found for %s.%d.%d\n",
      RFM12B_DEV_NAME, spi->master->bus_num, spi->chip_select);
      
      kfree(rfm12);
      
      return -ENODEV;
   }

   rfm12->state = RFM12_STATE_CONFIG;
   rfm12->spi = spi;
   rfm12->open = 0;
   rfm12->free_spi_msgs = 0;
   spin_lock_init(&rfm12->lock);

   INIT_LIST_HEAD(&rfm12->device_entry);
   
   init_waitqueue_head(&rfm12->wait_read);
   init_waitqueue_head(&rfm12->wait_write);

   mutex_lock(&device_list_lock);
   minor = find_first_zero_bit(minors, RFM12B_NUM_SPI_MINORS);
   if (minor < RFM12B_NUM_SPI_MINORS) {
      struct device *dev;

      rfm12->devt = MKDEV(RFM12B_SPI_MAJOR, minor);
      dev = device_create(rfm12_class, &spi->dev, rfm12->devt,
               rfm12, RFM12B_DEV_NAME ".%d.%d",
               spi->master->bus_num, spi->chip_select);
      err = IS_ERR(dev) ? PTR_ERR(dev) : 0;
   } else {
      dev_dbg(&spi->dev, "no minor number available!\n");
      err = -ENODEV;
   }

   if (0 == err) {
      set_bit(minor, minors);
      list_add(&rfm12->device_entry, &device_list);
   }

   mutex_unlock(&device_list_lock);

   if (0 == err) {
      spi_set_drvdata(spi, rfm12);

      rfm12->module_type = rfm_detect_module_type(rfm12);

      printk(KERN_INFO RFM12B_DRV_NAME
         ": added %s transceiver %s.%d.%d\n",
       rfm_string_for_module_type(rfm12->module_type),
       RFM12B_DEV_NAME, spi->master->bus_num, spi->chip_select);
   } else
      kfree(rfm12);

   return err;
}

static int __devexit
rfm_dev_remove(struct spi_device *spi)
{
   unsigned long flags;
   struct rfm12_data* rfm12 = spi_get_drvdata(spi);

   printk(KERN_INFO RFM12B_DRV_NAME
      ": removed %s transceiver %s.%d.%d\n",
     rfm_string_for_module_type(rfm12->module_type),
     RFM12B_DEV_NAME, spi->master->bus_num, spi->chip_select);

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm12->spi = NULL;
   spi_set_drvdata(spi, NULL);

   spin_unlock_irqrestore(&rfm12->lock, flags);

   mutex_lock(&device_list_lock);
   list_del(&rfm12->device_entry);
   device_destroy(rfm12_class, rfm12->devt);
   clear_bit(MINOR(rfm12->devt), minors);
   if (0 == rfm12->open) {
      kfree(rfm12);
   }
   mutex_unlock(&device_list_lock);

   return 0;
}

static struct spi_driver rfm12_spi_driver = {
   .driver = {
      .name =  RFM12B_DRV_NAME,
      .owner =   THIS_MODULE,
   },
   .probe =    rfm_dev_probe,
   .remove =   __devexit_p(rfm_dev_remove),
};

/* module lifecycle */

int __init
rfm_module_init(void)
{
   int err;

   err = platform_module_init();
   if (err)
      goto errReturn;

   err = register_chrdev(RFM12B_SPI_MAJOR, "spi", &rfm12_fops);
   if (err < 0) {
      platform_module_cleanup();
      goto errReturn;
   }

   rfm12_class = class_create(THIS_MODULE, RFM12B_DEV_NAME);
   if (IS_ERR(rfm12_class)) {
      unregister_chrdev(RFM12B_SPI_MAJOR, rfm12_spi_driver.driver.name);
      platform_module_cleanup();
      err = PTR_ERR(rfm12_class);
      goto errReturn;
   }

   err = spi_register_driver(&rfm12_spi_driver);
   if (err < 0) {
      class_destroy(rfm12_class);
      unregister_chrdev(RFM12B_SPI_MAJOR, rfm12_spi_driver.driver.name);
      platform_module_cleanup();
   }

errReturn:
   if (err) {
      printk(
         KERN_ALERT RFM12B_DRV_NAME
         " : driver failed to load: %i.\n",
         err
      );
   } else {
      printk(
         KERN_INFO RFM12B_DRV_NAME
         " : driver loaded.\n"
      );
   }

   return err;
}

void __exit
rfm_module_cleanup(void)
{   
   spi_unregister_driver(&rfm12_spi_driver);
   class_destroy(rfm12_class);
   unregister_chrdev(RFM12B_SPI_MAJOR, rfm12_spi_driver.driver.name);

   (void)platform_module_cleanup();

   printk(
      KERN_INFO RFM12B_DRV_NAME
      " : driver unloaded.\n"
   );
}

module_init(rfm_module_init);
module_exit(rfm_module_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Georg Kaindl <gkaindl --AT-- mac.com>");
MODULE_DESCRIPTION("kernel driver for the rfm12/rfm69 digital radio module");
MODULE_VERSION("0.0.2");

#endif // defined(MODULE_BOARD_CONFIGURED)
