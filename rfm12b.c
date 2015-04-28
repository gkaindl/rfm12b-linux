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
#define RF_STATUS_BIT_FFOV_RGUR      (0x2000)
#define RF_STATUS_BIT_RSSI          (0x0100)
#define RF_STATUS_BIT_FFIT_RGIT      (0x8000)

#define RF_MAX_DATA_LEN    66
#define RF_EXTRA_LEN       4 // 4 : 1 byte hdr, 1 byte len, 2 bytes crc16 (see JeeLib)
#define RF_MAX_LEN         (RF_MAX_DATA_LEN+RF_EXTRA_LEN)

#define OPEN_WAIT_MILLIS   (50)

#define READ_FIFO_WAIT           (0)
#define WRITE_TX_WAIT            (0)

#define RXTX_WATCHDOG_JIFFIES    (HZ/4)
#define TRYSEND_RETRY_JIFFIES      (HZ/16)

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
   RFM12_STATE_SEND_TAIL3      = 15
} rfm12_state_t;

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
   void*                   irq_identifier;

   dev_t                   devt;
   spinlock_t              lock;
   struct spi_device*   spi;
   struct list_head       device_entry;

   u8                   open, should_release, trysend;
   rfm12_state_t        state;
   u8                      group_id, band_id, bit_rate, jee_id, jee_autoack;
   unsigned long        bytes_recvd, pkts_recvd;
   unsigned long        bytes_sent, pkts_sent;
   unsigned long        num_recv_overflows, num_recv_timeouts, num_recv_crc16_fail;
   unsigned long          num_send_underruns, num_send_timeouts;
   u8*                  in_buf, *in_buf_pos;
   u8*                  out_buf, *out_buf_pos;
   u8*                  in_cur_len_pos;
   u8*                  in_cur_end, *out_cur_end;
   u16                   crc16, last_status;
   int                  in_cur_num_bytes, out_cur_num_bytes;
   struct rfm12_spi_message spi_msgs[NUM_MAX_CONCURRENT_MSG];
   u8                     free_spi_msgs;
   struct timer_list    rxtx_watchdog;
   u8                   rxtx_watchdog_running;
   struct timer_list       retry_sending_timer;
   u8                      retry_sending_running;
   wait_queue_head_t       wait_read;
   wait_queue_head_t       wait_write;
};

// forward declarations
static void
rfm12_handle_interrupt(struct rfm12_data* rfm12);
static int
rfm12_request_fifo_byte(struct rfm12_data* rfm12);
static int
rfm12_finish_receiving(struct rfm12_data* rfm12, int skip_packet);
static int
rfm12_finish_sending(struct rfm12_data* rfm12, int success);
static void
rfm12_begin_sending_or_receiving(struct rfm12_data* rfm12);
static int
rfm12_try_sending(struct rfm12_data* rfm12);
static void
rfm12_update_rxtx_watchdog(struct rfm12_data* rfm12, u8 cancelTimer);
static int
rfm12_reset(struct rfm12_data* rfm12);
static void
rfm12_apply_crc16(struct rfm12_data* rfm12, unsigned char* ptr, unsigned len);

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
rfm12_claim_spi_message(struct rfm12_data* rfm12)
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
rfm12_unclaim_spi_message(struct rfm12_spi_message* spi_msg)
{
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   rfm12->free_spi_msgs &= ~(1 << spi_msg->pos);   
}

struct spi_transfer
rfm12_make_spi_transfer(uint16_t cmd, u8* tx_buf, u8* rx_buf)
{
   struct spi_transfer tr = {
     .tx_buf           = tx_buf,
     .rx_buf           = rx_buf,
     .len              = 2,
     .cs_change        = 1,
     .bits_per_word    = 0,
     .delay_usecs      = 0,
     .speed_hz         = 0
   };

   tx_buf[0] = (cmd >> 8) & 0xff;
   tx_buf[1] = cmd & 0xff;

   return tr;
}

struct spi_transfer
rfm12_control_spi_transfer(struct rfm12_spi_message* msg,
   u8 pos, uint16_t cmd)
{   
   return rfm12_make_spi_transfer(cmd,
            msg->spi_tx + 2*pos,
            msg->spi_rx + 2*pos);
}

static void
rfm12_spi_completion_common(struct rfm12_spi_message* msg)
{
   rfm12_unclaim_spi_message(msg);
}

static void
__rfm12_generic_spi_completion_handler(void *arg)
{
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   if (RFM12_STATE_NO_CHANGE != spi_msg->spi_finish_state)
     rfm12->state = spi_msg->spi_finish_state;

   rfm12_spi_completion_common(spi_msg);
}

static void
rfm12_generic_spi_completion_handler(void *arg)
{
   unsigned long flags;
   struct rfm12_data* rfm12;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg;
   if (spi_msg == NULL) {
      printk(KERN_INFO RFM12B_DRV_NAME
        ": spi completion spi_msg = %p", spi_msg);
      return;
   }
    
  rfm12 = (struct rfm12_data*)spi_msg->context;
  if (rfm12 == NULL) {
      printk(KERN_INFO RFM12B_DRV_NAME
        ": spi completion rfm12 = %p", rfm12);
      return;
   }
   spin_lock_irqsave(&rfm12->lock, flags);

   __rfm12_generic_spi_completion_handler(arg);

   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static int
rfm12_send_generic_async_cmd(struct rfm12_data* rfm12, uint16_t* cmds,
                         int num_cmds, uint16_t delay_usecs,
                         void (*callback)(void*),
                         rfm12_state_t finish_state)
{
   int err, i;
   struct rfm12_spi_message* spi_msg;   

   spi_msg = rfm12_claim_spi_message(rfm12);

   if (NULL == spi_msg)
     return -EBUSY;

   spi_msg->spi_finish_state = finish_state;

   spi_message_init(&spi_msg->spi_msg);

   spi_msg->spi_msg.complete = 
     (NULL == callback) ? rfm12_generic_spi_completion_handler : callback;
   spi_msg->spi_msg.context = (void*)spi_msg;

   spi_msg->spi_transfers[0] =
     rfm12_control_spi_transfer(spi_msg, 0, cmds[0]);

   i=1;
   if (num_cmds > 1) {
        for (i=1; i<num_cmds; i++) {
           spi_msg->spi_transfers[i-1].delay_usecs = delay_usecs;
           spi_message_add_tail(&spi_msg->spi_transfers[i-1], &spi_msg->spi_msg);
   
           spi_msg->spi_transfers[i] =
               rfm12_control_spi_transfer(spi_msg, i, cmds[i]);
        }
   } 
   
   spi_message_add_tail(&spi_msg->spi_transfers[i-1], &spi_msg->spi_msg);

   err = spi_async(rfm12->spi, &spi_msg->spi_msg);
   if (err)
     __rfm12_generic_spi_completion_handler((void*)spi_msg);

   return err;
}

static u16
rfm12_crc16_update(u16 crc, u8 b)
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

static int
rfm12_start_receiving(struct rfm12_data* rfm12)
{
   int err;
   uint16_t cmd[2];
   
   rfm12->state = RFM12_STATE_LISTEN;
   
   // read the fifo before listening to make sure it's empty
   cmd[0] = RF_RX_FIFO_READ;
   cmd[1] = RF_RECEIVER_ON;
   
   err = rfm12_send_generic_async_cmd(rfm12, cmd, 2, 0,
     NULL, RFM12_STATE_NO_CHANGE);
   if (0 == err) {
     rfm12->in_cur_num_bytes = 0;
   }

   return err;
}

static int
rfm12_setup(struct rfm12_data* rfm12)
{
   static struct spi_transfer tr, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9,
     tr10, tr11, tr12;
   struct spi_message msg;
   static u8 tx_buf[26];
   int err;

   rfm12->state = RFM12_STATE_CONFIG;

   spi_message_init(&msg);

   tr = rfm12_make_spi_transfer(RF_READ_STATUS, tx_buf+0, NULL);
   spi_message_add_tail(&tr, &msg);

   tr2 = rfm12_make_spi_transfer(RF_SLEEP_MODE, tx_buf+2, NULL);
   spi_message_add_tail(&tr2, &msg);

   tr3 = rfm12_make_spi_transfer(RF_TXREG_WRITE, tx_buf+4, NULL);
   spi_message_add_tail(&tr3, &msg);

   err = spi_sync(rfm12->spi, &msg);

   if (err)
     goto pError;

   msleep(OPEN_WAIT_MILLIS);

   // ok, we're now ready to be configured.
   spi_message_init(&msg);

#if 0

   tr = rfm12_make_spi_transfer(0x80C7 |
         ((rfm12->band_id & 0xff) << 4), tx_buf+0, NULL);
   spi_message_add_tail(&tr, &msg);

   tr2 = rfm12_make_spi_transfer(0xA640, tx_buf+2, NULL);
   spi_message_add_tail(&tr2, &msg);

   tr3 = rfm12_make_spi_transfer(0xC600 | rfm12->bit_rate, tx_buf+4, NULL);
   spi_message_add_tail(&tr3, &msg);

   tr4 = rfm12_make_spi_transfer(0x94A2, tx_buf+6, NULL);
   spi_message_add_tail(&tr4, &msg);

   tr5 = rfm12_make_spi_transfer(0xC2AC, tx_buf+8, NULL);
   spi_message_add_tail(&tr5, &msg);

   if (0 != rfm12->group_id) {
     tr6 = rfm12_make_spi_transfer(0xCA83, tx_buf+10, NULL);
     spi_message_add_tail(&tr6, &msg);

     tr7 = rfm12_make_spi_transfer(0xCE00 |
        rfm12->group_id, tx_buf+12, NULL);
     spi_message_add_tail(&tr7, &msg);
   } else {
     tr6 = rfm12_make_spi_transfer(0xCA8B, tx_buf+10, NULL);
     spi_message_add_tail(&tr6, &msg);

     tr7 = rfm12_make_spi_transfer(0xCE2D, tx_buf+12, NULL);
     spi_message_add_tail(&tr7, &msg);
   }

   tr8 = rfm12_make_spi_transfer(0xC483, tx_buf+14, NULL);
   spi_message_add_tail(&tr8, &msg);

   tr9 = rfm12_make_spi_transfer(0x9850, tx_buf+16, NULL);
   spi_message_add_tail(&tr9, &msg);

   tr10 = rfm12_make_spi_transfer(0xCC77, tx_buf+18, NULL);
   spi_message_add_tail(&tr10, &msg);

   tr11 = rfm12_make_spi_transfer(0xE000, tx_buf+20, NULL);
   spi_message_add_tail(&tr11, &msg);

   tr12 = rfm12_make_spi_transfer(0xC800, tx_buf+22, NULL);
   spi_message_add_tail(&tr12, &msg);

   // set low battery threshold to 2.9V
   tr13 = rfm12_make_spi_transfer(0xC047, tx_buf+24, NULL);
   spi_message_add_tail(&tr13, &msg);

#else

   tr = rfm12_make_spi_transfer(0x8017, tx_buf+0, NULL);
   spi_message_add_tail(&tr, &msg);

   tr2 = rfm12_make_spi_transfer(0x8208, tx_buf+2, NULL);
   spi_message_add_tail(&tr2, &msg);

   tr3 = rfm12_make_spi_transfer(0xA620 | rfm12->bit_rate, tx_buf+4, NULL);
   spi_message_add_tail(&tr3, &msg);

   tr4 = rfm12_make_spi_transfer(0xC647, tx_buf+6, NULL);
   spi_message_add_tail(&tr4, &msg);

   tr5 = rfm12_make_spi_transfer(0x9489, tx_buf+8, NULL);
   spi_message_add_tail(&tr5, &msg);

   tr6 = rfm12_make_spi_transfer(0xC220, tx_buf+10, NULL);
   spi_message_add_tail(&tr6, &msg);

   tr7 = rfm12_make_spi_transfer(0xCA00, tx_buf+12, NULL);
   spi_message_add_tail(&tr7, &msg);

   tr8 = rfm12_make_spi_transfer(0xC4C3, tx_buf+14, NULL);
   spi_message_add_tail(&tr8, &msg);

   tr9 = rfm12_make_spi_transfer(0xCC67, tx_buf+16, NULL);
   spi_message_add_tail(&tr9, &msg);

   tr10 = rfm12_make_spi_transfer(0xC000, tx_buf+18, NULL);
   spi_message_add_tail(&tr10, &msg);

   tr11 = rfm12_make_spi_transfer(0xE000, tx_buf+20, NULL);
   spi_message_add_tail(&tr11, &msg);

   tr12 = rfm12_make_spi_transfer(0xC800, tx_buf+22, NULL);
   spi_message_add_tail(&tr12, &msg);

#endif

   err = spi_sync(rfm12->spi, &msg);

   if (0 == err) {
     spi_message_init(&msg);

     tr = rfm12_make_spi_transfer(RF_READ_STATUS, tx_buf+0, NULL);
     spi_message_add_tail(&tr, &msg);

     err = spi_sync(rfm12->spi, &msg);
   }

   printk(KERN_INFO RFM12B_DRV_NAME
       ": transceiver <0x%x> settings now: group %d, band %d, bit rate 0x%.2x (%d bps), "
       "jee id: %d, jee autoack: %d.\n",
       (unsigned)rfm12->irq_identifier, rfm12->group_id, rfm12->band_id,
       rfm12->bit_rate, RFM12B_BIT_RATE_FROM_BYTE(rfm12->bit_rate),
       rfm12->jee_id, rfm12->jee_autoack);

pError:
   rfm12->state = RFM12_STATE_IDLE;

   return err;
}

static int
rfm12_reset(struct rfm12_data* rfm12)
{   
   rfm12_setup(rfm12);
   rfm12_begin_sending_or_receiving(rfm12);
      
   return 0;
}

static void
rfm12_release_when_safe_sleep_mode_handler(void *arg)
{
   unsigned long flags;
   int dofree = 0;
   struct rfm12_spi_message* spi_msg =
      (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
      (struct rfm12_data*)spi_msg->context;
   
   spin_lock_irqsave(&rfm12->lock, flags);
   
   rfm12_spi_completion_common(spi_msg);
   
   kfree(rfm12->in_buf);
   rfm12->in_buf = rfm12->in_buf_pos = NULL;
   rfm12->out_buf = rfm12->out_buf_pos = NULL;
   
   dofree = (rfm12->spi == NULL);
      
   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   if (dofree) {
      kfree(rfm12);
   }
}

static void
rfm12_release_when_safe(struct rfm12_data* rfm12)
{
   uint16_t cmd;
   
   rfm12->should_release = 0;

   platform_irq_cleanup(rfm12->irq_identifier);
      
   rfm12_update_rxtx_watchdog(rfm12, 1);
      
   cmd = RF_SLEEP_MODE;
   (void)rfm12_send_generic_async_cmd(rfm12, &cmd, 1,
      0, rfm12_release_when_safe_sleep_mode_handler, RFM12_STATE_NO_CHANGE);   
}

static void
rfm12_rxtx_watchdog_expired(unsigned long ptr)
{
   unsigned long flags;
   struct rfm12_data* rfm12 = (struct rfm12_data*)ptr;

   spin_lock_irqsave(&rfm12->lock, flags);

   if (RFM12_STATE_RECV >= rfm12->state &&
         RFM12_STATE_LISTEN <= rfm12->state) {
     rfm12->num_recv_timeouts++;
     (void)rfm12_finish_receiving(rfm12, 1);
   } else if (RFM12_STATE_SEND_PRE1 <= rfm12->state &&
        RFM12_STATE_SEND_TAIL3 >= rfm12->state) {
     rfm12->num_send_timeouts++;
     (void)rfm12_finish_sending(rfm12, 0);        
   }

   rfm12->rxtx_watchdog_running = 0;
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

static void
rfm12_update_rxtx_watchdog(struct rfm12_data* rfm12, u8 cancelTimer)
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
     rfm12->rxtx_watchdog.function = rfm12_rxtx_watchdog_expired;
     add_timer(&rfm12->rxtx_watchdog);
     rfm12->rxtx_watchdog_running = 1;
   }
}

static void
rfm12_finish_send_or_recv_callback(void *arg)
{
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;
   u8 should_release = 0;
   
   spin_lock_irqsave(&rfm12->lock, flags);

   rfm12_spi_completion_common(spi_msg);
  
   if ((should_release = rfm12->should_release)) {
     spin_unlock_irqrestore(&rfm12->lock, flags);
     rfm12_release_when_safe(rfm12);
     spin_lock_irqsave(&rfm12->lock, flags);
   } else {
        rfm12_begin_sending_or_receiving(rfm12);
   }
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   if (!should_release)
      platform_irq_handled(rfm12->irq_identifier);
}

static int
rfm12_finish_send_recv_common(struct rfm12_data* rfm12)
{
   uint16_t cmds[3];
   
   cmds[0] = RF_READ_STATUS;
   cmds[1] = RF_IDLE_MODE;
   cmds[2] = RF_TXREG_WRITE | 0xAA;
   
   rfm12->state = RFM12_STATE_IDLE;
      
   return rfm12_send_generic_async_cmd(rfm12, cmds, 3,
      0, rfm12_finish_send_or_recv_callback, RFM12_STATE_NO_CHANGE);
}

static int
rfm12_finish_receiving(struct rfm12_data* rfm12, int skip_packet)
{
   int err = 0, num_bytes;

   if (RFM12_STATE_RECV == rfm12->state) {
     rfm12->state = RFM12_STATE_RECV_FINISH;

     num_bytes = rfm12->in_cur_num_bytes;
     rfm12->in_cur_num_bytes = 0;
     rfm12_update_rxtx_watchdog(rfm12, 1);

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
             
             rfm12_apply_crc16(rfm12, rfm12->out_cur_end-2, 2);
             
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
     
     err = rfm12_finish_send_recv_common(rfm12);
   }

   return err;
}

static int
rfm12_finish_sending(struct rfm12_data* rfm12, int success)
{
   int err = 0, len = 0;
   
   rfm12_update_rxtx_watchdog(rfm12, 1);
         
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
   
   err = rfm12_finish_send_recv_common(rfm12);
   
   return err;
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

   rfm12_spi_completion_common(spi_msg);

   recv_data = spi_msg->spi_rx;

   status = (recv_data[0] << 8) | recv_data[1];
   rfm12->last_status = status;

   valid_interrupt =
         ((status & RF_STATUS_BIT_FFIT_RGIT) && !(status & RF_STATUS_BIT_FFEM)) ||
         (status & RF_STATUS_BIT_FFOV_RGUR);
      
   if (valid_interrupt && RFM12_STATE_LISTEN == rfm12->state)
         rfm12->state = RFM12_STATE_RECV;

   if (valid_interrupt) {
      if (NULL != rfm12->in_buf) {
        if (rfm12->in_buf_pos < (rfm12->in_buf + DATA_BUF_SIZE)) {
          *rfm12->in_buf_pos++ = recv_data[3];
          
            if (0 == rfm12->in_cur_num_bytes)
                rfm12->crc16 = rfm12_crc16_update(~0, rfm12->group_id);
            
            rfm12->crc16 = rfm12_crc16_update(rfm12->crc16, recv_data[3]);
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
            (void)rfm12_finish_receiving(rfm12, 0);
            packet_finished = 1;
          }
        }
   
        if (!packet_finished) {
          rfm12->in_cur_num_bytes++;  
          rfm12_update_rxtx_watchdog(rfm12, 0);
        } 
      } else {
        (void)rfm12_finish_receiving(rfm12, 1);
        packet_finished = 1;
      }
   
      if ((status & RF_STATUS_BIT_FFOV_RGUR) && !packet_finished) {
        rfm12->num_recv_overflows++;
        if (RFM12B_DROP_PACKET_ON_FFOV)
           (void)rfm12_finish_receiving(rfm12, 1);
      }
   }

   spin_unlock_irqrestore(&rfm12->lock, flags);
   
   if (!rfm12->should_release &&
        (!valid_interrupt ||
        (!packet_finished && (RFM12B_DROP_PACKET_ON_FFOV &&
         !(status & RF_STATUS_BIT_FFOV_RGUR)))))
         platform_irq_handled(rfm12->irq_identifier);
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

   rfm12_spi_completion_common(spi_msg);

   status = (spi_msg->spi_rx[0] << 8) | spi_msg->spi_rx[1];
   rfm12->last_status = status;

   valid_interrupt =
         ((status & RF_STATUS_BIT_FFIT_RGIT) || (status & RF_STATUS_BIT_FFOV_RGUR));

   if (valid_interrupt && NULL != rfm12->out_buf) {
      if (RFM12B_RETRY_SEND_ON_RGUR && (status & RF_STATUS_BIT_FFOV_RGUR)) {
         packet_finished = 1;
         rfm12->num_send_underruns++;
         (void)rfm12_finish_sending(rfm12, 0);
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
                 (void)rfm12_finish_sending(rfm12, 1);
                 break;
            default:
               // should never happen
               packet_finished = 1;
               (void)rfm12_finish_sending(rfm12, 0);
               break;
         }
         
         if (!packet_finished)
               rfm12_update_rxtx_watchdog(rfm12, 0);
         }
   }

   spin_unlock_irqrestore(&rfm12->lock, flags);

   if (!rfm12->should_release && (!valid_interrupt || !packet_finished))
         platform_irq_handled(rfm12->irq_identifier);
}

static int
rfm12_request_fifo_byte(struct rfm12_data* rfm12)
{
   uint16_t cmds[2];

   cmds[0] = RF_READ_STATUS;
   cmds[1] = RF_RX_FIFO_READ;

   return rfm12_send_generic_async_cmd(rfm12, cmds, 2,
     READ_FIFO_WAIT, rfm12_recv_spi_completion_handler,
     RFM12_STATE_NO_CHANGE);
}

static int
rfm12_write_tx_byte(struct rfm12_data* rfm12, u8 tx_byte)
{
   uint16_t cmds[2];
   
   cmds[0] = RF_READ_STATUS;
   cmds[1] = RF_TXREG_WRITE | tx_byte;
   
   return rfm12_send_generic_async_cmd(rfm12, cmds, 2,
      WRITE_TX_WAIT, rfm12_send_spi_completion_handler,
      RFM12_STATE_NO_CHANGE);
}

static void
rfm12_trysend_retry_timer_expired(unsigned long ptr)
{
   unsigned long flags;
   struct rfm12_data* rfm12 = (struct rfm12_data*)ptr;

   spin_lock_irqsave(&rfm12->lock, flags);

   if (rfm12->retry_sending_running)
        rfm12_try_sending(rfm12);

   rfm12->retry_sending_running = 0;
   
   spin_unlock_irqrestore(&rfm12->lock, flags);
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

   rfm12->retry_sending_running = 0;

   rfm12_spi_completion_common(spi_msg);

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
      
      rfm12_update_rxtx_watchdog(rfm12, 0);
            
      rfm12_send_generic_async_cmd(rfm12, cmd, 4,
            0, NULL, RFM12_STATE_NO_CHANGE);
   } else if (RFM12_STATE_SEND_PRE1 > rfm12->state) {
      // try again a bit later...
      init_timer(&rfm12->retry_sending_timer);
      rfm12->retry_sending_timer.expires = jiffies + TRYSEND_RETRY_JIFFIES;
      rfm12->retry_sending_timer.data = (unsigned long)rfm12;
      rfm12->retry_sending_timer.function = rfm12_trysend_retry_timer_expired;
      add_timer(&rfm12->retry_sending_timer);
      rfm12->retry_sending_running = 1;
   }
   
   rfm12->trysend = 0;
   spin_unlock_irqrestore(&rfm12->lock, flags);
}

// 0 ... nothing to send, can go to listen state
// 1 ... something needs sending, don't go to listen state
static int
rfm12_try_sending(struct rfm12_data* rfm12)
{   
   int retval = 0;
   
   if (!rfm12->trysend && NULL != rfm12->out_buf && rfm12->out_cur_end != rfm12->out_buf) {      
      uint16_t cmd = RF_READ_STATUS;
      
      (void)rfm12_send_generic_async_cmd(rfm12, &cmd, 1,
         0, rfm12_trysend_completion_handler, RFM12_STATE_NO_CHANGE);
      
      retval = rfm12->trysend = 1;
   }
         
   return retval;
}

static void
rfm12_begin_sending_or_receiving(struct rfm12_data* rfm12)
{      
   if (RFM12_STATE_IDLE == rfm12->state) {      
      if (!rfm12_try_sending(rfm12))
         rfm12_start_receiving(rfm12);
   }
}

static void
rfm12_stop_listening_and_send_callback(void *arg)
{
   unsigned long flags;
   struct rfm12_spi_message* spi_msg =
     (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
     (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->lock, flags);

   rfm12_spi_completion_common(spi_msg);
   
   rfm12_begin_sending_or_receiving(rfm12);
   
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
      
      err = rfm12_send_generic_async_cmd(rfm12, cmd, 2,
         0, rfm12_stop_listening_and_send_callback, RFM12_STATE_NO_CHANGE);
   }
   
   return err;
}

static void
rfm12_apply_crc16(struct rfm12_data* rfm12, unsigned char* ptr, unsigned len)
{
   u16 i, crc16 = ~0;
   if (0 != rfm12->group_id)
      crc16 = rfm12_crc16_update(~0, rfm12->group_id);
   for (i=0; i<len; i++)
      crc16 = rfm12_crc16_update(crc16, ptr[i]);
   
   // crc16
   ptr[len] = crc16 & 0xFF;
   ptr[len+1] = (crc16 >> 8) & 0xFF;
}

// this should only be called from an interrupt context
static void
rfm12_handle_interrupt(struct rfm12_data* rfm12)
{
   spin_lock(&rfm12->lock);
   
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
        (void)rfm12_write_tx_byte(rfm12, 0xAA);
        break;
     case RFM12_STATE_SEND_TAIL3: {
      uint16_t cmd = RF_IDLE_MODE;
        (void)rfm12_send_generic_async_cmd(rfm12, &cmd, 1, 0,
           NULL, RFM12_STATE_NO_CHANGE);
        (void)rfm12_write_tx_byte(rfm12, 0xAA);
        break;
     }
     case RFM12_STATE_SEND_SYN1:
         (void)rfm12_write_tx_byte(rfm12, 0x2D);
         break;
     case RFM12_STATE_SEND_SYN2:
         (void)rfm12_write_tx_byte(rfm12, rfm12->group_id);
         break;
     case RFM12_STATE_SEND:
        (void)rfm12_write_tx_byte(rfm12, *rfm12->out_buf_pos++);
        break;
     default: {
       uint16_t cmd = RF_READ_STATUS; 
       (void)rfm12_send_generic_async_cmd(rfm12, &cmd, 1,
          0, NULL, RFM12_STATE_NO_CHANGE);
       break;
     }
   }
   
   spin_unlock(&rfm12->lock);
}

/* File Operations ------------------------------------------- */

static ssize_t
rfm12_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos)
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
rfm12_write(struct file *filp, const char __user *buf,
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
      
   rfm12_apply_crc16(rfm12, rfm12->out_cur_end-2, copied+offset);

   rfm12->out_cur_end += copied + offset;

    if (RFM12_STATE_IDLE == rfm12->state) {
       rfm12_begin_sending_or_receiving(rfm12);
    } else if (RFM12_STATE_LISTEN == rfm12->state) {
       rfm12_stop_listening_and_send(rfm12);
    }

   spin_unlock_irqrestore(&rfm12->lock, flags);

   return copied;
}


static void
rfm12_ook_completion(void *context)
{
  struct spi_message *spi_msg = context;
  dev_info(&spi_msg->spi->dev, "ook: transfer completed. status = %d", spi_msg->status);
}

#define RFM12_LOW_POWER_MODE 0x9807
#define RFM12_HIGH_POWER_MODE 0x9800

static int
rfm12_send_ook(struct rfm12_data* rfm12, rfm12_ook_cmds *ook)
{
  int err;
  int i, j, k;
  unsigned long flags;
  static struct spi_transfer ook_transfers[RFM12B_OOK_CMD_MAX_LEN * 8  + 2];
  static struct spi_message ook_spi_msg;

  static u16 reg_vals[] = {
    htons(RFM12_LOW_POWER_MODE),
    htons(RFM12_HIGH_POWER_MODE),
    htons(RF_XMITTER_ON),
    htons(RF_IDLE_MODE),};

  spi_message_init(&ook_spi_msg);
  ook_spi_msg.context = &ook_spi_msg,
  ook_spi_msg.complete = rfm12_ook_completion;
  
  k = 0;
  memset(&ook_transfers[k], 0, sizeof(ook_transfers[0]));
  ook_transfers[k].tx_buf = &reg_vals[2];
  ook_transfers[k].len = sizeof(reg_vals[2]);
  ook_transfers[k].cs_change = 1;
  ook_transfers[k].delay_usecs = ook->delay_us;
  spi_message_add_tail(&ook_transfers[k], &ook_spi_msg);
  k++;

  for (i = 0; i < ook->len; i++) {
    u8 bits = ook->cmds[i];
    for (j = 0; j < 8; j++, bits <<= 1) {
      memset(&ook_transfers[k], 0, sizeof(ook_transfers[0]));
      ook_transfers[k].tx_buf = &reg_vals[bits & 0x80 ? 1 : 0];
      ook_transfers[k].len = sizeof(reg_vals[0]);
      ook_transfers[k].delay_usecs = ook->delay_us;
      ook_transfers[k].cs_change = 1;
      spi_message_add_tail(&ook_transfers[k], &ook_spi_msg);
      k++;
    }
  }
  memset(&ook_transfers[k], 0, sizeof(ook_transfers[0]));
  ook_transfers[k].tx_buf = &reg_vals[3];
  ook_transfers[k].len = sizeof(reg_vals[3]);
  ook_transfers[k].cs_change = 1;
  ook_transfers[k].delay_usecs = ook->delay_us;
  spi_message_add_tail(&ook_transfers[k], &ook_spi_msg);
  k++;

  spin_lock_irqsave(&rfm12->lock, flags);
  err = spi_async(rfm12->spi, &ook_spi_msg);
  spin_unlock_irqrestore(&rfm12->lock, flags);
  return err;
}

static unsigned int
rfm12_poll(struct file* filp, poll_table* wait)
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
rfm12_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
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
            rfm12_reset(rfm12);
            
            break;
      }
      
      case RFM12B_IOCTL_SET_BAND_ID: {
            int bid;
                        
            if (0 != copy_from_user(&bid, (int*)arg, sizeof(bid)))
                  return -EACCES;
                              
            rfm12->band_id = bid;
            rfm12_reset(rfm12);
            
            break;
      }
       
      case RFM12B_IOCTL_SET_BIT_RATE: {
            int br;
                        
            if (0 != copy_from_user(&br, (int*)arg, sizeof(br)))
                  return -EACCES;
            
            rfm12->bit_rate = br;
            rfm12_reset(rfm12);
            
            break;
      }
      
      case RFM12B_IOCTL_SET_JEE_ID: {
            int ji;
                        
            if (0 != copy_from_user(&ji, (int*)arg, sizeof(ji)))
                  return -EACCES;
            
            rfm12->jee_id = ji;
            rfm12_reset(rfm12);
            
            break;
      }
      
      case RFM12B_IOCTL_SET_JEEMODE_AUTOACK: {
         int ack;
         
         if (0 != copy_from_user(&ack, (int*)arg, sizeof(ack)))
           return -EACCES;
        
         rfm12->jee_autoack = (ack ? 1 : 0);
         rfm12_reset(rfm12);
         
         break;
      }

      case RFM12B_IOCTL_SEND_OOK: {
        rfm12_ook_cmds ook_cmd;
        if (0 != copy_from_user(&ook_cmd, (rfm12_ook_cmds*)arg, sizeof(rfm12_ook_cmds)))
           return -EACCES;
        
        rfm12_send_ook(rfm12, &ook_cmd);

        break;
      }

      default:
            return -EINVAL;
   }
   
   return 0;
}

static int
rfm12_open(struct inode *inode, struct file *filp)
{
   struct rfm12_data* rfm12;
    unsigned long flags;
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
          err = rfm12_setup(rfm12);
      
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
            
      if (0 == err)
         rfm12_begin_sending_or_receiving(rfm12);
      
      spin_unlock_irqrestore(&rfm12->lock, flags);
      
      err = platform_irq_init(rfm12->irq_identifier, (void*)rfm12);
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
rfm12_release(struct inode *inode, struct file *filp)
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
        rfm12_release_when_safe(rfm12);
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
   .write            = rfm12_write,
   .read             = rfm12_read,
   .unlocked_ioctl   = rfm12_ioctl,
   .poll             = rfm12_poll,
   .open             = rfm12_open,
   .release          = rfm12_release,
};

/* Device Probing ------------------------------------------ */

static struct class* rfm12_class;

static int __devinit
rfm12_probe(struct spi_device *spi)
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

      printk(KERN_INFO RFM12B_DRV_NAME
         ": added RFM12(B) transceiver %s.%d.%d\n",
       RFM12B_DEV_NAME, spi->master->bus_num, spi->chip_select);
   } else
      kfree(rfm12);

   return err;
}

static int __devexit
rfm12_remove(struct spi_device *spi)
{
   unsigned long flags;
   struct rfm12_data* rfm12 = spi_get_drvdata(spi);

   printk(KERN_INFO RFM12B_DRV_NAME
      ": removed RFM12(B) transceiver %s.%d.%d\n",
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
   .probe =    rfm12_probe,
   .remove =   __devexit_p(rfm12_remove),
};

/*** NEW CODE STARTS HERE ***/

int __init
rfm12_init_module(void)
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
rfm12_cleanup_module(void)
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

module_init(rfm12_init_module);
module_exit(rfm12_cleanup_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Georg Kaindl <gkaindl --AT-- mac.com>");
MODULE_DESCRIPTION("kernel driver for the rfm12b digital radio module");
MODULE_VERSION("0.0.1");

#endif // defined(MODULE_BOARD_CONFIGURED)
