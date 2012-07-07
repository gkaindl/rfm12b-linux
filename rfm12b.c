#include <linux/module.h>
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


#define RFM12B_DRV_NAME		"rfm12"

#define RFM12B_SPI_MAX_HZ	2500000
#define RFM12B_SPI_MODE		0
#define RFM12B_SPI_BITS		8

#define  RFM12_NAME        "rfm12"
#define  RFM12_SPI_MAJOR      154
#define  RFM12_N_SPI_MINORS   32

#define RF_READ_STATUS     0x0000
#define RF_IDLE_MODE       0x820D
#define RF_SLEEP_MODE      0x8205
#define RF_TXREG_WRITE     0xB800
#define RF_RECEIVER_ON     0x82DD
#define RF_XMITTER_ON      0x823D
#define RF_RX_FIFO_READ    0xB000

#define RF_STATUS_BIT_FFEM (0x0200)
#define RF_STATUS_BIT_FFOV (0x2000)

#define RF_MAX_DATA_LEN    66
#define RF_EXTRA_LEN	   4
#define RF_MAX_LEN		   (RF_MAX_DATA_LEN+RF_EXTRA_LEN)

#define OPEN_WAIT_MILLIS   (50)

#define READ_FIFO_WAIT              (0)
#define FINISH_RECV_WAIT            (0)

#define RXTX_WATCHDOG_JIFFIES       (HZ/10)

#define DATA_BUF_SIZE            (512)
#define NUM_MAX_CONCURRENT_MSG   (1)

static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static DECLARE_BITMAP(minors, RFM12_N_SPI_MINORS);

DECLARE_WAIT_QUEUE_HEAD(rfm12_wait_read);
DECLARE_WAIT_QUEUE_HEAD(rfm12_wait_write);

typedef enum _rfm12_state_t {
   RFM12_STATE_NO_CHANGE,
   RFM12_STATE_CONFIG,
   RFM12_STATE_SLEEP,
   RFM12_STATE_RECV,
   RFM12_STATE_RECV_FINISH,
   RFM12_STATE_SEND
} rfm12_state_t;

struct rfm12_spi_message {
   struct spi_message   spi_msg;
   struct spi_transfer  spi_transfers[2];
   rfm12_state_t        spi_finish_state;
   u8                   spi_tx[4], spi_rx[4];
   void*                context;
   u8                   pos;
};

struct rfm12_data {
	u16					irq;

	dev_t				devt;
	spinlock_t		    rfm12_lock;
	struct spi_device*	spi;
	struct list_head	device_entry;

	u8                   open;
	rfm12_state_t        state;
	u8					 group_id, band_id;
	unsigned long        bytes_recvd;
	unsigned long        bytes_sent;
	unsigned long        num_overflows, num_crc16_fail;
	u8*                  in_buf, *in_buf_pos;
	u8*                  out_buf, *out_buf_pos;
	u8*                  in_cur_len_pos, *out_cur_len_pos;
	u8*                  in_cur_end, *out_cur_end;
	u16					 crc16;
	int                  in_cur_num_bytes, out_cur_num_bytes;
	struct rfm12_spi_message spi_msgs[NUM_MAX_CONCURRENT_MSG];
	u8                   free_spi_msgs;
	struct timer_list    rxtx_watchdog;
	u8                   rxtx_watchdog_running;
};

// forward declarations
static void
rfm12_handle_interrupt(struct rfm12_data* rfm12);
static int
rfm12_request_fifo_byte(struct rfm12_data* rfm12);
static int
rfm12_finish_receiving(struct rfm12_data* rfm12, int skip_packet);

#include "platform/platform.h"
#include "platform/plat_am33xx.h"

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
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg;
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   __rfm12_generic_spi_completion_handler(arg);

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
}

static int
rfm12_send_generic_async_cmd(struct rfm12_data* rfm12, uint16_t* cmds,
								 int num_cmds, uint16_t delay_usecs,
								 void (*callback)(void*),
								 rfm12_state_t finish_state)
{
   int err;
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

   if (num_cmds > 1) {
	  spi_msg->spi_transfers[0].cs_change = 1;
	  spi_msg->spi_transfers[0].delay_usecs = delay_usecs;
	  spi_message_add_tail(&spi_msg->spi_transfers[0], &spi_msg->spi_msg);

	  spi_msg->spi_transfers[1] =
		   rfm12_control_spi_transfer(spi_msg, 1, cmds[1]);
	   spi_message_add_tail(&spi_msg->spi_transfers[1], &spi_msg->spi_msg);
   } else
	  spi_message_add_tail(&spi_msg->spi_transfers[0], &spi_msg->spi_msg);

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
   uint16_t cmd = RF_RECEIVER_ON;
   int err = rfm12_send_generic_async_cmd(rfm12, &cmd, 1, 0,
	  NULL, RFM12_STATE_RECV);
   if (0 == err) {
	  rfm12->in_cur_num_bytes = 0;
   }

   return err;
}

static int
rfm12_start_sending(struct rfm12_data* rfm12)
{
   uint16_t cmd = RF_XMITTER_ON;
   return rfm12_send_generic_async_cmd(rfm12, &cmd, 1, 0,
	  NULL, RFM12_STATE_SEND);
}

static int
rfm12_start_sleeping(struct rfm12_data* rfm12)
{
   uint16_t cmd = RF_SLEEP_MODE;
   return rfm12_send_generic_async_cmd(rfm12, &cmd, 1, 0,
	  NULL, RFM12_STATE_SLEEP);
}

static int
rfm12_setup(struct rfm12_data* rfm12)
{
   struct spi_transfer tr, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9,
	  tr10, tr11, tr12, tr13;
   struct spi_message msg;
   u8 tx_buf[26];
   int err;

	// TODO: make me configurable
	rfm12->group_id = 211;
	rfm12->band_id = 2;

   rfm12->state = RFM12_STATE_CONFIG;

   spi_message_init(&msg);

   tr = rfm12_make_spi_transfer(RF_READ_STATUS, tx_buf+0, NULL);
   tr.cs_change = 1;
   spi_message_add_tail(&tr, &msg);

   tr2 = rfm12_make_spi_transfer(RF_SLEEP_MODE, tx_buf+2, NULL);
   tr2.cs_change = 1;
   spi_message_add_tail(&tr2, &msg);

   tr3 = rfm12_make_spi_transfer(RF_TXREG_WRITE, tx_buf+4, NULL);
   spi_message_add_tail(&tr3, &msg);

   err = spi_sync(rfm12->spi, &msg);

   if (err)
	  goto pError;

   msleep(OPEN_WAIT_MILLIS);

   // ok, we're now ready to be configured.
   spi_message_init(&msg);

   tr = rfm12_make_spi_transfer(0x80C7 |
   		((rfm12->band_id & 0xff) << 4), tx_buf+0, NULL);
   tr.cs_change = 1;
   spi_message_add_tail(&tr, &msg);

   tr2 = rfm12_make_spi_transfer(0xA640, tx_buf+2, NULL);
   tr2.cs_change = 1;
   spi_message_add_tail(&tr2, &msg);

   tr3 = rfm12_make_spi_transfer(0xC606, tx_buf+4, NULL);
   //tr3 = rfm12_make_spi_transfer(0xC623, tx_buf+4, NULL);
   tr3.cs_change = 1;
   spi_message_add_tail(&tr3, &msg);

   tr4 = rfm12_make_spi_transfer(0x94A2, tx_buf+6, NULL);
   tr4.cs_change = 1;
   spi_message_add_tail(&tr4, &msg);

   tr5 = rfm12_make_spi_transfer(0xC2AC, tx_buf+8, NULL);
   tr5.cs_change = 1;
   spi_message_add_tail(&tr5, &msg);

   if (0 != rfm12->group_id) {
	  tr6 = rfm12_make_spi_transfer(0xCA83, tx_buf+10, NULL);
	  tr6.cs_change = 1;
	  spi_message_add_tail(&tr6, &msg);

	  tr7 = rfm12_make_spi_transfer(0xCE00 |
	  	rfm12->group_id, tx_buf+12, NULL);
	  tr7.cs_change = 1;
	  spi_message_add_tail(&tr7, &msg);
   } else {
	  tr6 = rfm12_make_spi_transfer(0xCA8B, tx_buf+10, NULL);
	  tr6.cs_change = 1;
	  spi_message_add_tail(&tr6, &msg);

	  tr7 = rfm12_make_spi_transfer(0xCE2D, tx_buf+12, NULL);
	  tr7.cs_change = 1;
	  spi_message_add_tail(&tr7, &msg);
   }

   tr8 = rfm12_make_spi_transfer(0xC483, tx_buf+14, NULL);
   tr8.cs_change = 1;
   spi_message_add_tail(&tr8, &msg);

   tr9 = rfm12_make_spi_transfer(0x9850, tx_buf+16, NULL);
   tr9.cs_change = 1;
   spi_message_add_tail(&tr9, &msg);

   tr10 = rfm12_make_spi_transfer(0xCC77, tx_buf+18, NULL);
   tr10.cs_change = 1;
   spi_message_add_tail(&tr10, &msg);

   tr11 = rfm12_make_spi_transfer(0xE000, tx_buf+20, NULL);
   tr11.cs_change = 1;
   spi_message_add_tail(&tr11, &msg);

   tr12 = rfm12_make_spi_transfer(0xC800, tx_buf+22, NULL);
   tr12.cs_change = 1;
   spi_message_add_tail(&tr12, &msg);

   tr13 = rfm12_make_spi_transfer(0xC049, tx_buf+24, NULL);
   spi_message_add_tail(&tr13, &msg);

   err = spi_sync(rfm12->spi, &msg);

   if (0 == err) {
	  spi_message_init(&msg);

	  tr = rfm12_make_spi_transfer(RF_READ_STATUS, tx_buf+0, NULL);
	  spi_message_add_tail(&tr, &msg);

	  err = spi_sync(rfm12->spi, &msg);
   }

pError:

   return err;
}

static void
rfm12_rxtx_watchdog_expired(unsigned long ptr)
{
   unsigned long flags;
   struct rfm12_data* rfm12 = (struct rfm12_data*)ptr;

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   if (RFM12_STATE_RECV == rfm12->state) {
	  rfm12->num_overflows++;
	  (void)rfm12_finish_receiving(rfm12, 1);
   }

   rfm12->rxtx_watchdog_running = 0;
   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
}

static void
rfm12_update_rxtx_watchdog(struct rfm12_data* rfm12, u8 cancelTimer)
{
   if (rfm12->rxtx_watchdog_running) {
	  if (cancelTimer) {
		 del_timer_sync(&rfm12->rxtx_watchdog);
		 rfm12->rxtx_watchdog_running = 0;
	  } else
		 mod_timer(&rfm12->rxtx_watchdog,
			jiffies + RXTX_WATCHDOG_JIFFIES);
   } else {
	  init_timer(&rfm12->rxtx_watchdog);
	  rfm12->rxtx_watchdog.expires = jiffies + RXTX_WATCHDOG_JIFFIES;
	  rfm12->rxtx_watchdog.data = (unsigned long)rfm12;
	  rfm12->rxtx_watchdog.function = rfm12_rxtx_watchdog_expired;
	  add_timer(&rfm12->rxtx_watchdog);
	  rfm12->rxtx_watchdog_running = 1;
   }
}

static void
rfm12_finish_recv_callback(void *arg)
{
   unsigned long flags;
   uint16_t cmds[2];
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   rfm12_spi_completion_common(spi_msg);

   cmds[0] = RF_IDLE_MODE;
   cmds[1] = RF_RECEIVER_ON;

   (void)rfm12_send_generic_async_cmd(rfm12, cmds, 2,
	  FINISH_RECV_WAIT, NULL, RFM12_STATE_RECV);   

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
   
   platform_irq_handled((void*)rfm12);
}

static int
rfm12_finish_receiving(struct rfm12_data* rfm12, int skip_packet)
{
   uint16_t cmds[2];
   int err = 0;

   if (RFM12_STATE_RECV == rfm12->state) {
	  rfm12->state = RFM12_STATE_RECV_FINISH;

	  rfm12->in_cur_num_bytes = 0;
	  rfm12_update_rxtx_watchdog(rfm12, 1);

	  // TODO: check if this is for our node or broadcast.
	  // TODO: check if we are in "promiscuous mode".
	  if (0 == skip_packet && 0 == rfm12->crc16) {
		 rfm12->in_cur_end = rfm12->in_buf_pos;
		 wake_up_interruptible(&rfm12_wait_read);
	  } else {
		 if (0 == skip_packet && 0 != rfm12->crc16)
		 	rfm12->num_crc16_fail++;
		  
		 rfm12->in_buf_pos = rfm12->in_cur_end;
	  }

	  cmds[0] = RF_READ_STATUS;
	  cmds[1] = RF_IDLE_MODE;

	  err = rfm12_send_generic_async_cmd(rfm12, cmds, 2,
			   0, rfm12_finish_recv_callback, RFM12_STATE_NO_CHANGE);
   }

   return err;
}

static void
rfm12_recv_spi_completion_handler(void *arg)
{
   u8* recv_data;
   unsigned long flags;
   uint16_t status, packet_finished = 0;
   struct rfm12_spi_message* spi_msg =
	  (struct rfm12_spi_message*)arg; 
   struct rfm12_data* rfm12 =
	  (struct rfm12_data*)spi_msg->context;

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   rfm12_spi_completion_common(spi_msg);

   recv_data = spi_msg->spi_rx;

   if (NULL != rfm12->in_buf) {
	  if (rfm12->in_buf_pos < (rfm12->in_buf + DATA_BUF_SIZE)) {
		 *rfm12->in_buf_pos++ = recv_data[1];
		 
	  	 if (0 == rfm12->in_cur_num_bytes)
	  	 	 rfm12->crc16 = rfm12_crc16_update(~0, rfm12->group_id);
	  	 
	  	 rfm12->crc16 = rfm12_crc16_update(rfm12->crc16, recv_data[1]);
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

   status = (recv_data[2] << 8) | recv_data[3];

   if ((status & RF_STATUS_BIT_FFOV) && !packet_finished) {
	  rfm12->num_overflows++;
	  (void)rfm12_finish_receiving(rfm12, 1);
   }

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
   
   if (!packet_finished && !(status & RF_STATUS_BIT_FFOV))
   	   platform_irq_handled((void*)rfm12);
}

static int
rfm12_request_fifo_byte(struct rfm12_data* rfm12)
{
   uint16_t cmds[2];

   cmds[0] = RF_RX_FIFO_READ;
   cmds[1] = RF_READ_STATUS;

   return rfm12_send_generic_async_cmd(rfm12, cmds, 2,
	  READ_FIFO_WAIT, rfm12_recv_spi_completion_handler,
	  RFM12_STATE_NO_CHANGE);   
}

// this should only be called from an interrupt context
static void
rfm12_handle_interrupt(struct rfm12_data* rfm12)
{   
   spin_lock(&rfm12->rfm12_lock);

   switch (rfm12->state) {
	  case RFM12_STATE_RECV:
		 (void)rfm12_request_fifo_byte(rfm12);
		 break;

	  default:
		 break;
   }

   spin_unlock(&rfm12->rfm12_lock);
}

/* File Operations ------------------------------------------- */

static ssize_t
rfm12_read(struct file* filp, char __user *buf, size_t count, loff_t* f_pos)
{
   struct rfm12_data* rfm12 = (struct rfm12_data*)filp->private_data;
   int length = 0, bytes_to_copy = 0, mmovelen = 0;
   unsigned long flags;

   if (rfm12->in_cur_end == rfm12->in_buf)
	  wait_event_interruptible(rfm12_wait_read,
			(rfm12->in_cur_end > rfm12->in_buf));

   if (rfm12->in_cur_end <= rfm12->in_buf)
	  return 0;

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   length = *(rfm12->in_buf + 1) + 2;
   bytes_to_copy = (count > length) ? length : count;

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);

   bytes_to_copy -= copy_to_user(buf, rfm12->in_buf, bytes_to_copy);

   spin_lock_irqsave(&rfm12->rfm12_lock, flags);

   mmovelen = length + 2;
   if (mmovelen > 0)
	  memmove(rfm12->in_buf,
		 rfm12->in_buf + mmovelen,
		 mmovelen);

   rfm12->in_cur_end -= length + 2;
   rfm12->in_buf_pos -= length + 2;

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);

   return bytes_to_copy;
}

// TODO: this needs to be rewritten to allow only one packet to be written at once!
static ssize_t
rfm12_write(struct file *filp, const char __user *buf,
size_t count, loff_t *f_pos)
{
	struct rfm12_data* rfm12 = (struct rfm12_data*)filp->private_data;
	int bytes_to_copy = 0, cpos = 0, copied = 0;
	unsigned long flags, space_needed, total_bytes = 0;
	
	space_needed = (count / (RF_MAX_DATA_LEN-RF_EXTRA_LEN)) * RF_EXTRA_LEN +
					count +
					(count % (RF_MAX_DATA_LEN-RF_EXTRA_LEN) > 0) * RF_EXTRA_LEN;
	
	// TODO: adapt this for sending
	/* if (rfm12->in_cur_end == rfm12->in_buf)
		wait_event_interruptible(rfm12_wait_write,
			DATA_BUF_SIZE - (rfm12->out_cur_end - rfm12->out_buf)
				> space_needed); */
		
	spin_lock_irqsave(&rfm12->rfm12_lock, flags);
	
	cpos = 0;
	do {
		bytes_to_copy =
			(count > RF_MAX_DATA_LEN-4 ? RF_MAX_DATA_LEN-4 : count);
			
			spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
			
			copied = copy_from_user(
				rfm12->out_cur_end + 2,
				&buf[cpos],
				bytes_to_copy
			);
			
			spin_lock_irqsave(&rfm12->rfm12_lock, flags);
			
			// TODO: fill in header
			// TODO: setup crc16

			*rfm12->out_cur_end = bytes_to_copy + 3;
			
			rfm12->out_cur_end += copied + 4;
			
			cpos += bytes_to_copy;
			count -= bytes_to_copy;
			total_bytes += copied;
	} while (count > 0);
	
	spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
		
	return total_bytes;
}

static unsigned int
rfm12_poll(struct file* filp, poll_table* wait)
{
   return 0;
}

static long
rfm12_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
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
	   if (0 != rfm12->open) {
		 err = -EBUSY;
		 goto pError;
	   }

	  spin_lock_irqsave(&rfm12->rfm12_lock, flags);

	  if (0 == err)
		 err = rfm12_setup(rfm12);

	  if (err) {
		 spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
		 goto pError;
	  }

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
		 rfm12->bytes_sent = 0;
		 rfm12->num_overflows = 0;
		 rfm12->num_crc16_fail = 0;
		 rfm12->in_cur_num_bytes = 0;
		 rfm12->rxtx_watchdog_running = 0;
		 rfm12->in_cur_len_pos = rfm12->in_buf;
		 rfm12->in_cur_end = rfm12->in_buf;
	  }

	  spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);

	  if (0 == err)
		 err = rfm12_start_receiving(rfm12);

	  err = platform_irq_init((void*)rfm12);
	  has_irq = (0 == err);
	} else
		pr_debug("rfm12: nothing for minor %d\n", iminor(inode));

pError:

   if (err && has_irq)
	  platform_irq_cleanup((void*)rfm12);

	mutex_unlock(&device_list_lock);
	return err;
}

static int
rfm12_release(struct inode *inode, struct file *filp)
{
	struct rfm12_data* rfm12;
	int err = 0;
   unsigned long flags;

	mutex_lock(&device_list_lock);

	rfm12 = filp->private_data;
	filp->private_data = NULL;

	rfm12->open--;
	if (0 == rfm12->open) {
		int dofree = 0;

	  platform_irq_cleanup((void*)rfm12);

	  spin_lock_irqsave(&rfm12->rfm12_lock, flags);

	  rfm12_update_rxtx_watchdog(rfm12, 1);

	  rfm12_start_sleeping(rfm12);

		kfree(rfm12->in_buf);
		rfm12->in_buf = rfm12->in_buf_pos = NULL;
	  rfm12->out_buf = rfm12->out_buf_pos = NULL;

	  dofree = (rfm12->spi == NULL);

		if (dofree) {
			kfree(rfm12);
		}

		spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);
	}
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

   rfm12->state = RFM12_STATE_CONFIG;
	rfm12->spi = spi;
   rfm12->open = 0;
   rfm12->free_spi_msgs = 0;
	spin_lock_init(&rfm12->rfm12_lock);

	INIT_LIST_HEAD(&rfm12->device_entry);

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, RFM12_N_SPI_MINORS);
	if (minor < RFM12_N_SPI_MINORS) {
		struct device *dev;

		rfm12->devt = MKDEV(RFM12_SPI_MAJOR, minor);
		dev = device_create(rfm12_class, &spi->dev, rfm12->devt,
					rfm12, RFM12_NAME ".%d.%d",
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
		 RFM12_NAME, spi->master->bus_num, spi->chip_select);
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
	  RFM12_NAME, spi->master->bus_num, spi->chip_select);

	spin_lock_irqsave(&rfm12->rfm12_lock, flags);

	rfm12->spi = NULL;
	spi_set_drvdata(spi, NULL);

   spin_unlock_irqrestore(&rfm12->rfm12_lock, flags);

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
		.owner =	THIS_MODULE,
	},
	.probe =    rfm12_probe,
	.remove =	__devexit_p(rfm12_remove),
};

/*** NEW CODE STARTS HERE ***/

int __init
rfm12_init_module(void)
{
	int err;

	err = platform_module_init();
	if (err)
		goto errReturn;

	err = register_chrdev(RFM12_SPI_MAJOR, "spi", &rfm12_fops);
	if (err < 0) {
		platform_module_cleanup();
		goto errReturn;
	}

	rfm12_class = class_create(THIS_MODULE, RFM12_NAME);
	if (IS_ERR(rfm12_class)) {
		unregister_chrdev(RFM12_SPI_MAJOR, rfm12_spi_driver.driver.name);
		platform_module_cleanup();
		err = PTR_ERR(rfm12_class);
		goto errReturn;
	}

	err = spi_register_driver(&rfm12_spi_driver);
	if (err < 0) {
		class_destroy(rfm12_class);
		unregister_chrdev(RFM12_SPI_MAJOR, rfm12_spi_driver.driver.name);
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
	unregister_chrdev(RFM12_SPI_MAJOR, rfm12_spi_driver.driver.name);

	(void)platform_module_cleanup();

	printk(
		KERN_INFO RFM12B_DRV_NAME
		" : driver unloaded.\n"
	);
}

module_init(rfm12_init_module);
module_exit(rfm12_cleanup_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Georg Kaindl <gkaindl (AT) mac.com>");
MODULE_DESCRIPTION("kernel driver for rfm12b digital radio module");
MODULE_VERSION("0.0.1");
