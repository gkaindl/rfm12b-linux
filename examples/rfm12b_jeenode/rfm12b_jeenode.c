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

/*
   DOCUMENTATION:
   
   This is a slightly more involved example that mimics a chat client. Itt
   is very similar to rfm12b_chat.c, so you might also want to check its
   documentation.
   
   The difference to rfm12b_chat.c is that this example activates the
   Jee-compatible driver mode by assigning a Jee ID to the device. It also
   demonstrates how to decode Jee-specific protocol data in received packets
   and how to encode Jee-specific data in sent packets.
   
   Additionally, it uses the driver's ability to automatically send ACKs for
   Jee packets that request them, without having to do anything in userspace.
   This is achieved by enabling this feature via ioctl() (although it is on
   by default anyway).
   
   You can use this example with the RF12_demo Arduino sketch that ships as
   part of Jeelib or with another instance of itself – If the latter, make
   sure that the instances use different JEENODE_IDs.
*/

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/select.h>

#include "../common/common.h"
#include "../../rfm12b_config.h"
#include "../../rfm12b_ioctl.h"
#include "../../rfm12b_jeenode.h"

#define JEENODE_ID      12

#define RF12_MAX_RLEN   128
#define RF12_MAX_SLEN   66

#define SEND_COLOR      "\033[1;32m"
#define RECV_COLOR      "\033[1;31m"
#define STOP_COLOR      "\033[0m"

static volatile int running;

void sig_handler(int signum)
{
   signal(signum, SIG_IGN);
   running = 0;
}

int set_nonblock_fd(int fd)
{
   int opts;
   
   opts = fcntl(fd, F_GETFL);
   if (opts >= 0) {
   
      opts = (opts | O_NONBLOCK);
      if (fcntl(fd, F_SETFL,opts) >= 0) {
         return 0;
      }
   }
   
   return -1;
}

int main(int argc, char** argv)
{
   int rfm12_fd, len, i, nfds, ipos, jee_id = JEENODE_ID,
      band_id, group_id, bit_rate, ioctl_err, has_ack, has_ctl, is_dst,
      jee_addr, jee_len, send_ack;
   char* devname, ibuf[RF12_MAX_SLEN+1], obuf[RF12_MAX_RLEN+1], c;
   fd_set fds;
   
   if (set_nonblock_fd(STDIN_FILENO)) {
      printf("\nfailed to set non-blocking I/O on stdin: %s.\n\n",
         strerror(errno));
      return -1;
   }
   
   devname = RF12_TESTS_DEV;
   
   rfm12_fd = open(RF12_TESTS_DEV, O_RDWR);
   if (rfm12_fd < 0) {
      printf("\nfailed to open %s: %s.\n\n", devname, strerror(errno));
      return rfm12_fd;
   } else {
      if (set_nonblock_fd(rfm12_fd)) {
         printf("\nfailed to set non-blocking I/O on %s: %s.\n\n",
            devname, strerror(errno));
         return -1;
      }
   
      printf(
         "\nsuccessfully opened %s as fd %i.\n\n",
         devname, rfm12_fd
      );
   }

   fflush(stdout);
   signal(SIGINT, sig_handler);
   signal(SIGTERM, sig_handler);
   
   ioctl_err = 0;
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_GET_GROUP_ID, &group_id);
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_GET_BAND_ID, &band_id);
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_GET_BIT_RATE, &bit_rate);
   
   // we also want to send ACK packets automatically
   send_ack = 1;
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_SET_JEEMODE_AUTOACK, &send_ack);
   
   if (ioctl_err) {
      printf("\nioctl() error: %s.\n", strerror(errno));
      return -1;
   }
   
   // activate jeenode-compatible mode by giving this module a jeenode id
   if (ioctl(rfm12_fd, RFM12B_IOCTL_SET_JEE_ID, &jee_id)) {
      printf("\nioctl() error while setting jeenode id: %s.\n",
         strerror(errno));
      return -1;
   }

   printf("RFM12B configured to GROUP %i, BAND %i, BITRATE: 0x%.2x, JEE ID: %d\n\n",
      group_id, band_id, bit_rate, jee_id);
   
   printf("ready, type something to send it as broadcast + ACK request.\n\n");
   
   running = 1;
   ipos = 2;
   while (running) {
      FD_ZERO(&fds);
      FD_SET(STDIN_FILENO, &fds);
      FD_SET(rfm12_fd, &fds);
      
      nfds = select(rfm12_fd+1, &fds, NULL, NULL, NULL);
            
      if (nfds < 0 && running) {
         printf("\nan error happened during select: %s.\n\n",
            strerror(errno));
         return -1;
      } else if (nfds > 0) {
         // we ignore when select() returns 0, e.g. nothing is readable.
         if (FD_ISSET(STDIN_FILENO, &fds)) {
            // we can read from stdin, so read each available character until
            // we run out of space or get a \n, which means we should send
            
            len = read(STDIN_FILENO, &c, 1);
            if (len > 0) {
               if ('\n' == c) {
                  // in jeenode-compatible mode, we fill in the hdr and len
                  // fields manually and write them as part of the send buffer.
                  // what we do here is send a broadcast (DST is 0), but
                  // request an ACK from receivers. If they send an ACK, we
                  // will receive it, and you'll see the output on the console.
                  //
                  // note that the "len" byte will be "fixed" by the driver if
                  // you supply a length different from the amount of bytes you
                  // actually write().
                  ibuf[0] = jee_id | RFM12B_JEE_HDR_ACK_BIT;   // hdr
                  ibuf[1] = ipos-2;                              // len of payload
                  
                  len = write(rfm12_fd, ibuf, ipos);
                  
                  if (len < 0) {
                     printf("\nerror while sending: %s.\n\n",
                        strerror(errno));
                     return -1;
                  }
                  
                  ibuf[len] = '\0';
                  printf(SEND_COLOR "<SENT CTL:0 ACK:1 DST:0 ADDR:%d LEN:%d>" STOP_COLOR " %s\n",
                     jee_id, ipos-2, ibuf+2);
                  ipos = 2;
               } else if (ipos < RF12_MAX_SLEN)
                  ibuf[ipos++] = c;
            } else if (len < 0 && len != EWOULDBLOCK) {
               printf("\nerror while reading from stdin: %s.\n\n",
                  strerror(errno));
               return -1;
            }
         } else if (FD_ISSET(rfm12_fd, &fds)) {
            len = read(rfm12_fd, obuf, RF12_MAX_RLEN);
            
            if (len < 0) {
               printf("\nerror while receiving: %s.\n\n",
                  strerror(errno));
               return -1;
            } else if (len > 0) {
               // replace non-printable ASCII characters with a dot. the
               // first two bytes are preserved, though, since they are the
               // jeenode hdr and len bytes.
               for (i=2; i<len; i++)
                  if (' ' > obuf[i] || '~' < obuf[i])
                     obuf[i] = '.';
               
               obuf[len] = '\0';
               
               has_ack = (RFM12B_JEE_HDR_ACK_BIT & obuf[0]) ? 1 : 0;
               has_ctl = (RFM12B_JEE_HDR_CTL_BIT & obuf[0]) ? 1 : 0;
               is_dst = (RFM12B_JEE_HDR_DST_BIT & obuf[0]) ? 1 : 0;
               jee_addr = RFM12B_JEE_ID_FROM_HDR(obuf[0]);
               jee_len = obuf[1];
               
               printf(RECV_COLOR "<RECV CTL:%d ACK:%d DST:%d ADDR:%d LEN:%d>" STOP_COLOR " %s\n",
                  has_ctl, has_ack, is_dst, jee_addr, jee_len, &obuf[2]);
               
               // in jeenode-compatible mode, the driver sends ACK packets automatically by default,
               // so we're just writing out here what the driver does internally. you do
               // not need to manually send an ACK if one is requested, though!
               if ((obuf[0] & RFM12B_JEE_HDR_ACK_BIT) && !(obuf[0] & RFM12B_JEE_HDR_CTL_BIT)) {
                  if (obuf[0] & RFM12B_JEE_HDR_DST_BIT) {
                     // if this was only sent to us, an ACK is sent as broadcast
                     printf(SEND_COLOR "<SENT CTL:1 ACK:0 DST:0 ADDR:%d LEN:0>\n" STOP_COLOR,
                        jee_id);
                  } else {
                     // if this was a broadcast, the ACK is sent directly to the source node.
                     printf(SEND_COLOR "<SENT CTL:1 ACK:0 DST:1 ADDR:%d LEN:0>\n" STOP_COLOR,
                        RFM12B_JEE_ID_FROM_HDR(obuf[0]));
                  }
               }
            }
         }
      }
   }
   
   printf("\n");
   print_stats(rfm12_fd);
   
   close(rfm12_fd);
      
   return 0;
}
