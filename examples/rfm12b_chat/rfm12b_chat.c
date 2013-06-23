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
   
   This is a slightly more involved example that mimics a chat-like
   client, which will print received packets (as strings) and allows
   you to send strings by typing them in.
   
   It also demonstrates how to use ioctl() to change driver settings
   for the opened device and how to use select() to implement driver I/O
   that is not based on blocking read()s.
   
   It is most fun to use this example with another instance of itself
   running on a second board, but you can also use it together with the
   rfm12b_echo.c example.
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
   int rfm12_fd, len, i, nfds, ipos, band_id, group_id, bit_rate, ioctl_err;
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
   
   // this demonstrates how to use ioctl() to read and write config data
   ioctl_err = 0;
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_GET_GROUP_ID, &group_id);
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_GET_BAND_ID, &band_id);
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_GET_BIT_RATE, &bit_rate);
   
   // and this is how to reconfigure via ioctl()... we simply write the
   // same data back that we read...
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_SET_GROUP_ID, &group_id);
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_SET_BAND_ID, &band_id);
   ioctl_err |= ioctl(rfm12_fd, RFM12B_IOCTL_SET_BIT_RATE, &bit_rate);
   
   if (0 != ioctl_err) {
      printf("\nerror during ioctl(): %s.", strerror(errno));
      return -1;
   }
   
   printf("RFM12B configured to GROUP %i, BAND %i, BITRATE: 0x%.2x.\n\n",
      group_id, band_id, bit_rate);
   
   printf("ready, type something to send it.\n\n");
   
   running = 1;
   ipos = 0;
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
                  len = write(rfm12_fd, ibuf, ipos);
                  
                  if (len < 0) {
                     printf("\nerror while sending: %s.\n\n",
                        strerror(errno));
                     return -1;
                  }
                  
                  ibuf[len] = '\0';
                  printf(SEND_COLOR "<SENT>" STOP_COLOR " %s\n", ibuf);
                  ipos = 0;
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
               // replace non-printable ASCII characters with a dot.
               for (i=0; i<len; i++)
                  if (' ' > obuf[i] || '~' < obuf[i])
                     obuf[i] = '.';
               
               obuf[len] = '\0';
               printf(RECV_COLOR "<RECV>" STOP_COLOR " %s\n", obuf);
            }
         }
      }
   }
   
   printf("\n");
   print_stats(rfm12_fd);
   
   close(rfm12_fd);
      
   return 0;
}
