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

#include <fcntl.h>
#include <stdio.h>

#include "common.h"
#include "../../rfm12b_ioctl.h"

void print_stats(int fd)
{
   rfm12b_stats s;
   
   if (0 == ioctl(fd, RFM12B_IOCTL_GET_STATS, &s)) {
      printf("ioctl() succeeded.\n");
   
      printf(
         "\tbytes_recvd: %lu\n"
         "\tpkts_recvd: %lu\n"
         "\tbytes_sent: %lu\n"
         "\tpkts_sent: %lu\n"
         "\tnum_recv_overflows: %lu\n"
         "\tnum_recv_timeouts: %lu\n"
         "\tnum_recv_crc16_fail: %lu\n"
         "\tnum_send_underruns: %lu\n"
         "\tnum_send_timeouts: %lu\n"
         "\tlow_battery: %lu\n",
         s.bytes_recvd, s.pkts_recvd, s.bytes_sent, s.pkts_sent,
         s.num_recv_overflows, s.num_recv_timeouts, s.num_recv_crc16_fail,
         s.num_send_underruns, s.num_send_timeouts,
         s.low_battery
      );
   } else
      printf("ioctl() failed.\n");
}
