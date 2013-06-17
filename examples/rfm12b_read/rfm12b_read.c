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
   
   This is a trivial example that will open the rfm12b device and
   continously read() from it in an endless loop. The individual
   bytes of the received packet will then be written to stdout.
   
   On the receiving side, the driver will block on read() until a
   packet is received. A subsequent read will then return the entire
   packet, provided your userspace buffer is large enough (if your
   buffer is too small, the rest of the bytes in the packet will be
   lost forever). Since the maximum packet length is 66 bytes (to
   ensure compatibility with the JeeNode Arduino library), you should
   provide a buffer at least of this size. The return value of read()
   behaves as expected, and if it's a positive number, it's the length
   of the packet that has been received.
   
   Press Ctrl+C (e.g. send a SIGINT) to quit the program. It will
   then print the driver statistics.
   
   You can use this example as a receiver to the rfm12b_write.c example
   or as receiver for the rfm12b_send Arduino example. It's also useful
   to test or debug your own sending code.
*/

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <signal.h>
#include <errno.h>

#include "../common/common.h"
#include "../../rfm12b_config.h"
#include "../../rfm12b_ioctl.h"

#define RF12_BUF_LEN	128

static volatile int running;

void sig_handler(int signum)
{
	signal(signum, SIG_IGN);
	running = 0;
}

int main(int argc, char** argv)
{
	int fd, len, i;
	char* devname, buf[128];
	unsigned long pkt_cnt;
	time_t tt;
	
	devname = RF12_TESTS_DEV;
	
	fd = open(RF12_TESTS_DEV, O_RDWR);
	if (fd < 0) {
		printf("\nfailed to open %s: %s.\n\n", devname, strerror(errno));
		return fd;
	} else
		printf(
			"\nsuccessfully opened %s as fd %i, entering read loop...\n\n",
			devname, fd
		);

	fflush(stdout);
	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);
	
	pkt_cnt = 0;
	running = 1;
	do {		
		len = read(fd, buf, RF12_BUF_LEN);
		
		time (&tt);
		
		if (len > 0) {
			printf("%s", ctime(&tt));
			printf("\t%i bytes read\n\t\t", len);
			
			for (i=0; i<len; i++) {
				printf("%d ", buf[i]);
			}
			printf("\n");
			
			fflush(stdout);	
	
			pkt_cnt++;
		} else if (len < 0) {
			break;
		}		
	} while (running);
	
	print_stats(fd);
	
	close(fd);
	
	printf("\n\n%lu packet(s) received.\n\n", pkt_cnt);
	
	return len;
}
