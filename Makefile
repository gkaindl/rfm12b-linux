# rfm12b-linux: linux kernel driver for the rfm12(b) RF module by HopeRF
# Copyright (C) 2013 Georg Kaindl
# 
# This file is part of rfm12b-linux.
# 
# rfm12b-linux is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
# 
# rfm12b-linux is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with rfm12b-linux.  If not, see <http://www.gnu.org/licenses/>.
#

obj-m += rfm12b.o

KVERSION := $(shell uname -r)

# 3.7 moved version.h to a different location
if [ -f /lib/modules/$(KVERSION)/build/include/generated/uapi/linux/version.h ]; then \
	INCLUDE += -I/lib/modules/$(KVERSION)/build/include/generated/uapi/

all:
	make -C /lib/modules/$(KVERSION)/build $(INCLUDE) M=$(PWD) modules
clean:
	make -C /lib/modules/$(KVERSION)/build $(INCLUDE) M=$(PWD) clean

