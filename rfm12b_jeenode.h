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
   This file includes a couple of macros useful when using the driver
   in Jee-compatible mode (see the docs for RFM12B_DEFAULT_JEE_ID
   in rfm12b_config.h).
*/

#if !defined(__RFM12B_JEENODE_H__)
#define __RFM12B_JEENODE_H__

/*
   Bit masks to extract Jee-protocol flags from header
*/
#define RFM12B_JEE_HDR_CTL_BIT         0x80
#define RFM12B_JEE_HDR_DST_BIT         0x40
#define RFM12B_JEE_HDR_ACK_BIT         0x20

/*
   Macro to extract a Jeenode identifier from a header
*/
#define RFM12B_JEE_ID_FROM_HDR(hdr)    (hdr & 0x1f)

#endif // __RFM12B_JEENODE_H__