# rfm12b-linux

A Linux kernel driver for the [RFM12B](http://www.hoperf.com/rf_fsk/fsk/21.htm) digital RF module manufactured by HopeRF. It targets embedded Linux boards that provide a SPI interface.

Currently, Raspberry Pi, Beaglebone and Beaglebone Black are supported, but it should be easy to support additional boards by adding the necessary setup code (e.g. setting up pinmuxing, etc…).

The driver can be used to communicate with [JeeLib](https://github.com/jcw/jeelib) on the Arduino platform. Its interaction with the RFM12B board is based heavily on the fantastic work in JeeLib!

A demo video is available on [YouTube](http://www.youtube.com/watch?v=5aeSIBstJS0).

## Overall Functionality

The driver does enables you to connect an RFM12B module directly to a Linux board without requiring additional parts like an external MCU.

The RFM12B module will be exported as a character device in /dev/, from which packets can be read or to which packets can be written. Multiple bytes written in a single call to write() will be sent as a single packet, while a call to read() will return all the bytes of a packet in the receive buffer.

The module provides various settings that can be specified at compile-time, via module-load parameters or via ioctl().

## Hardware Setup

You will only need some wires to connect the RFM12B module to your Linux board.

The RFM12B data sheet with a visual description of the pinout is available at [http://www.hoperf.com/upload/rf/RFM12B.pdf](http://www.hoperf.com/upload/rf/RFM12B.pdf). For the pinouts of the Linux boards, either check the silkscreen or their respective technical manuals.

Be sure to double-check that your connections are correct, since wrong connections might harm your board or RFM12B module.

Also, be sure to add an antenna to your RFM12B board (ANT pin), or the range and reliability will be severely hampered.

**Raspberry Pi**

    Raspberry Pi          RFM12B
    ---------------------------------
    P1/17                 VDD (+3.3V)
    P1/19                 SDI
    P1/21                 SDO
    P1/22                 nIRQ
    P1/23                 SCK
    P1/25                 GND
    P1/26                 nSEL
    

**Beaglebone & Beaglebone Black**

    Beaglebone            RFM12B
    ---------------------------------
    P9/1                  GND
    P9/3                  VDD (+3.3V)
    P9/27                 nIRQ
    P9/29                 SDO
    P9/30                 SDI
    P9/31                 SCK
    P9/42                 nSEL

## Installation

Generally, you only need to clone the repository and run `make` to build the kernel module, then `insmod rfm12b.ko` to insert it into the kernel.

I've included more detailed steps below for the actual Linux distributions I am using on my boards. I compile the module directly on the board.

**Raspberry Pi — Raspbian**

1.    Install the kernel sources for your running kernel, either manually or by using a script such as this: [https://gist.github.com/azbesthu/3893319](https://gist.github.com/azbesthu/38933
2.    Open the `rfm12b_config.h` file, read it carefully and edit the settings to your liking. Be especially sure to correctly choose the board you are building for.
3.    Clone this repository and run `make`
4.    Make sure that the RFM12B module is connect
5.    Load the kernel driver for the SPI interface: `sudo modprobe spi-bcm2708`
6.    Load the RFM12B module: `sudo insmod rfm12b.ko`.
7.    Check the output of `dmesg` and look into `/dev` to make sure that the driver has been loaded successfully.

**Beaglebone (Black) — Ubuntu 13.04, 3.8+ kernel**

1.    Install the kernel sources for your running kernel, either manually or by using [this script](https://github.com/gkaindl/beaglebone-ubuntu-scripts/blob/master/bb-get-rcn-kernel-source.sh) I made.
2.    Ensure that the SPI bus driver is enabled via the device-tree. You can do this manually or by running [another script](https://github.com/gkaindl/beaglebone-ubuntu-scripts/blob/master/bb-enable-spi-devicetree.sh) I made (reboot after running this script).
3.    Open the `rfm12b_config.h` file, read it carefully and edit the settings to your liking. Be especially sure to correctly choose the board you are building for.
4.    Clone this repository and run `make`.
5.    Make sure that the RFM12B module is connected.
6.    Load the RFM12B module: `sudo insmod rfm12b.ko`
7.    Check the output of `dmesg` and look into `/dev` to make sure that the driver has been loaded successfully.
8.    *Hint*: You might also want to ensure that your Beaglebone is not throttling the CPU, since the RFM12B driver is somewhat timing-sensitive for its interrupt handling. I'd recommend running the CPU performance governor: `sudo cpufreq-set -g performance`

## Documentation

The relevant bits of the driver and user-space interface are all documented at the top of the source files, so I'd recommend to look in there.

If you intend to configure your board settings on-the-fly in your user-space programs, you should especially check out the documentation in `rfm12b_ioctl.h`, where all available ioctl() calls are detailed.

There are also a couple of example programs in the `examples` folder: You can build them all by going into this folder and running `make`. The examples are also documented in their source files.

Finally, there are some Arduino examples meant to be run on a [JeeNode](http://jeelabs.net/projects/hardware/wiki/JeeNode), which can work together with the Linux examples.

## JeeLib Compatibility

The driver uses a packet format that is compatible with [JeeLib](https://github.com/jcw/jeelib), e.g. a header byte and a length byte at the beginning of each packet and a trailing CRC16.

During normal operation, the driver will not populate the header byte (it remains at zero), nor will the header be inspected when a packet is received. Also, only the packet payload, but not the two extra bytes or the CRC16 will be exposed to user-space via read().

Thus, the format is entirely compatible with the JeeLib format, but does not interpret its protocol.

There is also a **Jee-compatible** mode that can be enabled by specifying a non-zero **Jee identifier** in the driver. This can be done as a compile-time option in `rfm12b_config.h`, as a module-load parameter or via ioctl().

Details about Jee-compatible mode are available in `rfm12b_config.h` and `rfm12b_ioctl.h`.

Basically, Jee-compatible mode introduces the following differences from normal operation:

1.    When sending a packet via write(), the first two bytes will be interpreted as JeeLib header and length bytes. If you write() less than 2 bytes of data (which would amount to sending an empty packet), write() will return an error in this mode. If you specify an invalid length byte, the driver will fix it for you. You can set the header byte to whatever settings you like (also check JeeLib's documentation).

2.    When receiving a packet via read(), the driver will return the received header and length bytes as the first 2 bytes into the provided data buffer. You can then inspect the header byte and structure your user-space code around it. This way, it becomes possible to emulate the behavior of a JeeNode entirely.

3.    When in Jee-compatible mode, the driver can automatically send ACK packets when a received packets requests one. This behavior is enabled by default, but can be turned off via compile-time option, module-load parameter or ioctl().

## Tested Environments

I've tested the driver on the following setups, usually running it for a couple of days ping-ponging with the rfm12b_echo.c example and a JeeNode running the rfm12b_echo_client sketch.

<table>
  <tr>
    <th>Board</th><th>Distribution</th><th>Kernel Version</th>
  </tr>
  <tr>
    <td>Raspberry Pi</td><td>Raspbian Wheezy</td><td>3.6.11+</td>
  </tr>
  <tr>
    <td>Beaglebone</td><td>Ubuntu 12.04</td><td>3.2.21-psp15</td>
  </tr>
  <tr>
    <td>Beaglebone Black</td><td>Ubuntu 13.04</td><td>3.8.12-bone17</td>
  </tr>
  <tr>
    <td>Beaglebone Black</td><td>Ubuntu 13.04</td><td>3.8.12-bone19</td>
  </tr>
  <tr>
    <td>Beaglebone Black</td><td>Ubuntu 13.04</td><td>3.8.12-bone20</td>
  </tr>
  <tr>
    <td>Beaglebone Black</td><td>Ubuntu 13.04</td><td>3.8.12-bone21</td>
  </tr>
  <tr>
    <td>Beaglebone Black</td><td>Ubuntu 13.04</td><td>3.8.12-bone22</td>
  </tr>
</table>

## Future Work

There are some areas of future work that would make sense.

*    The module currently employs some rather roguish techniques related to the setup, such as forcefully unregistering existing device drivers for the given SPI device (usually spidev), or writing directly to hardware registers to set the pinmux settings (rather than using a proper mechanism like pin-ctrl or device-tree). This makes it much more convenient to use and develop the module, but should be changed in the future.

*    The RFM12B module has a lot of functionality that is currently not addressed by the driver. Such functionality could be added to the driver, such as by adding new ioctl() calls for currently unused settings, etc...

*    The file structure of the sources is a bit… arbitrary. This is due to the history of the driver (I originally bit-banged on a tiny board without an (accessible) hardware SPI interface, which is why I put all the concrete SPI stuff in its own file), etc… This can definitely be cleaned up a lot!