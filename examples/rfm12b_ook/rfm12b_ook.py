import fcntl
import array
import struct
import ioctl_opt
import ctypes
import time

DEVNAME = '/dev/rfm12b.0.1'
RFM12B_SPI_MAJOR = ord('r')
RFM12B_OOK_CMD_MAX_LEN = 200


class rfm12b_stats(ctypes.Structure):
    _fields_ = [
        ('bytes_recvd', ctypes.c_ulong),
        ('bytes_sent', ctypes.c_ulong),
        ('pkts_recvd', ctypes.c_ulong),
        ('pkts_sent', ctypes.c_ulong),
        ('num_recv_overflows', ctypes.c_ulong),
        ('num_recv_timeouts', ctypes.c_ulong),
        ('num_recv_crc16_fail', ctypes.c_ulong),
        ('num_send_underruns', ctypes.c_ulong),
        ('num_send_timeouts', ctypes.c_ulong),
        ('low_battery', ctypes.c_ulong),
    ]

    @classmethod
    def get_ioctl_request(cls):
        return ioctl_opt.IOR(
            RFM12B_SPI_MAJOR, 0, ctypes.POINTER(cls))


class rfm12b_ook_cmds(ctypes.Structure):
    _fields_ = [
        ('len', ctypes.c_int),
        ('delay_us', ctypes.c_uint),
        ('cmds', ctypes.c_ubyte * RFM12B_OOK_CMD_MAX_LEN),
    ]

    @classmethod
    def get_ioctl_request(cls):
        return ioctl_opt.IOW(
            RFM12B_SPI_MAJOR, 11, ctypes.POINTER(cls))


class switch(object):
    SYNC = 'S'
    FLOAT = 'F'
    ONE = '1'
    ZERO = '0'

    SW_CMD_ON = 1
    SW_CMD_OFF = 2

    DELAY_CLK = 375
    REPEAT_COUNT = 3

    code = {
        # ZERO: [0b10001000],
        # ONE: [0b11101110],
        # FLOAT: [0b10001110],
        # SYNC: [0b10000000, 0, 0, 0],
        # ZERO: [0b11111111],
        # ONE: [0b11111111],
        # FLOAT: [0b11111111],
        # SYNC: [0b11111111, 0b11111111, 0b11111111, 0b11111111],
        ZERO: [0b11111111],
        ONE: [0b11111111],
        FLOAT: [0b11111111],
        SYNC: [0b11111111, 0b11111111, 0b11111111, 0b01111],
        }

    def __init__(self, system, unit, repeat=1):
        self.system = system
        self.unit = unit
        self.repeat = repeat
        self.rf12 = None

    def __enter__(self):
        self.rf12 = open(DEVNAME, 'wb')
        return self

    def __exit__(self, type_, value, traceback):
        self.rf12.close()

    def _do_cmd(self, cmd):
        frame = (self.system + self.unit + cmd + self.SYNC) * self.repeat
        print len(frame), repr(frame)

        ook = rfm12b_ook_cmds()
        ook.delay_us = 245
        k = 0
        for bit in frame:
            for encd in self.code[bit]:
                ook.cmds[k] = encd
                k += 1
        ook.len = k

        return fcntl.ioctl(self.rf12, ook.get_ioctl_request(), ook)

    def on(self):
        self._do_cmd(self.FLOAT + self.ZERO)

    def off(self):
        self._do_cmd(self.ZERO + self.FLOAT)


def main():
    with switch('0FFFF', '0FFFF', repeat=4) as unita:
        while True:
            unita.on()
            time.sleep(1)
            unita.off()
            time.sleep(1)


if __name__ == '__main__':
    main()
