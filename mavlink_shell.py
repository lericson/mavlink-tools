#!/usr/bin/env python

"""
Open a shell over MAVLink.

@author: Beat Kueng (beat-kueng@gmx.net)
"""


from __future__ import print_function
import sys
import time
import select
import termios

try:
    from pymavlink import mavutil
    import serial
except:
    print("Failed to import pymavlink.")
    print("You may need to install it with 'pip install pymavlink pyserial'")
    print("")
    raise
from argparse import ArgumentParser

mavlink_severity_names = 'EMERGENCY ALERT CRITICAL ERROR WARNING NOTICE INFO DEBUG'.split()

class MavlinkSerialPort():
    '''an object that looks like a serial port, but
    transmits using mavlink SERIAL_CONTROL packets'''
    def __init__(self, portname, baudrate, devnum=0, debug=0):
        self.baudrate = 0
        self._debug = debug
        self.buf = ''
        self.port = devnum
        self.mav = mavutil.mavlink_connection(portname, autoreconnect=True, baud=baudrate)
        self.mav.wait_heartbeat()

    def write(self, b):
        '''write some bytes'''
        while len(b) > 0:
            n = len(b)
            if n > 70:
                n = 70
            buf = [ord(x) for x in b[:n]]
            buf.extend([0]*(70-len(buf)))
            self.mav.mav.serial_control_send(self.port,
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_EXCLUSIVE |
                                             mavutil.mavlink.SERIAL_CONTROL_FLAG_RESPOND,
                                             0,
                                             0,
                                             n,
                                             buf)
            b = b[n:]

    def close(self):
        self.mav.mav.serial_control_send(self.port, 0, 0, 0, 0, [0]*70)

    def _recv(self):
        '''read some bytes into self.buf'''
        m = self.mav.recv_match(condition='SERIAL_CONTROL.count!=0',
                                type='SERIAL_CONTROL', blocking=True,
                                timeout=0.03)
        if m is not None:
            if self._debug > 2:
                print(m)
            data = m.data[:m.count]
            self.buf += ''.join(str(chr(x)) for x in data)

    def read(self, n):
        '''read some bytes'''
        if len(self.buf) == 0:
            self._recv()
        if len(self.buf) > 0:
            if n > len(self.buf):
                n = len(self.buf)
            ret = self.buf[:n]
            self.buf = self.buf[n:]
            return ret
        return ''


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('port', metavar='PORT', nargs='?', default = None,
            help='Mavlink port name: serial: DEVICE[,BAUD], udp: IP:PORT, tcp: tcp:IP:PORT. Eg: \
/dev/ttyUSB0 or 0.0.0.0:14550. Auto-detect serial if not given.')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int,
                      help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()


    if args.port is None:
        if sys.platform == "darwin":
            args.port = "/dev/tty.usbmodem1"
        else:
            serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',
                "*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*", '*PX4*', '*FMU*', "*Gumstix*"])

            if len(serial_list) == 0:
                print("Error: no serial connection found")
                return

            if len(serial_list) > 1:
                print('Auto-detected serial ports are:')
                for port in serial_list:
                    print(" {:}".format(port))
            print('Using port {:}'.format(serial_list[0]))
            args.port = serial_list[0].device


    print("Connecting to MAVLINK...")
    mav_serialport = MavlinkSerialPort(args.port, args.baudrate, devnum=10)

    mav_serialport.write('\n')  # make sure the shell is started

    try:
        next_heartbeat_time = time.time()

        import readline
        lines = []
        prompt = b'\n'
        def readlines():
            try:
                while True:
                    prompt_str = str(prompt[1:], encoding='ascii')
                    # PX4 prints VT102 Erase In Line, hack it out. Otherwise
                    # the Python line editing stuff miscalculates the length of
                    # the prompt.
                    if '\033[K' in prompt_str:
                        prompt_str = prompt_str[:prompt_str.rindex('\x1b[K')]
                    lines.append(input(prompt_str))
            except EOFError:
                lines.append(None)
        import threading
        t = threading.Thread(target=readlines, daemon=True)
        t.start()

        while True:
            if lines:
                cur_line = lines.pop(0)
                if cur_line is None:
                    print('\ncaught EOF, quitting', file=sys.stderr)
                    break
                mav_serialport.write(cur_line+'\n')
            while True:
                m = mav_serialport.mav.recv_match(type=['SERIAL_CONTROL', 'STATUSTEXT'],
                                                  blocking=True, timeout=0.1)
                if m is None:
                    break
                elif m.get_type() == 'SERIAL_CONTROL':
                    if m.count == 0:
                        break
                    data = bytes(m.data[:m.count])
                    prompt += data
                    prompt = prompt[prompt.rindex(b'\n'):]
                    sys.stdout.buffer.write(bytes(data))
                    sys.stdout.flush()
                else:
                    text = str(m.text[:m.text.index(0)], encoding='utf-8')
                    text = text.replace('\n', ' ')
                    text = f'[{mavlink_severity_names[m.severity]:>10s}] {text}'
                    #print(f'\033[2K\r{text}', file=sys.stderr)
                    print(f'\033\067\n\033[1A\r\033[1L{text}\033\070', end='', file=sys.stderr, flush=True)

            # handle heartbeat sending
            heartbeat_time = time.time()
            if heartbeat_time > next_heartbeat_time:
                mav_serialport.mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                        mavutil.mavlink.MAV_AUTOPILOT_GENERIC, 0, 0, 0)
                next_heartbeat_time = heartbeat_time + 1

    except serial.serialutil.SerialException as e:
        print(e)

    except KeyboardInterrupt:
        mav_serialport.close()


if __name__ == '__main__':
    main()

