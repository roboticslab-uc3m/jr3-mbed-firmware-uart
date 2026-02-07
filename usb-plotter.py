import argparse
import serial
import signal
import threading
import time

from jr3.plotter import Plotter

FULL_SCALES = [115, 108, 185, 56, 52, 61] # GoFa's spare JR3
OP_START = 2
OP_STOP = 3
OP_ZERO_OFFSET = 4
OP_READ = 9

parser = argparse.ArgumentParser()
parser.add_argument('--channel', type=str, default='/dev/ttyLPC1768', help='serial port')
parser.add_argument('--baudrate', type=int, default=115200, help='baud rate')
parser.add_argument('--cutoff', type=float, default=2.0, help='cutoff frequency [Hz]')
parser.add_argument('--period-ms', type=float, default=10.0, help='read period [ms]')
args = parser.parse_args()

ser = serial.Serial(args.channel, args.baudrate)
plotter = Plotter()

should_stop = False

def handler(signum, frame):
    global should_stop
    should_stop = True
    ser.write(build_message(OP_STOP))
    ser.close()

signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)

def parse_ft_message(data):
    forces = [2 * int.from_bytes(data[2*i:2*i+2], 'little', signed=True) / FULL_SCALES[i] for i in range(0, 3)]
    torques = [0.02 * int.from_bytes(data[2*i:2*i+2], 'little', signed=True) / FULL_SCALES[i] for i in range(3, 6)]
    framecounter = data[6]
    return forces, torques, framecounter

def build_message(op: int, data: bytes = bytes()) -> bytearray:
    buffer = bytearray()
    buffer.extend(b'<%02d' % op)

    if len(data) > 0:
        buffer.extend(data)

    buffer.extend(b'>')
    return buffer

def do_read():
    data = int(args.cutoff * 100).to_bytes(2, 'little') + int(args.period_ms * 1000).to_bytes(4, 'little')
    ser.write(build_message(OP_START, data))

    time.sleep(1)
    ser.write(build_message(OP_ZERO_OFFSET))

    while not should_stop:
        try:
            msg = ser.read_until(b'>')
            start_idx = msg.find(b'<')

            if start_idx == -1 or int(msg[start_idx+1:start_idx+3]) != OP_READ:
                continue

            data = msg[start_idx+3:-1]

            if len(data) != 14:
                continue

            forces, moments, fc = parse_ft_message(data)
            plotter.update(forces + moments)
        except Exception as e:
            continue

thread = threading.Thread(target=do_read)
thread.start()

while not should_stop:
    plotter.plot()
