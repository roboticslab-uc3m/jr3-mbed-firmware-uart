import argparse
import collections
import matplotlib.pyplot as plt
import numpy as np
import serial
import signal
import threading
import time

STEPS = 100
TIME_INTERVAL = 0.01 # [s]
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

t = np.arange(0, STEPS)
values = [collections.deque(np.zeros(t.shape)) for i in range(6)]
last_value = [0.0 for i in range(6)]
limits = [(0.0, 0.0) for i in range(2)]

should_stop = False

def handler(signum, frame):
    global should_stop
    should_stop = True
    ser.write(build_message(OP_STOP))
    ser.close()

signal.signal(signal.SIGINT, handler)
signal.signal(signal.SIGTERM, handler)

fig, axes = plt.subplots(1, 2)

axes[0].set_title('forces')
axes[0].set_animated(True)

axes[1].set_title('moments')
axes[1].set_animated(True)

(ln_fx,) = axes[0].plot(values[0], label='x', color='red')
(ln_fy,) = axes[0].plot(values[1], label='y', color='green')
(ln_fz,) = axes[0].plot(values[2], label='z', color='blue')
(ln_mx,) = axes[1].plot(values[3], label='x', color='red')
(ln_my,) = axes[1].plot(values[4], label='y', color='green')
(ln_mz,) = axes[1].plot(values[5], label='z', color='blue')

plt.show(block=False)
plt.pause(0.1)

bg = fig.canvas.copy_from_bbox(fig.bbox)

fig.draw_artist(axes[0])
fig.draw_artist(axes[1])

# https://matplotlib.org/stable/users/explain/animations/blitting.html + https://stackoverflow.com/a/15724978
fig.canvas.blit(fig.bbox)

def hex_to_signed_int(data):
    hex = int(data[1] + data[0], 16)
    return hex - 2**16 if hex > 2**15 - 1 else hex

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

            last_value[0:3], last_value[3:6], fc = parse_ft_message(data)

            limits[0] = (min([limits[0][0]] + last_value[0:3]), max([limits[0][1]] + last_value[0:3]))
            limits[1] = (min([limits[1][0]] + last_value[3:6]), max([limits[1][1]] + last_value[3:6]))

        except Exception as e:
            continue

thread = threading.Thread(target=do_read)
thread.start()

while not should_stop:
    for i in range(len(values)):
        values[i].popleft()
        values[i].append(last_value[i])

    fig.canvas.restore_region(bg)

    ln_fx.set_ydata(values[0])
    ln_fy.set_ydata(values[1])
    ln_fz.set_ydata(values[2])
    ln_mx.set_ydata(values[3])
    ln_my.set_ydata(values[4])
    ln_mz.set_ydata(values[5])

    axes[0].set_ylim(limits[0][0], limits[0][1])
    axes[1].set_ylim(limits[1][0], limits[1][1])

    fig.draw_artist(axes[0])
    fig.draw_artist(axes[1])

    fig.canvas.blit(fig.bbox)
    fig.canvas.flush_events()

    time.sleep(TIME_INTERVAL)
