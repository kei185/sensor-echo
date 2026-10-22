#!/usr/bin/env python3
"""Synchronize the observation firmware, then compare RTC/HAL ticks with the host."""

import argparse
import csv
import glob
import os
from pathlib import Path
import select
import struct
import termios
import time
import tty


def crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x31 if crc & 0x80 else 0)) & 0xFF
    return crc


class Link:
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()
        self.received_ns = self.monotonic_ns = 0

    def send(self, data):
        while data:
            if not select.select([], [self.fd], [], 5)[1]:
                raise TimeoutError("UART write timed out")
            data = data[os.write(self.fd, data):]

    def receive(self, timeout=5):
        deadline = time.monotonic() + timeout
        while True:
            start = self.buffer.find(b"\xaa\x55")
            if start < 0:
                self.buffer[:] = self.buffer[-1:]
            else:
                del self.buffer[:start]
                if len(self.buffer) >= 14:
                    length = int.from_bytes(self.buffer[2:4], "little")
                    if length > 1330 or self.buffer[4] != crc8(self.buffer[2:4]):
                        del self.buffer[0]
                        continue
                    if len(self.buffer) >= 14 + length:
                        frame = bytes(self.buffer[:14 + length])
                        del self.buffer[:14 + length]
                        return (frame[5], int.from_bytes(frame[6:14], "little"),
                                frame[14:], self.received_ns, self.monotonic_ns)

            remaining = deadline - time.monotonic()
            if remaining <= 0 or not select.select([self.fd], [], [], remaining)[0]:
                raise TimeoutError("UART read timed out; reset the board before running")
            chunk = os.read(self.fd, 4096)
            if not chunk:
                raise EOFError("UART disconnected")
            self.received_ns = time.time_ns()
            self.monotonic_ns = time.monotonic_ns()
            self.buffer.extend(chunk)


def observe(link, output, seconds):
    link.send(b"\xaa\xa4")
    while True:
        kind, _, _, received_ns, _ = link.receive()
        if kind == 0x0A:
            link.send(b"\xaa\xa5" + struct.pack(
                "<QQ", received_ns // 1_000_000, time.time_ns() // 1_000_000))
            break

    writer = csv.writer(output)
    writer.writerow([
        "host_unix_ms", "rtc_unix_ms", "offset_ms", "host_elapsed_ms",
        "rtc_elapsed_ms", "rtc_drift_ms", "tick_ms", "tick_elapsed_ms",
        "tick_drift_ms",
    ])
    baseline = None
    tick_elapsed = 0
    while True:
        kind, rtc, payload, received_ns, mono_ns = link.receive()
        if kind != 0x0C or not payload.startswith(b"CLOCK tick_ms="):
            continue
        tick = int(payload.removeprefix(b"CLOCK tick_ms=").strip())
        if baseline is None:
            baseline = (mono_ns, rtc)
            previous_tick = tick
        host_elapsed = (mono_ns - baseline[0]) / 1_000_000
        rtc_elapsed = rtc - baseline[1]
        tick_elapsed += (tick - previous_tick) & 0xFFFFFFFF
        previous_tick = tick
        offset = rtc - received_ns / 1_000_000
        writer.writerow([
            f"{received_ns / 1_000_000:.3f}", rtc, f"{offset:.3f}",
            f"{host_elapsed:.3f}", rtc_elapsed, f"{rtc_elapsed - host_elapsed:.3f}",
            tick, tick_elapsed, f"{tick_elapsed - host_elapsed:.3f}",
        ])
        output.flush()
        print(f"{host_elapsed / 1000:6.1f}s  offset={offset:+.1f}ms  "
              f"RTC drift={rtc_elapsed - host_elapsed:+.1f}ms  "
              f"HAL drift={tick_elapsed - host_elapsed:+.1f}ms", flush=True)
        if host_elapsed >= seconds * 1000:
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--output", type=Path, default=Path(
        time.strftime("/tmp/sensor-echo-clock-%Y%m%d-%H%M%S.csv")))
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    if not args.port:
        ports = sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
        if len(ports) != 1:
            parser.error(f"specify --port; detected: {ports}")
        args.port = ports[0]

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        raw = termios.tcgetattr(fd)
        raw[4] = raw[5] = termios.B230400
        termios.tcsetattr(fd, termios.TCSANOW, raw)
        termios.tcflush(fd, termios.TCIOFLUSH)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        print(f"port={args.port} CSV={args.output}", flush=True)
        with args.output.open("x", newline="") as output:
            observe(Link(fd), output, args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        os.close(fd)


if __name__ == "__main__":
    main()
