#!/usr/bin/env python3
"""Relay RTCM3 from the base to the rover over USB, bypassing the 433 MHz link.

This is the known-good reference path: it previously took the rover from
hAcc 1.33 m to RTK FLOAT at 0.11 m in under five seconds. Use it to prove the
receivers and the correction stream are fine when the radio is in doubt, and as
a working setup whenever both boards can share a cable.

The base must have RTCM3 on its USB/UART1 output for this to see anything. If
the base only emits RTCM on UART2, run set_rtcm_output.py --port usb on it
first.

    python3 tools/rtcm_usb_relay.py --base /dev/ttyUSB1 --rover /dev/ttyUSB0
"""

import argparse
import sys
import time
from collections import Counter

import serial

import ubx


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--base", required=True, help="base receiver device")
    ap.add_argument("--rover", required=True, help="rover receiver device")
    ap.add_argument("--base-baud", type=int, default=0, help="0 = autodetect")
    ap.add_argument("--rover-baud", type=int, default=0, help="0 = autodetect")
    ap.add_argument("--stats-every", type=float, default=5.0)
    args = ap.parse_args()

    if args.base == args.rover:
        print("Base and rover must be different devices.")
        return 2

    try:
        base = serial.Serial(args.base, args.base_baud or 38400, timeout=0.1)
        rover = serial.Serial(args.rover, args.rover_baud or 38400, timeout=0.1)
    except serial.SerialException as e:
        print("Cannot open port: %s" % e)
        if "Permission denied" in str(e):
            print("sudo usermod -aG dialout $USER   (then log out and back in)")
        return 2

    with base, rover:
        if args.base_baud:
            base.baudrate = args.base_baud
        else:
            b = ubx.detect_baud(base, verbose=False)
            if b is None:
                print("Base %s is silent at every baud." % args.base)
                return 1
            print("Base  %s @ %d" % (args.base, b))

        if args.rover_baud:
            rover.baudrate = args.rover_baud
        else:
            b = ubx.detect_baud(rover, verbose=False)
            if b is None:
                print("Rover %s is silent at every baud." % args.rover)
                return 1
            print("Rover %s @ %d" % (args.rover, b))

        print("\nRelaying RTCM3 base -> rover. Ctrl-C to stop.\n")

        buf = bytearray()
        seen = Counter()
        relayed_bytes = 0
        t_stats = time.time()
        t0 = time.time()

        try:
            while True:
                chunk = base.read(4096)
                if chunk:
                    buf += chunk
                    # Only forward verified RTCM3 frames. Passing raw bytes
                    # through would also hand the rover the base's UBX and NMEA.
                    msgs, buf = ubx.parse_stream(buf)
                    for m in msgs:
                        if m["kind"] != "rtcm3":
                            continue
                        rover.write(m["raw"])
                        relayed_bytes += len(m["raw"])
                        seen[m["type"]] += 1
                    rover.flush()
                else:
                    time.sleep(0.005)

                now = time.time()
                if now - t_stats >= args.stats_every:
                    t_stats = now
                    if seen:
                        mix = ", ".join("%d:%d" % kv for kv in sorted(seen.items()))
                        print("[%6.0fs] relayed %s B, RTCM3 -> %s"
                              % (now - t0, relayed_bytes, mix))
                        if 1005 not in seen:
                            print("           (no 1005 yet: base survey-in has not"
                                  " completed, so FIXED is not reachable)")
                    else:
                        print("[%6.0fs] no RTCM3 from the base yet -- is RTCM3 enabled"
                              " on the base's USB/UART1 output?" % (now - t0))
                    sys.stdout.flush()
        except KeyboardInterrupt:
            print("\nStopped. Relayed %d bytes." % relayed_bytes)

    return 0


if __name__ == "__main__":
    sys.exit(main())
