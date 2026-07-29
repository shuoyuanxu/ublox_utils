#!/usr/bin/env python3
"""Query or factory-reset a HopeRF HM-TRP through its direct UART.

The module must have ENABLE=low and CONFIG=low. Commands are binary:
AA FA E1 queries all settings and AA FA F0 restores factory defaults.
"""

import argparse
import time

import serial

BAUDS = (9600, 19200, 38400, 57600, 115200, 4800, 2400, 1200)
QUERY = b"\xaa\xfa\xe1"
DEFAULT = b"\xaa\xfa\xf0"


def exchange(device, baud, command):
    with serial.Serial(device, baud, timeout=0.4) as port:
        port.reset_input_buffer()
        port.write(command)
        port.flush()
        time.sleep(0.5)
        return port.read(128)


def probe(device):
    for baud in BAUDS:
        response = exchange(device, baud, QUERY)
        if response:
            return baud, response
    return None, b""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("devices", nargs="+")
    parser.add_argument("--defaults", action="store_true")
    parser.add_argument("--air-test", action="store_true")
    args = parser.parse_args()

    if args.air_test:
        if len(args.devices) != 2:
            parser.error("--air-test requires exactly two devices")
        for baud in BAUDS:
            with serial.Serial(args.devices[0], baud, timeout=0.8) as left, \
                    serial.Serial(args.devices[1], baud, timeout=0.8) as right:
                left.reset_input_buffer()
                right.reset_input_buffer()
                marker = b"HMTRP-AIR-%06d" % baud
                left.write(marker)
                left.flush()
                time.sleep(1.0)
                received = right.read(256)
                valid = marker in received
                print("%6d: sent %d, received %d: %s"
                      % (baud, len(marker), len(received),
                         "PASS" if valid else received.hex(" ") or "nothing"))
                if valid:
                    return 0
        return 1

    failed = False
    for device in args.devices:
        baud, response = probe(device)
        if baud is None:
            print("%s: no configuration response at any supported baud" % device)
            failed = True
            continue
        print("%s: configuration mode at %d, query response %s"
              % (device, baud, response.hex(" ")))
        if args.defaults:
            reset_response = exchange(device, baud, DEFAULT)
            print("%s: factory reset response %s"
                  % (device, reset_response.hex(" ") or "<none>"))
            new_baud, new_response = probe(device)
            if new_baud != 9600:
                print("%s: reset verification failed (detected %s)"
                      % (device, new_baud))
                failed = True
            else:
                print("%s: reset verified at 9600, response %s"
                      % (device, new_response.hex(" ")))
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
