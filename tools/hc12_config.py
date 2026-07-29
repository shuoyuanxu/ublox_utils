#!/usr/bin/env python3
"""Talk to an HC-12 433 MHz module directly, bypassing the F9P.

Connect a USB-serial adapter to the HC-12's UART breakout (CN7 on these boards,
the 4-pin header directly above the HC-12). Wire adapter TX -> module RX,
adapter RX -> module TX, and GND -> GND. The adapter must be 3.3 V logic.

AT command mode requires the module's SET pin held LOW. On this board SET may be
driven by SW1 or a pull-up; if AT commands do not answer, that is the first
thing to check.

    python3 tools/hc12_config.py --port /dev/ttyUSB2 --read
    python3 tools/hc12_config.py --port /dev/ttyUSB2 --set-baud 9600 --set-channel 1
    python3 tools/hc12_config.py --port /dev/ttyUSB2 --defaults

Two-board air test -- run --listen on one, --transmit on the other:

    python3 tools/hc12_config.py --port /dev/ttyUSB2 --listen
    python3 tools/hc12_config.py --port /dev/ttyUSB3 --transmit

If the air test passes but the F9P still sees nothing, the radios are fine and
the fault is between the module and the F9P. If the air test fails, the radios
are the fault and the F9P was never the problem.
"""

import argparse
import sys
import time

import serial

# HC-12 always speaks AT at 9600 regardless of its configured data baud.
AT_BAUD = 9600
DATA_BAUDS = [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]


def at(ser, cmd, wait=0.4):
    ser.reset_input_buffer()
    ser.write(cmd.encode())
    ser.flush()
    time.sleep(wait)
    return ser.read(256).decode("ascii", "replace").strip()


def read_config(ser):
    print("Querying module (AT mode needs SET held LOW)...\n")
    resp = at(ser, "AT")
    if "OK" not in resp:
        print("  AT -> %r" % resp)
        print("\nNo response. Either the SET pin is not low, the wiring is")
        print("swapped (TX<->RX), or this is not the module's UART.")
        return False
    print("  AT           -> %s" % resp)
    for cmd, label in (("AT+RX", "all parameters"),
                       ("AT+V", "firmware version")):
        r = at(ser, cmd, wait=0.6)
        print("  %-12s -> %s" % (cmd, r.replace("\r\n", " | ")))
    print("\nBoth boards must match on channel (C) and baud (B).")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="USB-serial adapter, NOT the F9P")
    ap.add_argument("--read", action="store_true")
    ap.add_argument("--set-baud", type=int, choices=DATA_BAUDS)
    ap.add_argument("--set-channel", type=int, help="1-127, must match both ends")
    ap.add_argument("--set-power", type=int, choices=range(1, 9),
                    help="1=lowest, 8=+20dBm")
    ap.add_argument("--defaults", action="store_true", help="AT+DEFAULT")
    ap.add_argument("--listen", action="store_true", help="air test receiver")
    ap.add_argument("--transmit", action="store_true", help="air test sender")
    ap.add_argument("--data-baud", type=int, default=9600,
                    help="module's configured data baud, for the air test")
    ap.add_argument("--seconds", type=float, default=30.0)
    args = ap.parse_args()

    baud = args.data_baud if (args.listen or args.transmit) else AT_BAUD
    try:
        ser = serial.Serial(args.port, baud, timeout=0.2)
    except serial.SerialException as e:
        print("Cannot open %s: %s" % (args.port, e))
        return 2

    with ser:
        if args.listen:
            print("Listening on %s @ %d for %.0fs. Ctrl-C to stop."
                  % (args.port, baud, args.seconds))
            got = bytearray()
            t0 = time.time()
            while time.time() - t0 < args.seconds:
                c = ser.read(256)
                if c:
                    got += c
                    print("  +%d bytes: %r" % (len(c), c[:60]))
                    sys.stdout.flush()
            print("\nTotal received: %d bytes" % len(got))
            print("AIR LINK WORKS." if got else
                  "NOTHING RECEIVED -- channel mismatch, baud mismatch, or a dead module.")
            return 0 if got else 1

        if args.transmit:
            print("Transmitting on %s @ %d for %.0fs." % (args.port, baud, args.seconds))
            n = 0
            t0 = time.time()
            while time.time() - t0 < args.seconds:
                msg = b"HC12TEST%04d\n" % (n % 10000)
                ser.write(msg)
                ser.flush()
                n += 1
                time.sleep(0.5)
            print("Sent %d messages." % n)
            return 0

        if args.defaults:
            print("AT+DEFAULT -> %s" % at(ser, "AT+DEFAULT", wait=1.0))
            print("Module reset to 9600 baud, channel 001, FU3, +20dBm.")
            print("Do this on BOTH boards so they match.")
            return 0

        changed = False
        if args.set_baud:
            print("AT+B%d -> %s" % (args.set_baud, at(ser, "AT+B%d" % args.set_baud)))
            changed = True
        if args.set_channel:
            print("AT+C%03d -> %s" % (args.set_channel,
                                      at(ser, "AT+C%03d" % args.set_channel)))
            changed = True
        if args.set_power:
            print("AT+P%d -> %s" % (args.set_power, at(ser, "AT+P%d" % args.set_power)))
            changed = True
        if changed:
            print("\nApply the SAME settings to the other board.")
            return 0

        read_config(ser)
    return 0


if __name__ == "__main__":
    sys.exit(main())
